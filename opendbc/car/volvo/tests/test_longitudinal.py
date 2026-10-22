from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import all_frames
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams as P

STATE = structs.CarControl.Actuators.LongControlState


def frame_bytes(data):
  return {f"Byte_{i}": value for i, value in enumerate(data)}


def fsm4(value=0xb408):
  return {"Byte_0": 0x55, "Byte_1": 0xf1, "Byte_2": 0, "ACC_LeadSpeed": 0,
          "Byte_4": 0x30, "Byte_5": value >> 8, "Byte_6": value & 255, "Byte_7": 0}


def fixture():
  cp = structs.CarParams.new_message(openpilotLongitudinalControl=True, minSteerSpeed=1.)
  controller = CarController({Bus.pt: "volvo_v60_2015_pt"}, cp)
  cc = structs.CarControl.new_message(enabled=True, longActive=True)
  cc.actuators.accel = -1.0
  cc.actuators.longControlState = STATE.pid
  cs = SimpleNamespace(
    out=structs.CarState.new_message(vEgo=15., vEgoRaw=15., cruiseState={"enabled": True}),
    acc_distance=255, ACC_Check=0, ecm_acc_engaged=True,
    stock_FSM3={"ACC_Standstill": 0, "ACC_Check": 0, "Byte_01": 0x15, "Byte_02": 0, "Byte_2": 0xd6,
                "Byte_3": 0x58, "Byte_4": 0x81, "Byte_5": 8, "Byte_6": 0, "Byte_7": 0,
                "ACC_AccelerationRequest": 1.0},
    stock_FSM0_frames=[frame_bytes(bytes.fromhex("0000860000000000"))],
    stock_FSM1={"ACC_Distance": 255, **{f"Byte_{i}": 0 for i in range(1, 8)}},
    stock_FSM4_frames=[fsm4()],
    stock_133_frames=[frame_bytes(bytes.fromhex("1122334455667788"))],
    stock_323_frames=[frame_bytes(bytes.fromhex("1122334455667788"))],
    stock_220_frames=[frame_bytes(bytes.fromhex("aabbc4dd11f700ff"))],
    stock_299_frames=[frame_bytes(bytes.fromhex("aabbc4dd11f700ff"))],
    pscm_stock_values=dict.fromkeys(("byte0", "byte4", "byte7", "LKAActive", "EPSTorque"), 0),
  )
  return controller, cc, cs


def update(controller, cc, cs, frame=0):
  act, sends = controller.update(cc.as_reader(), cs, 1_000_000_000 + frame * 20_000_000)
  return act, {addr: dat for addr, dat, _ in sends}


def test_op_only_braking_and_mask_preserve_other_fields():
  controller, cc, cs = fixture()
  act, frames = update(controller, cc, cs)
  assert act.accel == -1.
  assert frames[0x270][1:4] == bytes([101, 0xd4, 0])
  assert frames[0x31a][5] & 0x40
  assert frames[0x220] == bytes.fromhex("aabbc0dd11f700ff")
  assert frames[0x299] == bytes.fromhex("aabbc4dd11ff00ff")
  assert frames[0x133] == frames[0x323] == bytes.fromhex("1122334455667788")
  assert frames[0x260][0] == 255
  assert not {0x5c0, 0x5c1}.intersection(frames)


@pytest.mark.parametrize("camera_accel,honor", [(-0.4, False), (-1.96, False), (-2., False), (-2.04, True), (-3., True)])
def test_threshold_and_fabricated_acknowledgements(camera_accel, honor):
  controller, cc, cs = fixture()
  cc.actuators.accel = 0.5
  cs.stock_FSM3["ACC_AccelerationRequest"] = camera_accel
  cs.stock_FSM4_frames = [fsm4(0xf300)]
  cs.stock_220_frames = [frame_bytes(bytes.fromhex("aabbc0dd11ff00ff"))]
  cs.stock_299_frames = cs.stock_220_frames
  act, frames = update(controller, cc, cs)
  assert controller.cam_brake_honor == honor
  assert act.accel == pytest.approx(camera_accel if honor else 0.5)
  assert bool(frames[0x31a][5] & 0x40) == honor
  if not honor:
    assert frames[0x220] == bytes.fromhex("aabbc4dd11ff00ff")
    assert frames[0x299] == bytes.fromhex("aabbc0dd11f700ff")


def test_strong_brake_latch_releases_only_when_fsm_flag_clears():
  controller, cc, cs = fixture()
  cc.actuators.accel = 0.5
  cs.stock_FSM4_frames = [fsm4(0xf300)]
  for i, accel in enumerate((-2.04, -0.4, 0.1)):
    cs.stock_FSM3["ACC_AccelerationRequest"] = accel
    update(controller, cc, cs, i)
    assert controller.cam_brake_honor
  cs.stock_FSM4_frames = [fsm4()]
  update(controller, cc, cs, 4)
  assert not controller.cam_brake_honor


def test_spoof_ack_release_at_13280_while_wheel_ack_stays_set():
  controller, cc, cs = fixture()
  cc.actuators.accel = 0.5
  cs.stock_FSM3["ACC_AccelerationRequest"] = -0.4
  cs.stock_220_frames = [frame_bytes(bytes(8))]
  cs.stock_299_frames = [frame_bytes(bytes.fromhex("0000000000080000"))]
  for i, target in enumerate((13279, 13280, 13281)):
    cs.stock_FSM4_frames = [fsm4(0xc000 | target)]
    _, frames = update(controller, cc, cs, i)
    assert frames[0x220][2] & 4
    assert bool(frames[0x299][5] & 8) == (target >= 13280)


def test_camera_stop_handover_and_return():
  controller, cc, cs = fixture()
  cs.out.vEgo = 2.
  cs.acc_distance = 8
  cs.stock_FSM3.update(ACC_AccelerationRequest=-0.8, Byte_2=0xd4, Byte_3=0)
  cs.stock_FSM4_frames = [fsm4(0xf357)]
  cc.actuators.longControlState = STATE.stopping
  for i in range(50):
    _, frames = update(controller, cc, cs, i)
  assert controller.cam_relay
  assert frames[0x31a] == controller.packer_pt.make_can_msg("FSM4", 0, cs.stock_FSM4_frames[0])[1]
  cc.actuators.longControlState = STATE.pid
  cc.actuators.accel = -2.0
  update(controller, cc, cs, 50)
  assert not controller.cam_relay and controller.accel == -2.0
  cs.out.vEgo = 5.
  for i in range(51, 102):
    update(controller, cc, cs, i)
  assert controller.cam_stop_w == 0.


def test_native_relays_when_long_inactive_and_no_camera_abort_takeover():
  controller, cc, cs = fixture()
  cc.longActive = False
  cs.ecm_acc_engaged = False
  cs.stock_FSM4_frames = [fsm4(0xf300)]
  _, frames = update(controller, cc, cs)
  assert not frames[0x51][2] & 4
  assert frames[0x31a] == controller.packer_pt.make_can_msg("FSM4", 0, cs.stock_FSM4_frames[0])[1]
  assert not controller.op_owns_acc
  assert not P.OP_OWNS_ACC_AFTER_CAMERA_ABORT


def test_resume_burst_matches_upstream():
  controller, cc, cs = fixture()
  cs.out.vEgo = 0.
  cs.out.cruiseState.standstill = True
  cs.acc_distance = 8
  controller.frame = 101
  sends = []
  controller.resume_buttons(cc, cs, sends)
  assert not sends
  cs.acc_distance = 9  # source triggers on any growth, not our former 2 m gate
  controller.resume_buttons(cc, cs, sends)
  assert len(sends) == 15 and all(addr == 0x127 for addr, _, _ in sends)
  assert controller.sng_ack_frames == 25


def test_parser_preserves_batch_cadence():
  parser = CANParser("volvo_v60_2015_pt", [("FSM4", 33)], 2)
  native = bytes.fromhex("55f1000030b40800")
  second = bytes.fromhex("aaf1000030b40800")
  parser.update([(1_000_000_000, [(0x31a, native, 2), (0x31a, second, 2)])])
  packer = CANPacker("volvo_v60_2015_pt")
  assert [volvocan.create_fsm4(packer, f, 0., None)[1] for f in all_frames(parser, "FSM4")] == [native, second]
  parser.update([(1_010_000_000, [])])
  assert all_frames(parser, "FSM4") == []


def test_acceleration_sweep_and_new_relay_messages_pass_compiled_panda():
  from opendbc.safety.tests.libsafety import libsafety_py
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(structs.CarParams.SafetyModel.volvo, 0)
  safety.init_tests()
  controller, cc, cs = fixture()
  engine = controller.packer_pt.make_can_msg("EngineData", 0, {"ACC_State_ECM": 3})
  safety.safety_rx_hook(libsafety_py.make_CANPacket(engine[0], engine[2], engine[1]))
  for i, accel in enumerate(np.linspace(-4., 2., 601)):
    cc.actuators.accel = float(accel)
    _, sends = controller.update(cc.as_reader(), cs, 1_000_000_000 + i * 20_000_000)
    for addr, dat, bus in sends:
      assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (accel, hex(addr))


@pytest.mark.parametrize("enabled", [False, True])
def test_alpha_longitudinal_setting_is_respected(enabled):
  from opendbc.car.volvo.interface import CarInterface
  from opendbc.car.volvo.values import CAR
  cp = CarInterface.get_params(CAR.VOLVO_V60, {}, [], enabled, False, False)
  assert cp.alphaLongitudinalAvailable
  assert cp.openpilotLongitudinalControl == enabled
  if enabled:
    assert cp.longitudinalActuatorDelay == pytest.approx(0.5)
    assert list(cp.longitudinalTuning.kiV) == [0.5]
