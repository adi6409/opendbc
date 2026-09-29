from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.carcontroller import clip_longitudinal_accel
from opendbc.car.volvo.virtual_target import PreEngageTarget, VirtualBrakeTarget
from opendbc.car.volvo.values import Bus, CAR


def stock_fsm3_values():
  return {
    "ACC_Standstill": 0,
    "Byte_01": 0x15,
    "Byte_02": 0,
    "Byte_2": 0xd6,
    "Byte_3": 0x58,
    "Byte_4": 0x81,
    "Byte_5": 0x08,
    "Byte_6": 0,
    "Byte_7": 0,
  }


def test_takeoff_accel_is_bounded_to_panda_safety_limits():
  assert clip_longitudinal_accel(2.08) == 2.0
  assert clip_longitudinal_accel(-4.08) == -4.0
  assert clip_longitudinal_accel(0.75) == 0.75


def test_optional_can_messages_do_not_invalidate_vehicle_state_when_absent():
  parsers = CarState.get_can_parsers(SimpleNamespace(carFingerprint=CAR.VOLVO_V60))
  pt_optional = next(state for state in parsers[Bus.pt].message_states.values() if state.name == "CCButtons")
  body_optional = next(state for state in parsers[Bus.body].message_states.values() if state.name == "ESR_Sim1_5C0")

  assert pt_optional.ignore_alive
  assert body_optional.ignore_alive


def test_resume_ack_is_stock_shaped_instead_of_zero_filled():
  packer = CANPacker("volvo_v60_2015_pt")
  stock = stock_fsm3_values()
  _, normal_dat, _ = volvocan.create_longitudinal(packer, stock, accel=2.0, acc_check=0)
  _, ack_dat, _ = volvocan.create_acc_state_msg(packer, stock, accel=2.0)

  assert ack_dat[0] == normal_dat[0] | 0x04
  assert ack_dat[1:] == normal_dat[1:]
  assert ack_dat[1] == 0xb0
  assert ack_dat != bytes.fromhex("0400000000000000")


def test_fsm1_pass_through_is_bit_exact():
  packer = CANPacker("volvo_v60_2015_pt")
  stock = {
    "ACC_Distance": 255,
    "Byte_1": 255,
    "Byte_2": 0xb8,
    "Byte_3": 0,
    "Byte_4": 0x49,
    "Byte_5": 0xe3,
    "Byte_6": 0x74,
    "Byte_7": 0x08,
  }
  _, dat, _ = volvocan.create_radar(packer, stock)
  assert dat == bytes.fromhex("ffffb80049e37408")


def test_esr_simulation_encoder_matches_delphi_byte_layout():
  packer = CANPacker("ESR")
  _, dat, bus = volvocan.create_esr_simulation(
    packer, volvocan.ESR_SIM_STATUS_NEW, range_m=24, range_rate=-3.0,
    range_accel=-1.5, angle_deg=0.5, lateral_position=-0.25, lateral_rate=0.25,
  )
  assert bus == volvocan.CANBUS.body
  assert dat == bytes([0x28, 0x01, 0xFF, 0x01, 0x18, 0xFA, 0xF4, 0x00])


def test_esr_simulation_retirement_is_exact_zero():
  msg = volvocan.create_esr_simulation_retirement()
  assert (msg.address, msg.dat, msg.src) == (0x5C0, b"\x00" * 8, volvocan.CANBUS.body)
  assert volvocan.create_esr_simulation(CANPacker("ESR"), volvocan.ESR_SIM_STATUS_INVALID, 0, 0, 0) == msg


def test_radar_seed_uses_selected_stationary_track_relative_to_ego_speed():
  def track(range_m, angle, range_rate):
    return {
      "CAN_TX_TRACK_STATUS": 3,
      "CAN_TX_TRACK_RANGE": range_m,
      "CAN_TX_TRACK_ANGLE": angle,
      "CAN_TX_TRACK_RANGE_RATE": range_rate,
      "CAN_TX_TRACK_RANGE_ACCEL": 0.0,
      "CAN_TX_TRACK_LAT_RATE": 0.0,
    }

  body_cp = SimpleNamespace(vl={
    "ESR_Output_InPath": {
      "CAN_TX_PATH_ID_ACC_STAT": 8,
      "CAN_TX_PATH_ID_FCW_STAT": 0,
      "CAN_TX_PATH_ID_CMBB_STAT": 0,
    },
    "Target8": track(30.0, 0.5, -8.0),
    "Target9": track(10.0, 0.5, 0.0),
  })
  for i in range(1, 65):
    body_cp.vl.setdefault(f"Target{i}", track(0.0, 0.0, 0.0) | {"CAN_TX_TRACK_STATUS": 0})

  seed = CarState._select_radar_seed(body_cp, speed=8.0)
  assert seed is not None
  assert seed.range_m == 30.0
  assert seed.range_rate == -8.0


def test_native_adoption_requires_preexisting_signature_to_clear():
  state = object.__new__(CarState)
  state.stock_FSM0 = {"ACC_FrontCar": 1}
  state.stock_FSM1 = {"Byte_1": 1, "ACC_Distance": 30.0}
  state.stock_FSM3 = {"ACC_AccelerationRequest": -1.0}
  state.stock_FSM4 = {"NEW_SIGNAL_2": 0xB, "Byte_2": 0}
  state.virtual_target_new_nanos = 100
  state.virtual_target_target_range = 30.0
  state.virtual_target_preexisting_signature = True
  state.virtual_target_seen_signature_clear = False
  state.physical_brake_response = False
  state.physical_brake_since_nanos = 0
  cp = SimpleNamespace(ts_nanos={
    "FSM0": {"ACC_FrontCar": 200},
    "FSM1": {"Byte_1": 200},
    "FSM3": {"ACC_AccelerationRequest": 200},
    "FSM4": {"NEW_SIGNAL_2": 200, "Byte_2": 200},
  })
  assert not state._native_adoption_evidence(cp, 200)
  state.stock_FSM0["ACC_FrontCar"] = 0
  assert not state._native_adoption_evidence(cp, 210)
  state.stock_FSM0["ACC_FrontCar"] = 1
  assert state._native_adoption_evidence(cp, 220)


def test_virtual_target_debounces_intent_and_sends_one_new_frame():
  target = VirtualBrakeTarget()
  common = dict(
    diagnostic_trigger=False,
    controls_active=True, stock_acc_enabled=True, gas_pressed=False,
    brake_pressed=False, speed=10.0, accel_request=-1.0,
    scan_start_nanos=0, native_adopted=False, physical_response=False,
    native_distance=20.0, radar_seed=None,
  )
  assert target.update(1, phase_end_nanos=0, **{**common, "diagnostic_trigger": True}).frame is None
  assert target.update(200_000_001, phase_end_nanos=190_000_001, **common).frame is None
  assert target.update(450_000_001, phase_end_nanos=440_000_001, **common).frame is None
  first = target.update(700_000_001, phase_end_nanos=690_000_001, **common)
  assert first.frame is not None and first.frame.status == volvocan.ESR_SIM_STATUS_NEW
  second = target.update(750_000_001, phase_end_nanos=740_000_001, **common)
  assert second.frame is not None and second.frame.status == volvocan.ESR_SIM_STATUS_UPDATED
  assert target.new_nanos == 700_000_001


def test_virtual_target_does_not_arm_without_explicit_trigger():
  target = VirtualBrakeTarget()
  common = dict(
    controls_active=True, stock_acc_enabled=True, gas_pressed=False,
    brake_pressed=False, speed=10.0, accel_request=-1.0,
    phase_end_nanos=0, scan_start_nanos=0, native_adopted=False,
    physical_response=False, native_distance=20.0, radar_seed=None,
  )
  for now in (1, 200_000_001, 450_000_001, 700_000_001):
    assert target.update(now, **common).frame is None
  assert target.stats.trigger_accepted == 0
  assert target.state.name == "IDLE"


def test_virtual_target_automatically_arms_on_sustained_braking():
  target = VirtualBrakeTarget()
  common = dict(
    automatic_braking=True, controls_active=True, stock_acc_enabled=True,
    gas_pressed=False, brake_pressed=False, speed=10.0,
    accel_request=-1.0, scan_start_nanos=0, native_adopted=False,
    physical_response=False, native_distance=20.0, radar_seed=None,
  )
  assert target.update(1, phase_end_nanos=0, **common).frame is None
  assert target.update(450_000_001, phase_end_nanos=440_000_001, **common).frame is None
  output = target.update(700_000_001, phase_end_nanos=690_000_001, **common)
  assert output.frame is not None and output.frame.status == volvocan.ESR_SIM_STATUS_NEW


def test_virtual_target_authorizes_before_first_radar_sweep_frame():
  target = VirtualBrakeTarget()
  common = dict(
    automatic_braking=True, controls_active=True, stock_acc_enabled=True,
    gas_pressed=False, brake_pressed=False, speed=10.0,
    accel_request=-1.0, scan_start_nanos=0, native_adopted=False,
    physical_response=False, native_distance=20.0, radar_seed=None,
  )
  target.update(1, phase_end_nanos=0, **common)
  for t in (450_000_001, 550_000_001, 650_000_001):
    output = target.update(t, phase_end_nanos=0, **common)
    assert output.authorization and output.frame is None
  output = target.update(700_000_001, phase_end_nanos=690_000_001, **common)
  assert output.frame is not None and output.frame.status == volvocan.ESR_SIM_STATUS_NEW


def test_pre_engage_target_retires_on_set_and_does_not_reappear():
  target = PreEngageTarget()
  common = dict(
    controls_active=False, stock_acc_available=True, stock_acc_enabled=False,
    set_pressed=False, gas_pressed=False, brake_pressed=False, speed=5.0,
    native_lead=False, scan_start_nanos=0,
  )
  assert target.update(1, phase_end_nanos=0, **common).frame is None
  output = target.update(260_000_001, phase_end_nanos=250_000_001, **common)
  assert output.frame is not None and output.frame.status == volvocan.ESR_SIM_STATUS_NEW
  output = target.update(300_000_001, phase_end_nanos=290_000_001,
                         **{**common, "set_pressed": True})
  assert output.frame is not None and output.frame.status == volvocan.ESR_SIM_STATUS_INVALID
  for i in range(10):
    output = target.update(310_000_001 + i * 10_000_000,
                           phase_end_nanos=300_000_001 + i * 10_000_000, **common)
    assert output.frame is None or output.frame.status == volvocan.ESR_SIM_STATUS_INVALID


def test_pre_engage_target_can_start_after_departure_from_braked_standstill():
  target = PreEngageTarget()
  common = dict(
    controls_active=False, stock_acc_available=True, stock_acc_enabled=False,
    set_pressed=False, gas_pressed=False, brake_pressed=False, speed=5.0,
    native_lead=False, phase_end_nanos=0, scan_start_nanos=0,
  )
  target.update(1, **{**common, "speed": 0.0, "brake_pressed": True})
  target.update(100_000_001, **{**common, "speed": 0.5, "gas_pressed": True})
  assert not target.exhausted

  assert target.update(200_000_001, **common).authorization
  output = target.update(500_000_001, **{**common, "phase_end_nanos": 490_000_001, "scan_start_nanos": 480_000_001})
  assert output.frame is not None and output.frame.status == volvocan.ESR_SIM_STATUS_NEW

  output = target.update(510_000_001, **{**common, "brake_pressed": True})
  assert output.frame is not None and output.frame.status == volvocan.ESR_SIM_STATUS_INVALID
  assert target.exhausted
  assert not target.update(550_000_001, **common).authorization


def test_pre_engage_target_retires_on_missing_phase():
  target = PreEngageTarget()
  common = dict(
    controls_active=False, stock_acc_available=True, stock_acc_enabled=False,
    set_pressed=False, gas_pressed=False, brake_pressed=False, speed=5.0,
    native_lead=False, scan_start_nanos=0,
  )
  target.update(1, phase_end_nanos=0, **common)
  assert target.update(260_000_001, phase_end_nanos=250_000_001, **common).frame.status == 1
  output = target.update(400_000_001, phase_end_nanos=250_000_001, **common)
  assert output.frame is not None and output.frame.status == 0
  assert target.exhausted


def test_virtual_target_retires_on_control_loss_and_is_bounded():
  target = VirtualBrakeTarget()
  common = dict(
    diagnostic_trigger=False,
    controls_active=True, stock_acc_enabled=True, gas_pressed=False,
    brake_pressed=False, speed=10.0, accel_request=-1.0,
    scan_start_nanos=0, native_adopted=False, physical_response=False,
    native_distance=20.0, radar_seed=None,
  )
  target.update(1, phase_end_nanos=0, **{**common, "diagnostic_trigger": True})
  target.update(450_000_001, phase_end_nanos=440_000_001, **common)
  target.update(700_000_001, phase_end_nanos=690_000_001, **common)
  released = target.update(710_000_001, phase_end_nanos=700_000_001,
                           **{**common, "controls_active": False})
  assert released.cancel is False
  assert target.state.name == "RELEASING"
  for i in range(20):
    target.update(720_000_001 + i * 10_000_000, phase_end_nanos=0, **{**common, "controls_active": False})
  assert target.state.name == "IDLE"


def test_cancel_pulse_does_not_survive_retirement():
  target = VirtualBrakeTarget()
  common = dict(
    diagnostic_trigger=False,
    controls_active=True, stock_acc_enabled=True, gas_pressed=False,
    brake_pressed=False, speed=10.0, accel_request=-1.0,
    scan_start_nanos=0, native_adopted=False, physical_response=False,
    native_distance=20.0, radar_seed=None,
  )
  target.update(1, phase_end_nanos=0, **{**common, "diagnostic_trigger": True})
  target.update(450_000_001, phase_end_nanos=440_000_001, **common)
  target.update(700_000_001, phase_end_nanos=690_000_001, **common)
  target._begin_release(cancel=True)
  for i in range(20):
    target.update(710_000_001 + i * 10_000_000, phase_end_nanos=0,
                 **{**common, "controls_active": False})
  assert target.state.name == "IDLE"
  assert target.update(1_000_000_001, phase_end_nanos=0, **common).cancel is False
