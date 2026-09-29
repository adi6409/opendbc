#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.volvo.volvocan import calculate_lka_checksum, create_esr_simulation
from opendbc.car.volvo.virtual_target import VirtualBrakeTarget
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestVolvoSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[0x51, 0], [0x127, 0], [0x246, 2], [0x260, 0], [0x262, 0], [0x270, 0], [0x5C0, 1], [0x5C1, 0]]
  GAS_PRESSED_THRESHOLD = 10
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: [0x262], 2: [0x246]}
  FWD_BLACKLISTED_ADDRS = {2: [0x262], 0: [0x246]}

  VOLVO_MAIN_BUS = 0
  VOLVO_CAM_BUS = 2

  STEER_ANGLE_MAX = 90
  DEG_TO_CAN = 100
  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]
  ANGLE_RATE_DOWN = [5., 3.5, .4]

  def setUp(self):
    self.packer = CANPackerSafety("volvo_v60_2015_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvo, 0)
    self.safety.init_tests()

  def _angle_meas_msg(self, angle: float):
    return self.packer.make_can_msg_safety(
      "PSCM1", self.VOLVO_MAIN_BUS, {"SteeringAngleServo": angle},
    )

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {
      "LKAAngleReq": angle,
      "LKASteerDirection": 1 if enabled else 0,
      "TrqLim": 0,
      "SET_X_22": 0x25,
      "SET_X_02": 0x00,
      "SET_X_10": 0x10,
      "SET_X_A4": 0xA7,
    }
    dat = self.packer.make_can_msg("FSM2", self.VOLVO_MAIN_BUS, values)[1]
    values["Checksum"] = calculate_lka_checksum(dat)
    return self.packer.make_can_msg_safety("FSM2", self.VOLVO_MAIN_BUS, values)

  @unittest.skip("Volvo control addresses overlap unrelated platforms")
  def test_tx_hook_on_wrong_safety_mode(self):
    pass

  @unittest.skip("PSCM and FSM steering-angle signals have different resolutions")
  def test_angle_cmd_when_disabled(self):
    pass

  @unittest.skip("PSCM and FSM steering-angle signals have different resolutions")
  def test_angle_cmd_when_enabled(self):
    pass

  def test_inactive_angle_matches_quantized_measurement(self):
    self.safety.set_controls_allowed(False)
    for angle in range(-90, 91, 10):
      self._reset_angle_measurement(angle)
      measured_angle = self.safety.get_angle_meas_min() / self.DEG_TO_CAN
      self.assertTrue(self._tx(self._angle_cmd_msg(measured_angle, False)))

  def test_direction_handoff_keeps_tracking_target_angle(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)

    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    # EUCD direction changes require eight NONE frames that retain the target.
    # These are angle-control handoff frames, not inactive steering commands.
    for angle in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6):
      self.assertTrue(self._tx(self._angle_cmd_msg(angle, False)))
    self.assertTrue(self._tx(self._angle_cmd_msg(1.8, True)))

    # NONE still cannot carry an angle away from measurement when controls
    # are not allowed.
    self.safety.set_controls_allowed(False)
    self._reset_angle_measurement(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(1.0, False)))

  def _pcm_status_msg(self, enable, available=False):
    return self.packer.make_can_msg_safety(
      "FSM0", self.VOLVO_CAM_BUS, {"ACC_Enabled": int(enable), "ACC_Available": int(available)},
    )

  def _speed_msg(self, speed: float):
    return self.packer.make_can_msg_safety(
      "VehicleSpeed1", self.VOLVO_MAIN_BUS, {"VehicleSpeed": speed * 3.6},
    )

  def _vehicle_moving_msg(self, speed: float):
    return self._speed_msg(0 if speed <= self.STANDSTILL_THRESHOLD else 10)

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety(
      "Brake_Info", self.VOLVO_MAIN_BUS, {"BrakePedal": 2 if brake else 0},
    )

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety(
      "AccPedal", self.VOLVO_MAIN_BUS, {"AccPedal": 10 if gas else 0},
    )

  def _fsm3_accel_msg(self, accel, acc_check=0):
    return self.packer.make_can_msg_safety(
      "FSM3", self.VOLVO_MAIN_BUS,
      {"ACC_AccelerationRequest": accel, "ACC_Check": acc_check},
    )

  def _esr_sim_msg(self, status=1, range_m=20, range_rate=-2.0, range_accel=-1.0):
    msg = create_esr_simulation(None, status, range_m, range_rate, range_accel)
    return libsafety_py.make_CANPacket(msg.address, msg.src, msg.dat)

  def _esr_auth_msg(self):
    return common.make_msg(0, 0x5C1, dat=b"VLS1\x00\x00\x00\x00")

  def _esr_pre_set_auth_msg(self):
    return common.make_msg(0, 0x5C1, dat=b"VLP1\x00\x00\x00\x00")

  def test_pre_set_simulation_requires_stock_available_and_strict_geometry(self):
    self.safety.set_controls_allowed(False)
    self._rx(self._pcm_status_msg(False, available=True))
    for _ in range(6):
      self._rx(self._speed_msg(5.0))
    self.assertFalse(self._tx(self._esr_sim_msg(status=1, range_m=40, range_rate=0, range_accel=0)))
    for timestamp in range(10_000, 270_000, 10_000):
      self.safety.set_timer(timestamp)
      if timestamp % 50_000 == 0:
        self._rx(self._pcm_status_msg(False, available=True))
        self._rx(self._speed_msg(5.0))
      self.assertFalse(self._tx(self._esr_pre_set_auth_msg()))
    self.assertFalse(self._tx(self._esr_sim_msg(status=1, range_m=20, range_rate=0, range_accel=0)))
    self.assertTrue(self._tx(self._esr_sim_msg(status=1, range_m=40, range_rate=0, range_accel=0)))
    self.assertFalse(self._tx(self._esr_sim_msg(status=2, range_m=40, range_rate=-1, range_accel=0)))
    self._rx(self.packer.make_can_msg_safety("CCButtons", 0, {"ACCSetBtn": 1}))
    self.assertFalse(self._tx(self._esr_sim_msg(status=2, range_m=40, range_rate=0, range_accel=0)))
    self._rx(self.packer.make_can_msg_safety("CCButtons", 0, {"ACCSetBtn": 0}))
    self.assertFalse(self._tx(self._esr_sim_msg(status=2, range_m=40, range_rate=0, range_accel=0)))
    self.assertTrue(self._tx(common.make_msg(1, 0x5C0, dat=b"\x00" * 8)))

  def test_fsm3_longitudinal_limits(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._fsm3_accel_msg(2.0, acc_check=1)))
    self.assertFalse(self._tx(self._fsm3_accel_msg(2.04, acc_check=1)))
    self.assertTrue(self._tx(self._fsm3_accel_msg(-4.0)))
    self.assertFalse(self._tx(self._fsm3_accel_msg(-4.04)))

  def test_esr_simulation_requires_sustained_authorization_and_braking(self):
    self.safety.set_controls_allowed(True)
    for _ in range(6):
      self._rx(self._speed_msg(10.0))
    for timestamp in range(10_000, 220_000, 10_000):
      self.safety.set_timer(timestamp)
      self.assertFalse(self._tx(self._esr_auth_msg()))
      self.assertTrue(self._tx(self._fsm3_accel_msg(-1.0)))
    self.assertTrue(self._tx(self._esr_sim_msg()))
    self.assertFalse(self._tx(self._esr_sim_msg()))
    self.assertTrue(self._tx(common.make_msg(1, 0x5C0, dat=b"\x00" * 8)))

  def test_host_brake_target_lease_precedes_first_panda_accepted_frame(self):
    target = VirtualBrakeTarget()
    self.safety.set_controls_allowed(True)
    for timestamp in range(10_000, 710_000, 10_000):
      self.safety.set_timer(timestamp)
      if timestamp % 50_000 == 0:
        self._rx(self._speed_msg(10.0))
        self.assertTrue(self._tx(self._fsm3_accel_msg(-1.0)))
      output = target.update(
        timestamp * 1000, automatic_braking=True, controls_active=True,
        stock_acc_enabled=True, gas_pressed=False, brake_pressed=False,
        speed=10.0, accel_request=-1.0,
        phase_end_nanos=699_000_000 if timestamp == 700_000 else 0,
        scan_start_nanos=680_000_000 if timestamp == 700_000 else 0,
        native_adopted=False, physical_response=False, native_distance=20.0,
      )
      if output.authorization:
        self.assertFalse(self._tx(self._esr_auth_msg()))
      if output.frame is not None:
        self.assertEqual(output.frame.status, 1)
        self.assertTrue(self._tx(self._esr_sim_msg(
          status=output.frame.status, range_m=output.frame.range_m,
          range_rate=output.frame.range_rate, range_accel=output.frame.range_accel,
        )))
        return
    self.fail("brake target never produced its first radar-sweep frame")

  def test_esr_simulation_rejects_bad_geometry(self):
    self.safety.set_controls_allowed(True)
    for _ in range(6):
      self._rx(self._speed_msg(10.0))
    self.assertFalse(self._tx(self._esr_sim_msg(range_m=5)))
    # INVALID is encoded as the exact-zero retirement accepted in all states.
    self.assertTrue(self._tx(self._esr_sim_msg(status=0)))
    self.assertFalse(self._tx(common.make_msg(1, 0x5C0, dat=b"\x20" + b"\x00" * 7)))

  def test_stock_fsm_forwarding_is_replaced_only_during_control(self):
    for addr in (0x51, 0x260, 0x270):
      self.safety.set_controls_allowed(False)
      self.assertEqual(self.VOLVO_MAIN_BUS, self.safety.safety_fwd_hook(self.VOLVO_CAM_BUS, addr))
      self.safety.set_controls_allowed(True)
      self.assertEqual(-1, self.safety.safety_fwd_hook(self.VOLVO_CAM_BUS, addr))

    self._rx(self._user_gas_msg(True))
    for addr in (0x51, 0x260, 0x270):
      self.assertEqual(self.VOLVO_MAIN_BUS, self.safety.safety_fwd_hook(self.VOLVO_CAM_BUS, addr))


if __name__ == "__main__":
  unittest.main()
