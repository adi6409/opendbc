#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.volvo.volvocan import calculate_lka_checksum
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestVolvoSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[0x51, 0], [0x127, 0], [0x246, 2], [0x260, 0], [0x262, 0], [0x270, 0], [0x31a, 0], [0x133, 2], [0x323, 2], [0x220, 2], [0x299, 2]]
  GAS_PRESSED_THRESHOLD = 10
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: [0x262, 0x31a, 0x51], 2: [0x246, 0x133, 0x323, 0x220, 0x299]}
  FWD_BLACKLISTED_ADDRS = {2: [0x262, 0x31a, 0x51], 0: [0x246, 0x133, 0x323, 0x220, 0x299, 0x51]}

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

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety(
      "EngineData", self.VOLVO_MAIN_BUS, {"ACC_State_ECM": 3 if enable else 2},
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

  def test_fsm3_longitudinal_limits(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._fsm3_accel_msg(2.0, acc_check=1)))
    self.assertFalse(self._tx(self._fsm3_accel_msg(2.04, acc_check=1)))
    self.assertTrue(self._tx(self._fsm3_accel_msg(-4.0)))
    self.assertFalse(self._tx(self._fsm3_accel_msg(-4.04)))

  def test_stock_fsm_forwarding_is_replaced_only_during_control(self):
    for addr in (0x260, 0x270):
      self.safety.set_controls_allowed(False)
      self.assertEqual(self.VOLVO_MAIN_BUS, self.safety.safety_fwd_hook(self.VOLVO_CAM_BUS, addr))
      self.safety.set_controls_allowed(True)
      self.assertEqual(-1, self.safety.safety_fwd_hook(self.VOLVO_CAM_BUS, addr))

    self._rx(self._user_gas_msg(True))
    for addr in (0x260, 0x270):
      self.assertEqual(self.VOLVO_MAIN_BUS, self.safety.safety_fwd_hook(self.VOLVO_CAM_BUS, addr))

  def test_ecm_state_requires_both_bits(self):
    for state in (0, 1, 2, 3, 1, 2):
      self._rx(self.packer.make_can_msg_safety("EngineData", 0, {"ACC_State_ECM": state}))
      self.assertEqual(self.safety.get_controls_allowed(), state == 3)

  def test_fsm0_cannot_enable_acc_without_ecm(self):
    msg = self.packer.make_can_msg_safety("FSM0", 0, {"Byte_2": 4})
    self.assertFalse(self._tx(msg))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(msg))

  def test_inactive_acceleration_is_neutral_only(self):
    self.safety.set_controls_allowed(False)
    for accel in (-4., -0.04, 0., 0.04, 2.):
      self.assertEqual(self._tx(self._fsm3_accel_msg(accel)), accel == 0.)

  def test_relays_follow_upstream_forwarding_in_all_engagement_states(self):
    for enabled in (False, True):
      self.safety.set_controls_allowed(enabled)
      for addr in (0x31a, 0x51):
        self.assertEqual(self.safety.safety_fwd_hook(2, addr), -1)
      for addr in (0x133, 0x323, 0x220, 0x299):
        self.assertEqual(self.safety.safety_fwd_hook(0, addr), -1)


if __name__ == "__main__":
  unittest.main()
