#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.volvo.volvocan import calculate_lka_checksum
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestVolvoSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[0x51, 0], [0x127, 0], [0x246, 2], [0x260, 0], [0x262, 0], [0x270, 0]]
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

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety(
      "FSM0", self.VOLVO_CAM_BUS, {"ACC_Enabled": int(enable)},
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
