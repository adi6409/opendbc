from opendbc.car.can_definitions import CanData
from opendbc.car.volvo.values import CANBUS


ESR_SIM_TARGET_ID = 1
ESR_SIM_FUNCTION_ACC = 0
ESR_SIM_STATUS_INVALID = 0
ESR_SIM_STATUS_NEW = 1
ESR_SIM_STATUS_UPDATED = 2
ESR_SIM_STATUS_COASTED = 3
ESR_SIM_ADDR = 0x5C0
ESR_SIM_AUTH_ADDR = 0x5C1
ESR_SIM_AUTH_PAYLOAD = b"VLS1\x00\x00\x00\x00"
ESR_PRE_SET_AUTH_PAYLOAD = b"VLP1\x00\x00\x00\x00"


def create_button_msg(packer, resume=False, cancel=False, bus=0):
  # TODO: validate
  msg = {
    "ACCOnOffBtn": cancel,
    "ACCOnOffBtnInv": not cancel,
    "ACCResumeBtn": resume,
    "ACCResumeBtnInv": not resume,
  }
  return packer.make_can_msg("CCButtons", bus, msg)


def create_acc_state_msg(packer, stock_fsm3, accel):
  # The original Volvo SNG helper emitted a zero-filled FSM3 with only
  # ACC_Check set. With longitudinal safety enabled that encodes -5.04 m/s²,
  # so Panda rejects every acknowledgment. Preserve the complete current
  # stock FSM3 shape and change only the bounded accel plus ACC_Check.
  return create_longitudinal(packer, stock_fsm3, accel, 1)


def create_lkas_state_msg(packer, steering_angle: float, stock_values: dict):
  # Manipulate data from servo to FSM
  # Set LKATorque and LKAActive to zero otherwise LKA will be disabled. (Check dbc)
  msg = {
    "LKATorque": 0,
    "SteeringAngleServo": steering_angle,
    "byte0": stock_values["byte0"],
    "byte4": stock_values["byte4"],
    "byte7": stock_values["byte7"],
    "LKAActive": int(stock_values["LKAActive"]) & 0xF5,
    "EPSTorque": stock_values["EPSTorque"],
  }
  return packer.make_can_msg("PSCM1", 2, msg)


def calculate_lka_checksum(dat: bytearray) -> int:
  # Input: dat byte array, and fingerprint
  # Steering direction = 0 -> 3
  # TrqLim = 0 -> 255
  # Steering angle request = -360 -> 360

  # Extract LKAAngleRequest, LKADirection and Unknown
  steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
  steering_direction_request = dat[5] & 0x03
  trqlim = dat[2]

  # Sum of all bytes, carry ignored.
  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  # Checksum is inverted sum of all bytes
  return s ^ 0xFF


def create_lka_msg(packer, apply_steer: float, steer_direction: int):
  values = {
    "LKAAngleReq": apply_steer,
    "LKASteerDirection": steer_direction,
    "TrqLim": 0,

    # car specific parameters
    "SET_X_22": 0x25, # Test these values: 0x24, 0x22
    "SET_X_02": 0,    # Test 0x00, 0x02
    "SET_X_10": 0x10, # Test 0x10, 0x1c, 0x18, 0x00
    "SET_X_A4": 0xa7, # Test 0xa4, 0xa6, 0xa5, 0xe5, 0xe7
  }

  # calculate checksum
  dat = packer.make_can_msg("FSM2", 0, values)[1]
  values["Checksum"] = calculate_lka_checksum(dat)

  return packer.make_can_msg("FSM2", 0, values)


def create_longitudinal(packer, stock_fsm3, accel, acc_check):
  # Pass through ALL stock FSM3 bits verbatim so OP's message is byte-identical
  # to stock's latest except for ACC_AccelerationRequest (byte 1) and ACC_Check.
  # This preserves the car's 5-frame validation pattern and all counter/mode
  # bits the ECM checks. Missing any field here will flip a bit in the output
  # and the ECM may fault after accumulated errors (observed in drive 27 at ~30s).
  values = {s: stock_fsm3[s] for s in (
    "ACC_Standstill",
    "Byte_01",
    "Byte_02",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
  }
  return packer.make_can_msg("FSM3", 0, values)


def create_fsm0(packer, stock_fsm0):
  # Preserve the complete stock FSM0 payload and its validation pattern.
  values = {s: stock_fsm0[s] for s in (
    "Byte_0",
    "Byte_1",
    "ACC_FrontCar",
    "ACC_Available",
    "ACC_Enabled",
    "Byte_2_lo",
    "Byte_3_lo",
    "ACC_BrakeAlert",
    "Byte_3_hi",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  return packer.make_can_msg("FSM0", 0, values)


def create_radar(packer, stock_fsm1):
  # Preserve the complete stock FSM1 payload and its validation pattern.
  values = {s: stock_fsm1[s] for s in (
    "ACC_Distance",
    "Byte_1",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  return packer.make_can_msg("FSM1", 0, values)


def create_esr_simulation(packer, status, range_m, range_rate, range_accel,
                          angle_deg=0.0, lateral_position=0.0, lateral_rate=0.0):
  """Encode Delphi ESR simulation input 0x5C0 on the physical radar bus.

  This is deliberately upstream of the ESR output/fusion boundary.  It must
  not be replaced with a fabricated TargetN, FSM0, FSM1, or FSM4 frame.
  """
  def signed_byte(value: float, scale: float) -> int:
    return int(round(float(value) / scale)) & 0xFF

  dat = bytes([
    (ESR_SIM_TARGET_ID << 5) | (int(status) << 3) | ESR_SIM_FUNCTION_ACC,
    signed_byte(angle_deg, 0.5),
    signed_byte(lateral_position, 0.25),
    signed_byte(lateral_rate, 0.25),
    max(0, min(255, int(round(float(range_m))))),
    signed_byte(range_accel, 0.25),
    signed_byte(range_rate, 0.25),
    0,
  ])
  return CanData(ESR_SIM_ADDR, dat, CANBUS.body)


def create_esr_simulation_retirement():
  """Return the exact-zero lifecycle reset accepted in every safety state."""
  return CanData(ESR_SIM_ADDR, b"\x00" * 8, CANBUS.body)


def create_esr_simulation_authorization():
  """Private host-to-Panda lifecycle lease; safety consumes it on bus 0."""
  return CanData(ESR_SIM_AUTH_ADDR, ESR_SIM_AUTH_PAYLOAD, CANBUS.pt)


def create_esr_pre_set_authorization():
  """Lease for the limited pre-SET same-speed lead mode."""
  return CanData(ESR_SIM_AUTH_ADDR, ESR_PRE_SET_AUTH_PAYLOAD, CANBUS.pt)
