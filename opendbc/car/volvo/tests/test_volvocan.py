from opendbc.can import CANPacker
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.carcontroller import clip_longitudinal_accel


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
