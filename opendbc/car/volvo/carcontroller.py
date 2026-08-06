from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, SteerDirection


def clip_longitudinal_accel(accel: float) -> float:
  return max(CarControllerParams.ACCEL_MIN, min(CarControllerParams.ACCEL_MAX, float(accel)))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CP = CP
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.apply_steer_prev = 0.0
    self.apply_steer_dir_prev = SteerDirection.NONE
    self.lat_active_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_before_block = SteerDirection.NONE

    self.last_resume_frame = 0
    self.takeoff_start_frame = -1_000_000
    self.resume_distance = 0
    self.waiting_for_resume = False
    self.resume_count = 0
    self.resume_ack_frames = 0

    self.next_long_tx_nanos = 0
    self.next_fsm0_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000
    self.FSM0_TX_PERIOD_NANOS = 10_000_000

  @staticmethod
  def _next_tx_time(previous: int, now_nanos: int, period: int) -> int:
    next_tx = previous + period
    if previous == 0 or next_tx <= now_nanos:
      next_tx = now_nanos + period
    return next_tx

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    if CC.cruiseControl.cancel and CS.out.vEgo > self.CP.minSteerSpeed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    # The Volvo FSM expects steering commands and the spoofed PSCM state at 50 Hz.
    if self.frame % 2 == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        apply_steer = apply_std_steer_angle_limits(
          actuators.steeringAngleDeg,
          self.apply_steer_prev,
          CS.out.vEgoRaw,
          CS.out.steeringAngleDeg,
          CC.latActive,
          CarControllerParams.ANGLE_LIMITS,
        )
        apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

        error = CS.out.steeringAngleDeg - apply_steer
        error_with_deadzone = 0 if abs(error) < CarControllerParams.DEADZONE else error

        if not self.lat_active_prev:
          self.apply_steer_dir_prev = apply_steer_dir

        if self.steer_blocked:
          if (apply_steer_dir == self.steer_dir_before_block or
              self.steer_blocked_cnt <= 0 or error_with_deadzone == 0):
            self.steer_blocked = False
        elif apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
          self.steer_blocked = True
          self.steer_blocked_cnt = CarControllerParams.BLOCK_LEN
          self.steer_dir_before_block = self.apply_steer_dir_prev

        if self.steer_blocked:
          self.steer_blocked_cnt -= 1
          apply_steer_dir = SteerDirection.NONE
        elif error_with_deadzone == 0:
          apply_steer_dir = self.apply_steer_dir_prev
      else:
        apply_steer = CS.out.steeringAngleDeg
        apply_steer_dir = SteerDirection.NONE

      can_sends.append(volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)))
      can_sends.append(volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values))

      self.apply_steer_prev = apply_steer
      self.apply_steer_dir_prev = apply_steer_dir
      self.lat_active_prev = CC.latActive

    # Stock ACC requires a resume button plus matching FSM3 acknowledgements.
    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.05)
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.0:
      if at_standstill and not self.waiting_for_resume:
        self.resume_distance = CS.acc_distance
        self.waiting_for_resume = True
        self.resume_count = 0

      lead_moved = (self.resume_distance < 45 and CS.acc_distance < 45 and
                    CS.acc_distance - self.resume_distance >= 2)
      resume_requested = bool(CC.cruiseControl.resume) and not CC.longActive
      if at_standstill and self.waiting_for_resume and (lead_moved or resume_requested):
        stock_accel = clip_longitudinal_accel(CS.stock_FSM3["ACC_AccelerationRequest"])
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * 25)
        can_sends.extend([volvocan.create_acc_state_msg(self.packer_pt, CS.stock_FSM3, stock_accel)] * 25)
        if self.resume_count == 0:
          self.takeoff_start_frame = self.frame
          self.resume_ack_frames = 25
        self.resume_distance = CS.acc_distance
        self.resume_count += 1

      if self.waiting_for_resume and (self.resume_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting_for_resume = False
        self.last_resume_frame = self.frame

    # Replace the three stock FSM control frames only while stock openpilot is
    # actively controlling longitudinal. Ordinary disengaged traffic stays stock.
    long_tx_due = (self.CP.openpilotLongitudinalControl and CC.longActive and
                   now_nanos >= self.next_long_tx_nanos)
    if long_tx_due:
      self.next_long_tx_nanos = self._next_tx_time(
        self.next_long_tx_nanos, now_nanos, self.LONG_TX_PERIOD_NANOS,
      )

      op_accel = clip_longitudinal_accel(actuators.accel)
      stock_accel = clip_longitudinal_accel(CS.stock_FSM3["ACC_AccelerationRequest"])
      takeoff_elapsed = (self.frame - self.takeoff_start_frame) * DT_CTRL
      in_takeoff_window = takeoff_elapsed < 15.0 and CS.out.vEgo < 5.0
      accel = stock_accel if in_takeoff_window and stock_accel > max(op_accel, 0.0) else op_accel
      if CS.out.cruiseState.enabled and CS.out.vEgo < 0.05 and not in_takeoff_window and accel > -0.5:
        accel = -1.0

      acc_check = int(self.resume_ack_frames > 0)
      self.resume_ack_frames = max(0, self.resume_ack_frames - 1)
      can_sends.append(volvocan.create_longitudinal(
        self.packer_pt, CS.stock_FSM3, clip_longitudinal_accel(accel), acc_check,
      ))
      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1))

    if CC.longActive and now_nanos >= self.next_fsm0_tx_nanos:
      self.next_fsm0_tx_nanos = self._next_tx_time(
        self.next_fsm0_tx_nanos, now_nanos, self.FSM0_TX_PERIOD_NANOS,
      )
      can_sends.append(volvocan.create_fsm0(self.packer_pt, CS.stock_FSM0))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends
