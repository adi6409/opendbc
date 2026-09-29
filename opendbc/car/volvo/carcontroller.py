from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.virtual_target import PreEngageTarget, VirtualBrakeTarget
from opendbc.car.volvo.values import CarControllerParams, SteerDirection

try:
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
except ImportError:  # Standalone opendbc tests do not include the root runtime.
  Params = None
  cloudlog = None


def clip_longitudinal_accel(accel: float) -> float:
  return max(CarControllerParams.ACCEL_MIN, min(CarControllerParams.ACCEL_MAX, float(accel)))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CP = CP
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.packer_radar = CANPacker(dbc_names[Bus.body])
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
    self.virtual_target = VirtualBrakeTarget()
    self.pre_engage_target = PreEngageTarget()
    self.pre_set_authorizations = 0
    self.pre_set_frames = 0
    self.pre_set_retirements = 0
    self.startup_retirements_remaining = 3
    self._params = None
    self._last_simulation_log = None
    if Params is not None:
      try:
        self._params = Params()
      except Exception:
        # A controller unit test or a non-device opendbc process has no
        # parameter store. The diagnostic trigger is therefore unavailable,
        # which is the safe default.
        self._params = None

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

    # Clear a lifecycle left behind by a host/Panda restart. Exact-zero is
    # always safety-allowed and never represents a fabricated radar target.
    if self.startup_retirements_remaining > 0:
      can_sends.append(volvocan.create_esr_simulation_retirement())
      self.virtual_target.stats.simulation_frames += 1
      self.virtual_target.stats.retirement_frames += 1
      self.startup_retirements_remaining -= 1

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

    # Stock ACC requires a resume button plus matching FSM3 acknowledgments.
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

      accel, in_takeoff_window = self._effective_longitudinal_accel(CS, actuators.accel)

      acc_check = int(self.resume_ack_frames > 0)
      self.resume_ack_frames = max(0, self.resume_ack_frames - 1)
      can_sends.append(volvocan.create_longitudinal(
        self.packer_pt, CS.stock_FSM3, clip_longitudinal_accel(accel), acc_check,
      ))
      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1))

    # The simulation input is the only virtual radar message. It is scheduled
    # from the fresh ESR 0x5E8 end-of-sweep edge, never from control-loop
    # modulo timing or a userspace replay of native FSM4.
    effective_accel, _ = self._effective_longitudinal_accel(CS, actuators.accel)
    target_output = self.virtual_target.update(
      now_nanos,
      diagnostic_trigger=self._consume_simulation_trigger(),
      automatic_braking=True,
      controls_active=bool(self.CP.openpilotLongitudinalControl and CC.longActive),
      stock_acc_enabled=bool(CS.out.cruiseState.enabled),
      gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      speed=float(CS.out.vEgo),
      accel_request=effective_accel,
      phase_end_nanos=CS.esr_sweep_end_nanos,
      scan_start_nanos=CS.esr_scan_start_nanos,
      native_adopted=bool(CS.native_adoption_evidence),
      physical_response=bool(CS.physical_brake_response),
      native_distance=float(CS.acc_distance),
      radar_seed=CS.radar_seed,
    )
    pre_set_output = self.pre_engage_target.update(
      now_nanos,
      controls_active=bool(CC.longActive),
      stock_acc_available=bool(CS.out.cruiseState.available),
      stock_acc_enabled=bool(CS.out.cruiseState.enabled),
      set_pressed=bool(CS.stock_set_pressed),
      gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      speed=float(CS.out.vEgo),
      native_lead=bool(CS.stock_FSM0["ACC_FrontCar"]),
      phase_end_nanos=CS.esr_sweep_end_nanos,
      scan_start_nanos=CS.esr_scan_start_nanos,
    )
    if pre_set_output.authorization:
      can_sends.append(volvocan.create_esr_pre_set_authorization())
      self.pre_set_authorizations += 1
    if pre_set_output.frame is not None:
      pre_set_frame = pre_set_output.frame
      self.pre_set_frames += 1
      if pre_set_frame.status == volvocan.ESR_SIM_STATUS_INVALID:
        self.pre_set_retirements += 1
      can_sends.append(volvocan.create_esr_simulation(
        self.packer_radar, pre_set_frame.status, pre_set_frame.range_m,
        pre_set_frame.range_rate, pre_set_frame.range_accel,
      ))
    self.virtual_target.stats.panda_returned_frames = int(
      getattr(CS, "panda_returned_simulation_frames", 0),
    )
    self._log_simulation_stats()
    if target_output.authorization:
      can_sends.append(volvocan.create_esr_simulation_authorization())
    if target_output.frame is not None:
      target = target_output.frame
      can_sends.append(volvocan.create_esr_simulation(
        self.packer_radar,
        target.status,
        target.range_m,
        target.range_rate,
        target.range_accel,
        target.angle_deg,
        target.lateral_position,
        target.lateral_rate,
      ))
      if target.status == volvocan.ESR_SIM_STATUS_NEW:
        CS.virtual_target_preexisting_signature = CS._native_signature_shape(target.range_m)
        CS.virtual_target_seen_signature_clear = False
        CS.virtual_target_new_nanos = now_nanos
        CS.virtual_target_target_range = target.range_m
    if target_output.cancel:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    if CC.longActive and now_nanos >= self.next_fsm0_tx_nanos:
      self.next_fsm0_tx_nanos = self._next_tx_time(
        self.next_fsm0_tx_nanos, now_nanos, self.FSM0_TX_PERIOD_NANOS,
      )
      can_sends.append(volvocan.create_fsm0(self.packer_pt, CS.stock_FSM0))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev

    self.frame += 1
    return new_actuators, can_sends

  def _consume_simulation_trigger(self) -> bool:
    """Consume the operator's one-shot development parameter, if available."""
    if self._params is None:
      return False
    try:
      if not self._params.get_bool("VolvoRadarSimulationTrigger"):
        return False
      self._params.put_bool("VolvoRadarSimulationTrigger", False, block=True)
      return True
    except Exception:
      # Unknown keys on an older device image and unavailable parameter
      # storage must both fail closed without affecting normal Volvo control.
      return False

  def _log_simulation_stats(self):
    if cloudlog is None or self.frame % 100 != 0:
      return
    stats = self.virtual_target.stats
    snapshot = (
      self.virtual_target.state.name,
      stats.trigger_accepted, stats.trigger_rejected, stats.authorization_frames,
      stats.simulation_frames, stats.retirement_frames, stats.cancel_requests,
      stats.panda_returned_frames, stats.native_adoptions, stats.physical_responses,
      self.pre_set_authorizations, self.pre_set_frames, self.pre_set_retirements,
    )
    if snapshot != self._last_simulation_log:
      self._last_simulation_log = snapshot
      cloudlog.info("volvo_esr_sim stats=%s", snapshot)

  def _effective_longitudinal_accel(self, CS, requested_accel: float) -> tuple[float, bool]:
    op_accel = clip_longitudinal_accel(requested_accel)
    stock_accel = clip_longitudinal_accel(CS.stock_FSM3["ACC_AccelerationRequest"])
    takeoff_elapsed = (self.frame - self.takeoff_start_frame) * DT_CTRL
    in_takeoff_window = takeoff_elapsed < 15.0 and CS.out.vEgo < 5.0
    accel = stock_accel if in_takeoff_window and stock_accel > max(op_accel, 0.0) else op_accel
    if CS.out.cruiseState.enabled and CS.out.vEgo < 0.05 and not in_takeoff_window and accel > -0.5:
      accel = -1.0
    return clip_longitudinal_accel(accel), in_takeoff_window
