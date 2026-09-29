"""Bounded Delphi ESR simulation-input lifecycle for the Volvo EUCD port.

The controller uses this module to decide *when* an ESR simulation input may
be sent.  It never creates a fused FSM message or an ESR output-slot frame.
The corresponding Panda policy is deliberately stricter than this host-side
state machine; a host bug must fail closed at the vehicle boundary.
"""

from dataclasses import dataclass
from enum import Enum, auto


class TargetState(Enum):
  IDLE = auto()
  AUTHORIZING = auto()
  ACTIVE = auto()
  RELEASING = auto()
  FAILED = auto()


@dataclass(frozen=True)
class RadarSeed:
  range_m: float
  range_rate: float
  range_accel: float = 0.0
  angle_deg: float = 0.0
  lateral_position: float = 0.0
  lateral_rate: float = 0.0


@dataclass(frozen=True)
class TargetFrame:
  status: int
  range_m: float
  range_rate: float
  range_accel: float
  angle_deg: float = 0.0
  lateral_position: float = 0.0
  lateral_rate: float = 0.0


@dataclass(frozen=True)
class VirtualTargetOutput:
  frame: TargetFrame | None = None
  authorization: bool = False
  cancel: bool = False


@dataclass
class VirtualTargetStats:
  """Counters for post-drive verification of one simulation episode."""
  trigger_accepted: int = 0
  trigger_rejected: int = 0
  authorization_frames: int = 0
  simulation_frames: int = 0
  panda_returned_frames: int = 0
  new_frames: int = 0
  updated_frames: int = 0
  coasted_frames: int = 0
  retirement_frames: int = 0
  cancel_requests: int = 0
  native_adoptions: int = 0
  physical_responses: int = 0


class VirtualBrakeTarget:
  # Handoff-derived gates. Keep these values mirrored in volvo.h and tests.
  MIN_SPEED = 4.5
  MAX_SPEED = 17.5
  BRAKE_INTENT_ACCEL = -0.72
  INTENT_DEBOUNCE_NS = 400_000_000
  AUTH_LEAD_NS = 250_000_000
  AUTH_PERIOD_NS = 100_000_000
  PHASE_MAX_AGE_NS = 15_000_000
  PHASE_GAP_NS = 120_000_000
  ADOPTION_DEADLINE_NS = 400_000_000
  RESPONSE_AFTER_ADOPTION_NS = 700_000_000
  ATTEMPT_CAP_NS = 1_000_000_000
  BRAKE_RESPONSE_DEBOUNCE_NS = 100_000_000
  RELEASE_FRAMES = 3
  ZERO_RETIREMENT_FRAMES = 8
  CANCEL_FRAMES = 5

  def __init__(self):
    self.state = TargetState.IDLE
    self.intent_since_nanos = 0
    self.attempt_since_nanos = 0
    self.new_nanos = 0
    self.adopted_nanos = 0
    self.auth_last_nanos = 0
    self.last_phase_nanos = 0
    self.last_frame_nanos = 0
    self.target_speed = 0.0
    self.range_m = 0.0
    self.range_rate = 0.0
    self.range_accel = 0.0
    self.attempted_episode = False
    self.release_count = 0
    self.zero_count = 0
    self.cancel_count = 0
    self.request_pending = False
    self.stats = VirtualTargetStats()
    self._physical_response_seen = False

  @property
  def active(self) -> bool:
    return self.state in (TargetState.AUTHORIZING, TargetState.ACTIVE)

  @staticmethod
  def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))

  def _clear_episode(self):
    self.state = TargetState.IDLE
    self.intent_since_nanos = 0
    self.attempt_since_nanos = 0
    self.new_nanos = 0
    self.adopted_nanos = 0
    self.auth_last_nanos = 0
    self.last_phase_nanos = 0
    self.last_frame_nanos = 0
    self.release_count = 0
    self.zero_count = 0
    # A safety cancel is scoped to the failed ACC episode.  Do not let a
    # partially consumed pulse leak into a later engagement after retirement.
    self.cancel_count = 0
    self.request_pending = False
    self._physical_response_seen = False

  def _begin_release(self, cancel: bool):
    if self.state not in (TargetState.IDLE, TargetState.RELEASING):
      self.state = TargetState.RELEASING
      self.release_count = 0
      self.zero_count = 0
    if cancel:
      if self.cancel_count < self.CANCEL_FRAMES:
        self.stats.cancel_requests += 1
      self.cancel_count = max(self.cancel_count, self.CANCEL_FRAMES)

  def _start_target(self, speed: float, native_distance: float, seed: RadarSeed | None):
    if seed is not None and 0.5 < seed.range_m < 250.0:
      range_m = seed.range_m
      range_rate = seed.range_rate
      range_accel = seed.range_accel
      angle_deg = seed.angle_deg
      lateral_position = seed.lateral_position
      lateral_rate = seed.lateral_rate
    elif 0.5 < native_distance < 250.0:
      range_m = native_distance
      range_rate = -min(max(speed * 0.25, 1.0), 8.0)
      range_accel = -1.5
      angle_deg = lateral_position = lateral_rate = 0.0
    else:
      range_m = max(speed * 2.5, 12.0)
      range_rate = -min(max(speed * 0.25, 1.0), 8.0)
      range_accel = -1.5
      angle_deg = lateral_position = lateral_rate = 0.0

    # The first target must be beyond a conservative stopping-distance floor;
    # a sudden stationary wall was one of the failed Sunnypilot strategies.
    stopping_floor = (speed * speed) / (2.0 * 4.0) + 3.0
    self.range_m = max(range_m, stopping_floor)
    self.range_rate = self._clamp(range_rate, -min(speed, 8.0), 0.0)
    self.target_speed = max(0.0, speed + self.range_rate)
    self.range_accel = self._clamp(range_accel, -4.0, 4.0)
    self._seed_angle_deg = self._clamp(angle_deg, -2.0, 2.0)
    self._seed_lateral_position = self._clamp(lateral_position, -1.0, 1.0)
    self._seed_lateral_rate = self._clamp(lateral_rate, -1.0, 1.0)

  def _next_frame(self, now_nanos: int, status: int, speed: float) -> TargetFrame:
    if self.last_frame_nanos:
      dt = self._clamp((now_nanos - self.last_frame_nanos) * 1e-9, 0.0, 0.12)
      self.target_speed = max(0.0, self.target_speed + self.range_accel * dt)
      self.range_rate = self._clamp(self.target_speed - speed, -32.0, 31.75)
      self.range_m = self._clamp(self.range_m + self.range_rate * dt, 1.0, 255.0)

    self.last_frame_nanos = now_nanos
    return TargetFrame(
      status=status,
      range_m=self._clamp(self.range_m, 1.0, 255.0),
      range_rate=self._clamp(self.range_rate, -32.0, 31.75),
      range_accel=self._clamp(self.range_accel, -32.0, 31.75),
      angle_deg=self._seed_angle_deg,
      lateral_position=self._seed_lateral_position,
      lateral_rate=self._seed_lateral_rate,
    )

  def _phase_is_fresh(self, now_nanos: int, phase_end_nanos: int, scan_start_nanos: int) -> bool:
    return (
      phase_end_nanos > self.last_phase_nanos and
      phase_end_nanos <= now_nanos and
      now_nanos - phase_end_nanos <= self.PHASE_MAX_AGE_NS and
      phase_end_nanos > scan_start_nanos
    )

  def update(self, now_nanos: int, *, diagnostic_trigger: bool = False,
             automatic_braking: bool = False,
             controls_active: bool, stock_acc_enabled: bool,
             gas_pressed: bool, brake_pressed: bool, speed: float,
             accel_request: float, phase_end_nanos: int, scan_start_nanos: int,
             native_adopted: bool, physical_response: bool,
             native_distance: float = 0.0, radar_seed: RadarSeed | None = None) -> VirtualTargetOutput:
    """Advance the lifecycle and return at most one phase-aligned target frame."""
    gates = (
      controls_active and stock_acc_enabled and not gas_pressed and not brake_pressed and
      self.MIN_SPEED <= speed <= self.MAX_SPEED
    )

    # An explicit trigger remains available for a controlled experiment.
    # Automatic use requires the same sustained on-wire braking intent.
    if diagnostic_trigger:
      if gates and accel_request <= self.BRAKE_INTENT_ACCEL:
        self.request_pending = True
        self.stats.trigger_accepted += 1
      else:
        self.stats.trigger_rejected += 1

    eligible = gates and accel_request <= self.BRAKE_INTENT_ACCEL
    intent = eligible and (automatic_braking or self.request_pending or self.active)

    if not intent:
      # A continuous brake-intent episode gets one attempt.  Clearing intent
      # explicitly arms the next episode, including after a failed release.
      if self.state == TargetState.IDLE:
        self.attempted_episode = False
        self.request_pending = False
      if self.state == TargetState.RELEASING:
        pass
      elif self.state != TargetState.IDLE:
        self._begin_release(cancel=False)
      self.intent_since_nanos = 0
    elif self.intent_since_nanos == 0:
      self.intent_since_nanos = now_nanos

    if self.state == TargetState.IDLE:
      if not intent:
        return VirtualTargetOutput(cancel=self._take_cancel())
      if self.attempted_episode or now_nanos - self.intent_since_nanos < self.INTENT_DEBOUNCE_NS:
        return VirtualTargetOutput(cancel=self._take_cancel())
      self.attempted_episode = True
      self.request_pending = False
      self.state = TargetState.AUTHORIZING
      self.attempt_since_nanos = now_nanos
      self.auth_last_nanos = 0
      self._start_target(speed, native_distance, radar_seed)

    if self.state == TargetState.AUTHORIZING:
      if now_nanos - self.attempt_since_nanos > self.ATTEMPT_CAP_NS:
        self._begin_release(cancel=True)
      elif now_nanos - self.attempt_since_nanos >= self.AUTH_LEAD_NS:
        self.state = TargetState.ACTIVE

    output = VirtualTargetOutput(
      authorization=self.active and (self.auth_last_nanos == 0 or now_nanos - self.auth_last_nanos >= self.AUTH_PERIOD_NS),
      cancel=self._take_cancel(),
    )
    if output.authorization:
      self.auth_last_nanos = now_nanos
      self.stats.authorization_frames += 1

    fresh_phase = self._phase_is_fresh(now_nanos, phase_end_nanos, scan_start_nanos)
    if self.active:
      if now_nanos - self.attempt_since_nanos > self.ATTEMPT_CAP_NS:
        self._begin_release(cancel=True)
      elif self.last_phase_nanos and now_nanos - self.last_phase_nanos > self.PHASE_GAP_NS:
        self._begin_release(cancel=True)
      elif self.new_nanos and not self.adopted_nanos and native_adopted:
        self.adopted_nanos = now_nanos
        self.stats.native_adoptions += 1
      elif self.new_nanos and not self.adopted_nanos and now_nanos - self.new_nanos > self.ADOPTION_DEADLINE_NS:
        self._begin_release(cancel=True)
      elif self.adopted_nanos and not physical_response and now_nanos - self.adopted_nanos > self.RESPONSE_AFTER_ADOPTION_NS:
        self._begin_release(cancel=True)

    if physical_response and not self._physical_response_seen:
      self.stats.physical_responses += 1
      self._physical_response_seen = True

    if self.state == TargetState.ACTIVE and fresh_phase:
      self.last_phase_nanos = phase_end_nanos
      if self.new_nanos == 0:
        self.new_nanos = now_nanos
        self.stats.simulation_frames += 1
        self.stats.new_frames += 1
        return VirtualTargetOutput(
          frame=self._next_frame(now_nanos, 1, speed),
          authorization=output.authorization,
          cancel=output.cancel,
        )
      self.stats.simulation_frames += 1
      self.stats.updated_frames += 1
      return VirtualTargetOutput(
        frame=self._next_frame(now_nanos, 2, speed),
        authorization=output.authorization,
        cancel=output.cancel,
      )

    if self.state == TargetState.RELEASING:
      if fresh_phase and self.release_count < self.RELEASE_FRAMES:
        self.last_phase_nanos = phase_end_nanos
        self.release_count += 1
        self.stats.simulation_frames += 1
        self.stats.coasted_frames += 1
        return VirtualTargetOutput(
          frame=self._next_frame(now_nanos, 3, speed),
          authorization=False,
          cancel=output.cancel,
        )
      if self.zero_count < self.ZERO_RETIREMENT_FRAMES:
        self.zero_count += 1
        self.stats.simulation_frames += 1
        self.stats.retirement_frames += 1
        if self.zero_count == self.ZERO_RETIREMENT_FRAMES:
          self._clear_episode()
        return VirtualTargetOutput(
          frame=TargetFrame(0, 0.0, 0.0, 0.0),
          authorization=False,
          cancel=output.cancel,
        )

    # Keep the lease alive while waiting for an ESR sweep. Dropping it here
    # made the first NEW frame precede authorization, so Panda rejected every
    # simulation frame and the no-adoption fail-safe cancelled ACC.
    return output

  def _take_cancel(self) -> bool:
    if self.cancel_count <= 0:
      return False
    self.cancel_count -= 1
    return True


class PreEngageTarget:
  """A short, same-speed ESR lead shown only before the driver presses SET."""
  MIN_SPEED = 1.0
  MAX_SPEED = 8.2  # below the vehicle's 30 km/h no-lead SET threshold
  RANGE_M = 40.0
  MAX_DURATION_NS = 15_000_000_000
  AUTH_LEAD_NS = 250_000_000
  AUTH_PERIOD_NS = 100_000_000
  PHASE_MAX_AGE_NS = 15_000_000
  PHASE_GAP_NS = 120_000_000
  RETIREMENT_FRAMES = 3

  def __init__(self):
    self.start_nanos = 0
    self.auth_last_nanos = 0
    self.last_phase_nanos = 0
    self.new_sent = False
    self.retirements_remaining = 0
    self.exhausted = False

  def update(self, now_nanos: int, *, controls_active: bool,
             stock_acc_available: bool, stock_acc_enabled: bool,
             set_pressed: bool, gas_pressed: bool, brake_pressed: bool,
             speed: float, native_lead: bool, phase_end_nanos: int,
             scan_start_nanos: int) -> VirtualTargetOutput:
    eligible = (not controls_active and stock_acc_available and not stock_acc_enabled and not set_pressed and
                not gas_pressed and not brake_pressed and not native_lead and
                self.MIN_SPEED <= speed < self.MAX_SPEED)
    if not eligible:
      if self.start_nanos:
        self.retirements_remaining = self.RETIREMENT_FRAMES
        self.exhausted = True
      self.start_nanos = 0
      self.new_sent = False
      # A pedal or native lead before the episode starts must not consume the
      # only low-speed attempt. SET and stock ACC do consume it in Panda.
      if set_pressed or stock_acc_enabled:
        self.exhausted = True
      if speed >= self.MAX_SPEED and not stock_acc_enabled:
        self.exhausted = False
    elif self.start_nanos == 0 and not self.exhausted and self.retirements_remaining == 0:
      self.start_nanos = now_nanos
      self.auth_last_nanos = 0
      self.last_phase_nanos = 0

    if self.retirements_remaining:
      self.retirements_remaining -= 1
      return VirtualTargetOutput(frame=TargetFrame(0, 0.0, 0.0, 0.0))
    if self.start_nanos == 0:
      return VirtualTargetOutput()
    if now_nanos - self.start_nanos > self.MAX_DURATION_NS or (
      self.last_phase_nanos and now_nanos - self.last_phase_nanos > self.PHASE_GAP_NS
    ):
      self.exhausted = True
      self.start_nanos = 0
      if self.new_sent:
        self.retirements_remaining = self.RETIREMENT_FRAMES - 1
        self.new_sent = False
        return VirtualTargetOutput(frame=TargetFrame(0, 0.0, 0.0, 0.0))
      return VirtualTargetOutput()

    authorization = self.auth_last_nanos == 0 or now_nanos - self.auth_last_nanos >= self.AUTH_PERIOD_NS
    if authorization:
      self.auth_last_nanos = now_nanos
    if now_nanos - self.start_nanos < self.AUTH_LEAD_NS:
      return VirtualTargetOutput(authorization=authorization)
    fresh_phase = (phase_end_nanos > self.last_phase_nanos and phase_end_nanos <= now_nanos and
                   now_nanos - phase_end_nanos <= self.PHASE_MAX_AGE_NS and
                   phase_end_nanos > scan_start_nanos)
    if not fresh_phase:
      return VirtualTargetOutput(authorization=authorization)
    self.last_phase_nanos = phase_end_nanos
    status = 2 if self.new_sent else 1
    self.new_sent = True
    # Zero relative speed and acceleration avoid presenting an imminent wall.
    return VirtualTargetOutput(
      frame=TargetFrame(status, self.RANGE_M, 0.0, 0.0),
      authorization=authorization,
    )
