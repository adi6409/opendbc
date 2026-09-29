import copy
from opendbc.can import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import CarControllerParams, DBC, CANBUS
from opendbc.car.volvo.radar_interface import DELPHI_ESR_TRACK_NAMES, INVALID_STATUSES
from opendbc.car.volvo.virtual_target import RadarSeed
from opendbc.car import Bus, structs


class CarState(CarStateBase):
  # These are the stationary path selectors documented by the ESR output
  # stream.  They are more useful than choosing the numerically closest raw
  # slot: the route captures show many occupied off-axis tracks at the same
  # time as the selected ACC path.
  ESR_STATIONARY_SELECTOR_SIGNALS = (
    "CAN_TX_PATH_ID_ACC_STAT",
    "CAN_TX_PATH_ID_FCW_STAT",
    "CAN_TX_PATH_ID_CMBB_STAT",
  )
  RADAR_CENTER_ANGLE_DEG = 5.0
  RADAR_STATIONARY_RATE_TOLERANCE_MS = 2.0

  def __init__(self, CP):
    super().__init__(CP)
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0
    self.frame = 0
    self.virtual_target_new_nanos = 0
    self.virtual_target_target_range = 0.0
    self.physical_brake_since_nanos = 0
    self.physical_brake_response = False
    self.esr_sweep_end_nanos = 0
    self.esr_scan_start_nanos = 0
    self.radar_seed: RadarSeed | None = None
    self.native_adoption_evidence = False
    self.virtual_target_preexisting_signature = False
    self.virtual_target_seen_signature_clear = False
    self.stock_set_pressed = False
    self.panda_returned_simulation_frames = 0
    self._last_simulation_return_nanos = 0

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]
    body_cp = can_parsers[Bus.body]

    ret = structs.CarState()

    # car speed
    ret.vEgoRaw = pt_cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # Volvo P3 cluster pads displayed speed by ~7-9% above true CAN speed
    # (regulatory: speedometer must never under-read). User reported
    # comma showing 28 when cluster set to 30, 32 when cluster set to 35
    # (ratios 1.071 and 1.094). vEgoCluster lets the UI show the cluster-
    # matched value so the user's set point matches what they see.
    VOLVO_CLUSTER_SCALE = 1.08
    ret.vEgoCluster = ret.vEgoRaw * VOLVO_CLUSTER_SCALE
    ret.standstill = ret.vEgoRaw < 0.1

    # gas pedal
    ret.gasPressed = pt_cp.vl["AccPedal"]["AccPedal"] >= 10

    # brake pedal
    ret.brakePressed = pt_cp.vl["Brake_Info"]["BrakePedal"] == 2

    # steering
    ret.steeringAngleDeg = pt_cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringRateDeg = pt_cp.vl["SAS0"]["SteeringRateOfChange"]
    self.steeringDirection = pt_cp.vl["SAS0"]["SteeringDirection"]
    ret.steeringTorque = pt_cp.vl["PSCM1"]["EPSTorque"]
    if self.steeringDirection:
      ret.steeringTorque = -abs(ret.steeringTorque)
    ret.steeringTorqueEps = pt_cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = False

    # cruise state
    ret.cruiseState.speed = pt_cp.vl["ACC_Speed"]["ACC_Speed"] * CV.KPH_TO_MS
    # Same cluster-scale applied to ACC setpoint so what the UI shows
    # matches what the user dialed on the car cluster.
    ret.cruiseState.speedCluster = ret.cruiseState.speed * VOLVO_CLUSTER_SCALE
    ret.cruiseState.available = bool(cam_cp.vl["FSM0"]["ACC_Available"])
    ret.cruiseState.enabled = bool(cam_cp.vl["FSM0"]["ACC_Enabled"])
    self.stock_set_pressed = bool(pt_cp.vl["CCButtons"]["ACCSetBtn"])
    # ACC_Standstill bit = 1 when Volvo's ACC is holding the car at 0 km/h
    # with brake applied (standstill hold). OP's SNG block reads this to know
    # when to blast Resume button so stock ACC properly releases and follows
    # the lead resuming. Without this, stock ACC sees OP commanding accel
    # from standstill without a Resume press and hard-cancels (observed in
    # drive 38).
    ret.cruiseState.standstill = bool(cam_cp.vl["FSM3"]["ACC_Standstill"])
    ret.cruiseState.nonAdaptive = False
    ret.accFaulted = False
    self.acc_distance = cam_cp.vl["FSM1"]["ACC_Distance"]

    # Physical brake response is intentionally independent of the requested
    # acceleration. BrakeCmd alone is only a command/feedback bit; require
    # pressure or sustained measured deceleration before calling it response.
    parser_nanos = max(pt_cp._last_update_nanos, cam_cp._last_update_nanos)
    brake_response_sample = bool(pt_cp.vl["Brake_Info"]["BrakeCmd"]) and (
      pt_cp.vl["Brake_Info"]["BrakePressure"] > 5 or ret.aEgo <= -0.3
    )
    if brake_response_sample:
      if self.physical_brake_since_nanos == 0:
        self.physical_brake_since_nanos = parser_nanos
    else:
      self.physical_brake_since_nanos = 0
    self.physical_brake_response = (
      self.physical_brake_since_nanos != 0 and
      parser_nanos - self.physical_brake_since_nanos >= 100_000_000
    )

    # The ESR sweep has a reliable start/end phase.  The end edge is the only
    # host-side opportunity to transmit 0x5C0; a stale parser batch must not
    # trigger an injection after the next scan has begun.
    self.esr_sweep_end_nanos = int(body_cp.ts_nanos["CIPV_Targets_Etc"]["CAN_RX_CIPV_TARGETS_BYTE_0"])
    self.esr_scan_start_nanos = int(body_cp.ts_nanos["ESR_Status"]["CAN_TX_SCAN_INDEX"])
    simulation_return_nanos = int(body_cp.ts_nanos["ESR_Sim1_5C0"]["SimStatus"])
    if simulation_return_nanos > self._last_simulation_return_nanos:
      self.panda_returned_simulation_frames += 1
      self._last_simulation_return_nanos = simulation_return_nanos
    self.radar_seed = self._select_radar_seed(body_cp, ret.vEgoRaw)

    # Check if servo stops responding when ACC is active
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      if not self.cruiseState_enabled_prev:
        self.eps_torque_timer = 0

      if ret.steeringTorqueEps == 0:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0

      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT
    else:
      ret.steerFaultTemporary = False

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    # gear
    ret.gearShifter = structs.CarState.GearShifter.drive

    # safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # lock info
    ret.doorOpen = not all([pt_cp.vl["Doors"]["DriverDoorClosed"], pt_cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False

    # Electronic parking brake. The HandBrake message (0x2EE) reports
    # Hand_Brake_State as a small enum, decoded from drive 0000004c EPB
    # capture: state=4 is the NORMAL/RELEASED state (default while
    # driving), state=2 is mid-transition, state=1 is fully engaged.
    # Initial guess `bool(state)` was wrong and made parkingBrake=True
    # almost always — which causes openpilot's disengage logic to refuse
    # engagement (drive 0000004d/4e: user couldn't engage CC at all).
    hb_state = pt_cp.vl["HandBrake"]["Hand_Brake_State"]
    ret.parkingBrake = hb_state in (1, 2)

    # Store info from servo message PSCM1
    self.pscm_stock_values = pt_cp.vl["PSCM1"]

    # Stock messages preserved for openpilot longitudinal control.
    self.stock_FSM0 = copy.copy(cam_cp.vl["FSM0"])
    self.stock_FSM1 = copy.copy(cam_cp.vl["FSM1"])
    self.stock_FSM3 = copy.copy(cam_cp.vl["FSM3"])
    self.stock_FSM4 = copy.copy(cam_cp.vl["FSM4"])
    self.ACC_Check = cam_cp.vl["FSM3"]["ACC_Check"]

    self.native_adoption_evidence = self._native_adoption_evidence(cam_cp, parser_nanos)

    self.frame += 1
    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("VehicleSpeed1", 50),
      ("AccPedal", 100),
      ("Brake_Info", 50),
      ("PSCM1", 50),
      ("ACC_Speed", 50),
      ("CCButtons", float("nan")),
      ("MiscCarInfo", 25),
      ("Doors", 20),
      ("SAS0", 100),
      ("HandBrake", 5),
    ]

    cam_messages = [
      ("FSM0", 100),
      ("FSM1", 50),
      ("FSM3", 50),
      ("FSM4", 50),
    ]

    body_messages = [
      ("ESR_Status", 20),
      ("ESR_Sim1_5C0", float("nan")),
      ("CIPV_Targets_Etc", 20),
      ("ESR_Output_InPath", 20),
      *[(name, 20) for name in DELPHI_ESR_TRACK_NAMES],
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.radar], body_messages, CANBUS.body),
    }

  @classmethod
  def _select_radar_seed(cls, body_cp, speed: float = 0.0) -> RadarSeed | None:
    selectors = body_cp.vl.get("ESR_Output_InPath", {})
    selected_ids = {
      int(selectors[signal]) for signal in cls.ESR_STATIONARY_SELECTOR_SIGNALS
      if signal in selectors and 1 <= int(selectors[signal]) <= len(DELPHI_ESR_TRACK_NAMES)
    }
    candidates = []
    for name in DELPHI_ESR_TRACK_NAMES:
      track = body_cp.vl[name]
      status = int(track["CAN_TX_TRACK_STATUS"])
      angle = float(track["CAN_TX_TRACK_ANGLE"])
      range_m = float(track["CAN_TX_TRACK_RANGE"])
      range_rate = float(track["CAN_TX_TRACK_RANGE_RATE"])
      if status in INVALID_STATUSES or not 0.5 < range_m < 250.0:
        continue
      if abs(angle) > cls.RADAR_CENTER_ANGLE_DEG:
        continue

      # ESR range rate is relative to the car.  A stationary roadside object
      # therefore reports approximately -vEgo, not zero.  The old filter used
      # abs(range_rate) <= 1 and discarded every stationary target in the
      # saved braking routes.
      stationary = abs(range_rate + speed) <= cls.RADAR_STATIONARY_RATE_TOLERANCE_MS
      selected = int(name.removeprefix("Target")) in selected_ids
      priority = (0 if selected and stationary else
                  1 if stationary else
                  2 if selected else 3)
      candidates.append((priority, range_m, RadarSeed(
        range_m=range_m,
        range_rate=range_rate,
        range_accel=float(track["CAN_TX_TRACK_RANGE_ACCEL"]),
        angle_deg=angle,
        lateral_position=0.0,
        lateral_rate=float(track["CAN_TX_TRACK_LAT_RATE"]),
      )))
    return min(candidates, key=lambda candidate: (candidate[0], candidate[1]))[2] if candidates else None

  def _native_signature_shape(self, target_range: float) -> bool:
    fsm0 = self.stock_FSM0
    fsm1 = self.stock_FSM1
    fsm3 = self.stock_FSM3
    fsm4 = self.stock_FSM4
    # Byte 0 is the observed 0x55/0xAA heartbeat.  The route data and the
    # handoff's 0xB/0xF observations identify NEW_SIGNAL_2 as the primary
    # state.  The upper two bits of Byte 2 are the no-main-track marker in
    # the saved native streams (0xFC is the common no-lead value); require
    # them clear instead of treating Byte 1's always-set low bit as validity.
    primary_track_state = int(fsm4["NEW_SIGNAL_2"])
    main_track_valid = (int(fsm4["Byte_2"]) & 0xC0) == 0
    action = bool(int(fsm1["Byte_1"]) & 0x01)
    distance_matches = abs(float(fsm1["ACC_Distance"]) - target_range) <= max(
      3.0, target_range * 0.2,
    )
    return (
      bool(fsm0["ACC_FrontCar"]) and action and main_track_valid and
      primary_track_state in (0xB, 0xF) and distance_matches and
      float(fsm3["ACC_AccelerationRequest"]) <= -0.3
    )

  def _native_adoption_evidence(self, cam_cp, now_nanos: int) -> bool:
    if self.virtual_target_new_nanos == 0 or self.virtual_target_target_range <= 0:
      return False

    def fresh(message: str, signal: str) -> bool:
      return cam_cp.ts_nanos[message][signal] > self.virtual_target_new_nanos

    shape = self._native_signature_shape(self.virtual_target_target_range)
    if not shape:
      self.virtual_target_seen_signature_clear = True
    native_signature = (
      shape and (not self.virtual_target_preexisting_signature or self.virtual_target_seen_signature_clear) and
      fresh("FSM0", "ACC_FrontCar") and fresh("FSM1", "Byte_1") and
      fresh("FSM4", "NEW_SIGNAL_2") and fresh("FSM4", "Byte_2") and
      fresh("FSM3", "ACC_AccelerationRequest")
    )
    physical_response_after_new = (
      self.physical_brake_response and
      self.physical_brake_since_nanos > self.virtual_target_new_nanos and
      now_nanos > self.virtual_target_new_nanos
    )
    return native_signature or physical_response_after_new
