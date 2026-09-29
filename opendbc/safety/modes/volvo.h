#pragma once

#include "opendbc/safety/declarations.h"

#define VOLVO_EUCD_ACC_PEDAL      0x020U
#define VOLVO_EUCD_FSM0           0x051U
#define VOLVO_EUCD_CC_BUTTONS     0x127U
#define VOLVO_EUCD_VEHICLE_SPEED  0x148U
#define VOLVO_EUCD_BRAKE_INFO     0x20AU
#define VOLVO_EUCD_PSCM1          0x246U
#define VOLVO_EUCD_FSM1           0x260U
#define VOLVO_EUCD_FSM2           0x262U
#define VOLVO_EUCD_FSM3           0x270U
#define VOLVO_EUCD_ESR_SIM        0x5C0U
#define VOLVO_EUCD_ESR_SIM_AUTH   0x5C1U

#define VOLVO_MAIN_BUS 0U
#define VOLVO_CAM_BUS  2U
#define VOLVO_RADAR_BUS 1U

static const AngleSteeringLimits VOLVO_STEERING_LIMITS = {
  .max_angle = 9000,
  .angle_deg_to_can = 100,
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {5., .8, .15},
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {5., 3.5, .4},
  },
  .frequency = 50U,
};

static const LongitudinalLimits VOLVO_LONG_LIMITS = {
  .max_accel = 50,    // +2.00 m/s^2, raw value relative to the DBC offset
  .min_accel = -100,  // -4.00 m/s^2
  .inactive_accel = 0,
};

// The 0x5C0 contract is intentionally duplicated here rather than relying on
// host-side validation.  A Panda running this safety model must reject a
// malformed or stale simulation lifecycle even if the controller is faulty.
static uint32_t volvo_sim_auth_first_ts = 0U;
static uint32_t volvo_sim_auth_last_ts = 0U;
static uint32_t volvo_sim_fsm_first_ts = 0U;
static uint32_t volvo_sim_fsm_last_ts = 0U;
static uint32_t volvo_sim_last_ts = 0U;
static int volvo_sim_last_range = 0;
static int volvo_sim_last_rate = 0;
static bool volvo_sim_new_seen = false;
static bool volvo_sim_pre_set_mode = false;
static bool volvo_sim_stock_available = false;
static bool volvo_sim_set_pressed = false;
static bool volvo_sim_set_seen = false;
static uint32_t volvo_sim_stock_last_ts = 0U;
static uint32_t volvo_sim_speed_last_ts = 0U;

static bool volvo_sim_exact_zero(const CANPacket_t *msg) {
  for (unsigned int i = 0U; i < 8U; i++) {
    if (msg->data[i] != 0U) {
      return false;
    }
  }
  return true;
}

static void volvo_sim_reset(void) {
  volvo_sim_auth_first_ts = 0U;
  volvo_sim_auth_last_ts = 0U;
  volvo_sim_fsm_first_ts = 0U;
  volvo_sim_fsm_last_ts = 0U;
  volvo_sim_last_ts = 0U;
  volvo_sim_last_range = 0;
  volvo_sim_last_rate = 0;
  volvo_sim_new_seen = false;
  volvo_sim_pre_set_mode = false;
}

static void volvo_sim_update_lease(uint32_t *first_ts, uint32_t *last_ts, uint32_t now) {
  if ((*last_ts == 0U) || safety_get_ts_elapsed(now, *last_ts) > 120000U) {
    *first_ts = now;
  }
  *last_ts = now;
}

static bool volvo_sim_speed_allowed(void) {
  return vehicle_speed.min >= 4000 && vehicle_speed.max <= 18000;
}

static bool volvo_sim_pre_set_speed_allowed(void) {
  return vehicle_speed.min >= 800 && vehicle_speed.max < 8300;
}

static bool volvo_sim_tx_checks(const CANPacket_t *msg, uint32_t now) {
  if (msg->bus != VOLVO_RADAR_BUS || GET_LEN(msg) != 8U) {
    return false;
  }
  if (volvo_sim_exact_zero(msg)) {
    volvo_sim_reset();
    return true;
  }

  const unsigned int target_id = (msg->data[0] >> 5U) & 0x03U;
  const unsigned int status = (msg->data[0] >> 3U) & 0x03U;
  const unsigned int function = msg->data[0] & 0x07U;
  const int range = msg->data[4];
  const int range_accel = (int8_t)msg->data[5];
  const int range_rate = (int8_t)msg->data[6];
  const int speed_min = vehicle_speed.min;
  const float speed = (float)speed_min / VEHICLE_SPEED_FACTOR;

  const bool pre_set = volvo_sim_pre_set_mode && !controls_allowed && !cruise_engaged_prev &&
                       volvo_sim_stock_available && !volvo_sim_set_pressed && !volvo_sim_set_seen &&
                       safety_get_ts_elapsed(now, volvo_sim_stock_last_ts) <= 120000U &&
                       safety_get_ts_elapsed(now, volvo_sim_speed_last_ts) <= 120000U &&
                       volvo_sim_pre_set_speed_allowed() && range == 40 && range_rate == 0 && range_accel == 0;
  const bool active_braking = !volvo_sim_pre_set_mode && controls_allowed && volvo_sim_speed_allowed() &&
                              volvo_sim_fsm_first_ts != 0U &&
                              safety_get_ts_elapsed(now, volvo_sim_fsm_first_ts) >= 200000U &&
                              safety_get_ts_elapsed(now, volvo_sim_fsm_last_ts) <= 120000U;

  if (gas_pressed || brake_pressed || (!pre_set && !active_braking) ||
      volvo_sim_auth_first_ts == 0U ||
      safety_get_ts_elapsed(now, volvo_sim_auth_first_ts) < 200000U ||
      safety_get_ts_elapsed(now, volvo_sim_auth_last_ts) > 120000U ||
      target_id != 1U || function != 0U ||
      msg->data[1] != 0U || msg->data[2] != 0U || msg->data[3] != 0U || msg->data[7] != 0U ||
      range < 1 || range > 255 || range_rate < -128 || range_accel < -16 || range_accel > 16 ||
      speed + ((float)range_rate * 0.25F) < -0.5F) {
    return false;
  }

  if ((status == 1U) && volvo_sim_new_seen) {
    return false;
  }
  if (status < 1U || status > 3U) {
    return false;
  }
  if ((status == 2U || status == 3U) && !volvo_sim_new_seen) {
    return false;
  }
  if (status == 1U && range < (int)((speed * speed) / 8.0F + 3.0F)) {
    return false;
  }

  if (volvo_sim_last_ts != 0U) {
    const uint32_t dt_us = safety_get_ts_elapsed(now, volvo_sim_last_ts);
    if (dt_us > 120000U || dt_us == 0U) {
      return false;
    }
    const float dt = (float)dt_us * 1e-6F;
    const float max_range_delta = (float)(SAFETY_ABS(volvo_sim_last_rate) + 8) * 0.25F * dt + 2.0F;
    const float max_rate_delta = 4.0F * dt + 1.0F;
    if (SAFETY_ABS(range - volvo_sim_last_range) > (int)(max_range_delta + 1.0F) ||
        SAFETY_ABS(range_rate - volvo_sim_last_rate) > (int)(max_rate_delta + 1.0F) * 4) {
      return false;
    }
  }

  if (status == 1U) {
    volvo_sim_new_seen = true;
  }
  volvo_sim_last_ts = now;
  volvo_sim_last_range = range;
  volvo_sim_last_rate = range_rate;
  return true;
}

static void volvo_rx_hook(const CANPacket_t *msg) {
  if ((msg->bus == VOLVO_MAIN_BUS) && (msg->addr == VOLVO_EUCD_VEHICLE_SPEED)) {
    unsigned int speed_raw = (GET_BYTES(msg, 6, 1) << 8) | GET_BYTES(msg, 7, 1);
    vehicle_moving = speed_raw >= 36U;
    UPDATE_VEHICLE_SPEED(speed_raw * 0.01 * KPH_TO_MS);
    volvo_sim_speed_last_ts = microsecond_timer_get();
    if ((vehicle_speed.min >= 8300) && !cruise_engaged_prev) {
      volvo_sim_set_seen = false;
    }
  }

  if ((msg->bus == VOLVO_MAIN_BUS) && (msg->addr == VOLVO_EUCD_ACC_PEDAL)) {
    unsigned int gas_raw = ((GET_BYTES(msg, 2, 1) & 0x03U) << 8) | GET_BYTES(msg, 3, 1);
    gas_pressed = gas_raw >= 100U;
  }

  if ((msg->bus == VOLVO_MAIN_BUS) && (msg->addr == VOLVO_EUCD_BRAKE_INFO)) {
    brake_pressed = ((GET_BYTES(msg, 2, 1) & 0x0CU) >> 2U) == 2U;
  }

  if ((msg->bus == VOLVO_MAIN_BUS) && (msg->addr == VOLVO_EUCD_PSCM1)) {
    int raw_angle = (GET_BYTES(msg, 2, 1) << 8) | GET_BYTES(msg, 3, 1);
    int angle_meas_new = ((raw_angle * 447) / 100) - 146500;
    // FSM2's requested angle has 0.04 degree resolution. Normalize the
    // higher-resolution PSCM measurement to the same scale for inactive checks.
    int angle_sign = angle_meas_new < 0 ? -1 : 1;
    angle_meas_new = angle_sign * ((((angle_meas_new * angle_sign) + 1) / 4) * 4);
    update_sample(&angle_meas, angle_meas_new);
  }

  if ((msg->bus == VOLVO_CAM_BUS) && (msg->addr == VOLVO_EUCD_FSM0)) {
    bool cruise_engaged = (GET_BYTES(msg, 2, 1) & 0x04U) != 0U;
    if (cruise_engaged) {
      volvo_sim_set_seen = true;
    }
    volvo_sim_stock_available = (GET_BYTES(msg, 2, 1) & 0x02U) != 0U;
    volvo_sim_stock_last_ts = microsecond_timer_get();
    pcm_cruise_check(cruise_engaged);
  }

  if ((msg->bus == VOLVO_MAIN_BUS) && (msg->addr == VOLVO_EUCD_CC_BUTTONS)) {
    volvo_sim_set_pressed = GET_BIT(msg, 63U);
    if (volvo_sim_set_pressed) {
      volvo_sim_set_seen = true;
    }
  }
}

static bool volvo_tx_hook(const CANPacket_t *msg) {
  bool violation = false;

  if (msg->addr == VOLVO_EUCD_CC_BUTTONS) {
    bool cancel = GET_BIT(msg, 59U) || !GET_BIT(msg, 43U);
    bool resume = GET_BIT(msg, 61U) || !GET_BIT(msg, 45U);
    violation |= cancel && !cruise_engaged_prev;
    violation |= resume && !controls_allowed;
  }

  if (msg->addr == VOLVO_EUCD_FSM2) {
    int raw_angle = ((GET_BYTES(msg, 3, 1) & 0x3FU) << 8) | GET_BYTES(msg, 4, 1);
    int desired_angle = (raw_angle * 4) - 32768;
    bool lka_active = (GET_BYTES(msg, 5, 1) & 0x03U) != 0U;
    // EUCD requires LKASteerDirection=NONE for eight frames when changing
    // direction, while LKAAngleReq continues tracking the target. Keep the
    // angle-rate state active through that handoff; treating NONE as fully
    // inactive rejects the retained target and causes a rejection cascade
    // when the direction becomes active again.
    bool angle_tracking_active = controls_allowed || lka_active;
    violation |= steer_angle_cmd_checks(desired_angle, angle_tracking_active, VOLVO_STEERING_LIMITS);
  }

  if ((msg->addr == VOLVO_EUCD_FSM0) || (msg->addr == VOLVO_EUCD_FSM1)) {
    violation |= !controls_allowed;
  }

  if (msg->addr == VOLVO_EUCD_FSM3) {
    int raw_accel = (int)GET_BYTES(msg, 1, 1) - 126;
    violation |= !controls_allowed || longitudinal_accel_checks(raw_accel, VOLVO_LONG_LIMITS);
    if (!violation && raw_accel <= -18) {
      volvo_sim_update_lease(&volvo_sim_fsm_first_ts, &volvo_sim_fsm_last_ts, microsecond_timer_get());
    }
  }

  if (msg->addr == VOLVO_EUCD_ESR_SIM_AUTH) {
    const bool valid_magic = msg->bus == VOLVO_MAIN_BUS && GET_LEN(msg) == 8U &&
                             msg->data[0] == 'V' && msg->data[1] == 'L' &&
                             (msg->data[2] == 'S' || msg->data[2] == 'P') && msg->data[3] == '1' &&
                             msg->data[4] == 0U && msg->data[5] == 0U &&
                             msg->data[6] == 0U && msg->data[7] == 0U;
    if (valid_magic) {
      const bool pre_set = msg->data[2] == 'P';
      if (volvo_sim_pre_set_mode != pre_set) {
        volvo_sim_reset();
        volvo_sim_pre_set_mode = pre_set;
      }
      volvo_sim_update_lease(&volvo_sim_auth_first_ts, &volvo_sim_auth_last_ts, microsecond_timer_get());
    }
    // This is a host/Panda lease, never a vehicle CAN message.
    return false;
  }

  if (msg->addr == VOLVO_EUCD_ESR_SIM) {
    violation |= !volvo_sim_tx_checks(msg, microsecond_timer_get());
  }

  return !violation;
}

static bool volvo_fwd_hook(int bus_num, int addr) {
  if ((bus_num == (int)VOLVO_CAM_BUS) && controls_allowed && !gas_pressed) {
    return (addr == VOLVO_EUCD_FSM0) || (addr == VOLVO_EUCD_FSM1) || (addr == VOLVO_EUCD_FSM3);
  }
  return false;
}

static safety_config volvo_init(uint16_t param) {
  (void)param;
  volvo_sim_reset();
  volvo_sim_stock_available = false;
  volvo_sim_set_pressed = false;
  volvo_sim_set_seen = false;
  volvo_sim_stock_last_ts = 0U;
  volvo_sim_speed_last_ts = 0U;

  static const CanMsg VOLVO_TX_MSGS[] = {
    {VOLVO_EUCD_FSM0,       VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_CC_BUTTONS, VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_PSCM1,      VOLVO_CAM_BUS,  8, .check_relay = true},
    {VOLVO_EUCD_FSM1,       VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_FSM2,       VOLVO_MAIN_BUS, 8, .check_relay = true},
    {VOLVO_EUCD_FSM3,       VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_ESR_SIM,    VOLVO_RADAR_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_ESR_SIM_AUTH, VOLVO_MAIN_BUS, 8, .check_relay = false},
  };

  static RxCheck volvo_rx_checks[] = {
    {.msg = {{VOLVO_EUCD_ACC_PEDAL, VOLVO_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_FSM0, VOLVO_CAM_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_VEHICLE_SPEED, VOLVO_MAIN_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_BRAKE_INFO, VOLVO_MAIN_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_PSCM1, VOLVO_MAIN_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(volvo_rx_checks, VOLVO_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
  .fwd = volvo_fwd_hook,
};
