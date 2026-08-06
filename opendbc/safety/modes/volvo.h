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

#define VOLVO_MAIN_BUS 0U
#define VOLVO_CAM_BUS  2U

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

static void volvo_rx_hook(const CANPacket_t *msg) {
  if ((msg->bus == VOLVO_MAIN_BUS) && (msg->addr == VOLVO_EUCD_VEHICLE_SPEED)) {
    unsigned int speed_raw = (GET_BYTES(msg, 6, 1) << 8) | GET_BYTES(msg, 7, 1);
    vehicle_moving = speed_raw >= 36U;
    UPDATE_VEHICLE_SPEED(speed_raw * 0.01 * KPH_TO_MS);
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
    pcm_cruise_check(cruise_engaged);
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

  static const CanMsg VOLVO_TX_MSGS[] = {
    {VOLVO_EUCD_FSM0,       VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_CC_BUTTONS, VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_PSCM1,      VOLVO_CAM_BUS,  8, .check_relay = true},
    {VOLVO_EUCD_FSM1,       VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_FSM2,       VOLVO_MAIN_BUS, 8, .check_relay = true},
    {VOLVO_EUCD_FSM3,       VOLVO_MAIN_BUS, 8, .check_relay = false},
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
