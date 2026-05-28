#include <Arduino.h>
#include <math.h>

#include "angle_control.hpp"
#include "filter.hpp"
#include "icm20948.hpp"
#include "motor.hpp"
#include "rc_control.hpp"
#include "vl53l0x.hpp"
#include "webserver.hpp"
#include "z_control.hpp"

const char* ssid = "Drone-Config";
const char* password = "password123";

ICM20948 imu;
VL53L0X lidar;
Filter filter;
AngleController angle_controller;
ZControl z_control;
WebDashboard dashboard;

#define MOTOR_FL_PIN 16
#define MOTOR_FR_PIN 17
#define MOTOR_RL_PIN 18
#define MOTOR_RR_PIN 19

Motor motors[4] = {
  Motor(MOTOR_FL_PIN, 0),
  Motor(MOTOR_FR_PIN, 1),
  Motor(MOTOR_RL_PIN, 2),
  Motor(MOTOR_RR_PIN, 3)
};

uint32_t last_micros = 0;
uint32_t last_ws_send_ms = 0;

static constexpr float THROTTLE_ARM_PCT = 5.0f;
static constexpr uint32_t WS_PERIOD_MS = 20;

float alt_set_mm = 0.0f;
float alt_kp = 0.002f;
float alt_ki = 0.0000f;
float alt_kd = 0.000f;
float alt_lpf_cutoff_hz = 6.0f;
float alt_deadband_mm = 20.0f;
float alt_int_limit = 2000.0f;
float alt_trim_limit = 20.0f;
float hover_throttle_pct = 35.0f;
float alt_set_expo = 2.0f;

TelemetrySnapshot telem;

static void syncZControl() {
  z_control.setSetpointMm(alt_set_mm);
  z_control.setHoverThrottle(hover_throttle_pct);
  z_control.setParams(alt_kp, alt_ki, alt_kd,
                      alt_lpf_cutoff_hz,
                      alt_deadband_mm,
                      alt_int_limit,
                      alt_trim_limit);
}

static void onPidGains(const PidGainsRequest& req) {
  angle_controller.updateGains(req.pp, req.pi, req.pd,
                               req.rp, req.ri, req.rd,
                               req.yp, req.yi, req.yd);
}

static void onAltitudeParams(const AltitudeParamsRequest& req) {
  alt_set_mm = req.set_mm;
  alt_kp = req.kp;
  alt_ki = req.ki;
  alt_kd = req.kd;
  alt_lpf_cutoff_hz = req.lpf_cutoff_hz;
  alt_deadband_mm = req.deadband_mm;
  alt_int_limit = req.int_limit;
  alt_trim_limit = req.trim_limit;
  hover_throttle_pct = req.hover_throttle;
  alt_set_expo = req.set_expo;

  if (alt_set_mm < ZControl::ALT_MIN_MM) alt_set_mm = ZControl::ALT_MIN_MM;
  if (alt_set_mm > ZControl::ALT_MAX_MM) alt_set_mm = ZControl::ALT_MAX_MM;

  if (alt_lpf_cutoff_hz < 0.1f) alt_lpf_cutoff_hz = 0.1f;
  if (alt_deadband_mm < 0.0f) alt_deadband_mm = 0.0f;
  if (alt_int_limit < 0.0f) alt_int_limit = 0.0f;
  if (alt_trim_limit < 0.0f) alt_trim_limit = 0.0f;

  if (alt_set_expo < 1.0f) alt_set_expo = 1.0f;
  if (alt_set_expo > 4.0f) alt_set_expo = 4.0f;

  syncZControl();
}

void setup() {
  Serial.begin(115200);

  dashboard.begin(ssid, password, onPidGains, onAltitudeParams);

  RCControl::begin();
  Serial.println("SUCCESS: RC input initialized.");

  if (!imu.begin()) {
    Serial.println("FATAL ERROR: ICM20948 IMU NOT FOUND! Check wiring (SDA/SCL).");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("SUCCESS: IMU Initialized.");

  if (!lidar.begin(VL53L0X_I2C_ADDR, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE)) {
    Serial.println("WARN: VL53L0X LiDAR not detected; continuing without distance updates.");
  } else {
    Serial.println("SUCCESS: VL53L0X Initialized and tared.");
  }

  z_control.reset();
  syncZControl();

  filter.begin(100.0f);
  Serial.println("SUCCESS: Filter Initialized.");

  for (int i = 0; i < 4; i++) {
    motors[i].begin(2000);
  }

  last_micros = micros();
  Serial.println("Setup complete.");
}

void loop() {
  const uint16_t ch_throttle = RCControl::readChannelRaw(2);

  float throttle_cmd = (static_cast<float>(ch_throttle) - 1000.0f) / 10.0f;
  if (throttle_cmd < 0.0f) throttle_cmd = 0.0f;
  if (throttle_cmd > 100.0f) throttle_cmd = 100.0f;

  if (!imu.update()) {
    dashboard.cleanupClients();
    return;
  }

  if (!filter.update9(imu.data())) {
    dashboard.cleanupClients();
    return;
  }

  lidar.update();
  const bool dist_ok = lidar.data().valid;
  float dist_mm = telem.alt_meas_mm;
  if (dist_ok) {
    dist_mm = lidar.data().height_mm;
    if (dist_mm < ZControl::ALT_MIN_MM) dist_mm = ZControl::ALT_MIN_MM;
    if (dist_mm > ZControl::ALT_MAX_MM) dist_mm = ZControl::ALT_MAX_MM;
  }

  const uint32_t current_micros = micros();
  float dt = (current_micros - last_micros) * 1e-6f;
  last_micros = current_micros;
  if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;

  const euler_t current_angle = filter.euler9();
  const euler_t target = {0.0f, 0.0f, current_angle.yaw_deg};

  if (throttle_cmd < THROTTLE_ARM_PCT) {
    z_control.reset();
    alt_set_mm = 0.0f;
    z_control.setSetpointMm(alt_set_mm);

    for (int i = 0; i < 4; i++) motors[i].stop();
    angle_controller.resetIntegrals();
    telem.throttle = 0.0f;
  } else {
    float setpoint_alpha = (throttle_cmd - THROTTLE_ARM_PCT) / (100.0f - THROTTLE_ARM_PCT);
    if (setpoint_alpha < 0.0f) setpoint_alpha = 0.0f;
    if (setpoint_alpha > 1.0f) setpoint_alpha = 1.0f;

    const float setpoint_curve = powf(setpoint_alpha, alt_set_expo);
    alt_set_mm = setpoint_curve * ZControl::ALT_MAX_MM;
    if (alt_set_mm < ZControl::ALT_MIN_MM) alt_set_mm = ZControl::ALT_MIN_MM;
    if (alt_set_mm > ZControl::ALT_MAX_MM) alt_set_mm = ZControl::ALT_MAX_MM;

    z_control.setSetpointMm(alt_set_mm);
    z_control.update(dist_mm, dist_ok, dt);

    const float throttle_stab = z_control.commandThrottle();
    angle_controller.compute(target, current_angle, throttle_stab, dt);
    for (int i = 0; i < 4; i++) {
      motors[i].setPercent(angle_controller.getMotorOutput(i));
    }
    telem.throttle = throttle_stab;
  }


  telem.t_ms = millis(); 
  telem.roll = current_angle.roll_deg;
  telem.pitch = current_angle.pitch_deg;
  telem.yaw = current_angle.yaw_deg;
  telem.throttle_in = throttle_cmd;
  if (throttle_cmd < THROTTLE_ARM_PCT) telem.throttle = 0.0f;
  telem.alt_set_mm = z_control.setpointMm();
  telem.alt_meas_mm = dist_mm;
  telem.alt_filt_mm = z_control.filteredMm();
  telem.alt_err_mm = z_control.errorMm();
  telem.alt_trim = z_control.trimPct();
  telem.hover_throttle = z_control.hoverThrottlePct();
  telem.alt_expo = alt_set_expo;
  telem.dt = dt;
  telem.m0 = angle_controller.getMotorOutput(0);
  telem.m1 = angle_controller.getMotorOutput(1);
  telem.m2 = angle_controller.getMotorOutput(2);
  telem.m3 = angle_controller.getMotorOutput(3);
  telem.alt_ok = z_control.valid() ? 1 : 0;

  const uint32_t now_ms = telem.t_ms;
  if (now_ms - last_ws_send_ms >= WS_PERIOD_MS) {
    last_ws_send_ms = now_ms;
    dashboard.publishTelemetry(telem);
    dashboard.cleanupClients();
  }

  static uint32_t last_print = 0;
  if (millis() - last_print > 100) {
    last_print = millis();
    Serial.print("Ang-> R:"); Serial.print(current_angle.roll_deg, 1);
    Serial.print(" P:"); Serial.print(current_angle.pitch_deg, 1);
    Serial.print(" Y:"); Serial.print(current_angle.yaw_deg, 1);
    Serial.print(" | AltSet(mm):"); Serial.print(telem.alt_set_mm, 1);
    Serial.print(" Alt(mm):"); Serial.print(telem.alt_meas_mm, 1);
    Serial.print(" Trim(%):"); Serial.print(telem.alt_trim, 2);
    Serial.print(" ["); Serial.print(telem.alt_ok ? "ok" : "stale"); Serial.print("]");
    Serial.print(" | Motors-> FL:"); Serial.print(telem.m0, 1);
    Serial.print("% FR:"); Serial.print(telem.m1, 1);
    Serial.print("% RL:"); Serial.print(telem.m2, 1);
    Serial.print("% RR:"); Serial.print(telem.m3, 1);
    Serial.println("%");
  }
}
