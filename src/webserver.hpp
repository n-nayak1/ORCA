#pragma once

#include <Arduino.h>

struct TelemetrySnapshot {
  uint32_t t_ms = 0;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float m0 = 0.0f;
  float m1 = 0.0f;
  float m2 = 0.0f;
  float m3 = 0.0f;
  float throttle = 0.0f;
  float throttle_in = 0.0f;
  float alt_set_mm = 0.0f;
  float alt_meas_mm = 0.0f;
  float alt_filt_mm = 0.0f;
  float alt_err_mm = 0.0f;
  float alt_trim = 0.0f;
  float hover_throttle = 0.0f;
  float alt_expo = 0.0f;
  float dt = 0.0f;
  uint8_t alt_ok = 0;
};

struct PidGainsRequest {
  float pp = 0.06f;
  float pi = 0.0f;
  float pd = 0.065f;
  float rp = 0.07f;
  float ri = 0.0f;
  float rd = 0.065f;
  float yp = 0.005f;
  float yi = 0.0f;
  float yd = 0.0f;
};

struct AltitudeParamsRequest {
  float set_mm = 0.0f;
  float kp = 0.020f;
  float ki = 0.0005f;
  float kd = 0.010f;
  float lpf_cutoff_hz = 6.0f;
  float deadband_mm = 20.0f;
  float int_limit = 2000.0f;
  float trim_limit = 20.0f;
  float hover_throttle = 35.0f;
  float set_expo = 2.0f;
};

using PidGainsHandler = void (*)(const PidGainsRequest& req);
using AltitudeParamsHandler = void (*)(const AltitudeParamsRequest& req);

class WebDashboard {
public:
  void begin(const char* ssid,
             const char* password,
             PidGainsHandler pid_handler,
             AltitudeParamsHandler alt_handler);

  void publishTelemetry(const TelemetrySnapshot& telem);
  void cleanupClients();

private:
  AltitudeParamsRequest alt_params_{};
};
