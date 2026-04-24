#pragma once

#include <Arduino.h>
#include <Wire.h>

struct rcwl1601_t {
  uint32_t distance_um;
  float distance_mm;
  uint32_t t_us;
};

class RCWL1601 {
public:
  explicit RCWL1601(uint8_t addr = I2C_ADDR);

  bool begin(uint8_t sda_pin = 21, uint8_t scl_pin = 22, uint32_t freq_hz = 100000, bool begin_i2c = true);
  bool update(uint8_t ranging_delay_ms = 5);
  bool read_distance_um_filtered(uint32_t& distance_um_filtered, uint8_t ranging_delay_ms = 5);

  const rcwl1601_t& data() const { return data_; }

private:
  static constexpr uint8_t I2C_ADDR = 0x57;
  static constexpr uint8_t START_RANGING = 0x01;
  static constexpr size_t MEDIAN_WINDOW = 5;

  uint8_t addr_;
  rcwl1601_t data_{};
  uint32_t history_[MEDIAN_WINDOW]{};
  size_t history_count_ = 0;
  size_t history_head_ = 0;

  bool start_ranging();
  bool read_distance_um(uint32_t& distance_um);
  uint32_t compute_median() const;
};
