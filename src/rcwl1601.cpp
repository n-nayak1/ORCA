#include "rcwl1601.hpp"

RCWL1601::RCWL1601(uint8_t addr) : addr_(addr) {}

bool RCWL1601::begin(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq_hz, bool begin_i2c) {
  if (begin_i2c) {
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(freq_hz);
  }

  uint32_t distance_um = 0;
  history_count_ = 0;
  history_head_ = 0;
  if (!read_distance_um_filtered(distance_um, 5)) return false;

  data_.distance_um = distance_um;
  data_.distance_mm = distance_um / 1000.0f;
  data_.t_us = micros();
  return true;
}

bool RCWL1601::start_ranging() {
  Wire.beginTransmission(addr_);
  Wire.write(START_RANGING);
  return (Wire.endTransmission() == 0);
}

bool RCWL1601::read_distance_um(uint32_t& distance_um) {
  const size_t len = 3;
  size_t n = Wire.requestFrom((int)addr_, (int)len, (int)true);
  if (n != len) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }

  uint8_t b0 = (uint8_t)Wire.read();
  uint8_t b1 = (uint8_t)Wire.read();
  uint8_t b2 = (uint8_t)Wire.read();

  distance_um = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;
  return true;
}

bool RCWL1601::update(uint8_t ranging_delay_ms) {
  uint32_t distance_um = 0;
  if (!read_distance_um_filtered(distance_um, ranging_delay_ms)) return false;

  data_.distance_um = distance_um;
  data_.distance_mm = distance_um / 1000.0f;
  data_.t_us = micros();
  return true;
}

bool RCWL1601::read_distance_um_filtered(uint32_t& distance_um_filtered, uint8_t ranging_delay_ms) {
  uint32_t distance_um_raw = 0;
  if (!start_ranging()) return false;
  delay(ranging_delay_ms);
  if (!read_distance_um(distance_um_raw)) return false;

  history_[history_head_] = distance_um_raw;
  history_head_ = (history_head_ + 1) % MEDIAN_WINDOW;
  if (history_count_ < MEDIAN_WINDOW) history_count_++;

  distance_um_filtered = compute_median();
  return true;
}

uint32_t RCWL1601::compute_median() const {
  if (history_count_ == 0) return 0;

  uint32_t sorted[MEDIAN_WINDOW]{};
  for (size_t i = 0; i < history_count_; i++) {
    sorted[i] = history_[i];
  }

  for (size_t i = 1; i < history_count_; i++) {
    uint32_t key = sorted[i];
    size_t j = i;
    while (j > 0 && sorted[j - 1] > key) {
      sorted[j] = sorted[j - 1];
      j--;
    }
    sorted[j] = key;
  }

  return sorted[history_count_ / 2];
}
