// #include <Arduino.h>
//
// #include "icm20948.hpp"
// #include "vl53l0x.hpp"
//
// namespace {
//
// constexpr uint32_t IMU_I2C_FREQ_HZ = 100000;
// constexpr uint32_t LOG_PERIOD_MS = 100;
// constexpr float HEIGHT_MIN_MM = 0.0f;
// constexpr float HEIGHT_MAX_MM = 2000.0f;
//
// // ICM20948 imu;
// VL53L0X lidar;
// uint32_t last_log_ms = 0;
//
// }  // namespace
//
// void setup() {
//   Serial.begin(115200);
//   delay(500);
//
//   Serial.println();
//   Serial.println("VL53L0X LiDAR + IMU test starting...");
//   // Serial.println("INFO: Initializing ICM20948 IMU...");
//
//   // if (!imu.begin(5, 4, IMU_I2C_FREQ_HZ, true)) {
//   //   Serial.println("FATAL ERROR: ICM20948 IMU not detected.");
//   //   while (true) {
//   //     delay(1000);
//   //   }
//   // }
//
//   // Serial.println("SUCCESS: ICM20948 initialized.");
//   Serial.println("INFO: Initializing I2C bus...");
//   Wire.begin(5, 4);
//   Wire.setClock(100000);
//
//   Serial.println("INFO: Initializing VL53L0X LiDAR (includes auto-tare)...");
//
//   if (!lidar.begin(VL53L0X_I2C_ADDR, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE)) {
//     Serial.println("FATAL ERROR: VL53L0X LiDAR not detected or tare failed.");
//     while (true) {
//       delay(1000);
//     }
//   }
//
//   Serial.println("SUCCESS: VL53L0X initialized and tared.");
//   Serial.println("Logging LiDAR height and IMU data...");
// }
//
// void loop() {
//   // const bool imu_ok = imu.update();
//   const bool lidar_ok = lidar.update();
//
//   if (millis() - last_log_ms < LOG_PERIOD_MS) {
//     return;
//   }
//
//   last_log_ms = millis();
//
//   // if (!imu_ok) {
//   //   Serial.println("WARN: IMU reading failed.");
//   // }
//   if (!lidar_ok) {
//     Serial.println("WARN: LiDAR reading failed.");
//   }
//
//   // Serial.print("IMU ");
//   // if (imu_ok) {
//   //   const imu9_t& imu_data = imu.data();
//   //   Serial.print("accel_g[");
//   //   Serial.print(imu_data.ax, 3);
//   //   Serial.print(", ");
//   //   Serial.print(imu_data.ay, 3);
//   //   Serial.print(", ");
//   //   Serial.print(imu_data.az, 3);
//   //   Serial.print("] gyro_dps[");
//   //   Serial.print(imu_data.gx, 1);
//   //   Serial.print(", ");
//   //   Serial.print(imu_data.gy, 1);
//   //   Serial.print(", ");
//   //   Serial.print(imu_data.gz, 1);
//   //   Serial.print("] mag_uT[");
//   //   Serial.print(imu_data.mx, 1);
//   //   Serial.print(", ");
//   //   Serial.print(imu_data.my, 1);
//   //   Serial.print(", ");
//   //   Serial.print(imu_data.mz, 1);
//   //   Serial.print("]");
//   // } else {
//   //   Serial.print("read_failed");
//   // }
//
//   Serial.print(" | LiDAR ");
//   if (lidar_ok && lidar.data().valid) {
//     float height_mm = lidar.data().height_mm;
//     if (height_mm > HEIGHT_MAX_MM) height_mm = HEIGHT_MAX_MM;
//     Serial.print("height_mm=");
//     Serial.print(height_mm, 1);
//     Serial.print(" raw_mm=");
//     Serial.print(lidar.data().raw_distance_mm, 1);
//   } else {
//     Serial.print("read_failed");
//   }
//
//   Serial.println();
// }
