

// main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>              // REQUIRED on ESP32 for ESPAsyncWebServer
#include <ESPAsyncWebServer.h>
#include <math.h>

#include "icm20948.hpp"
#include "filter.hpp"
#include "control.hpp"
#include "motor.hpp"
#include "rc_control.hpp"

// ===================== Wi-Fi AP Credentials =====================
const char* ssid     = "Drone-Config";
const char* password = "password123";

// ===================== Objects =====================
ICM20948 imu;
Filter filter;
Controller controller;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===================== Motor Pins =====================
#define MOTOR_FL_PIN 16
#define MOTOR_FR_PIN 17
#define MOTOR_RL_PIN 18
#define MOTOR_RR_PIN 19

Motor motors[4] = {
  Motor(MOTOR_FL_PIN, 0), // FL
  Motor(MOTOR_FR_PIN, 1), // FR
  Motor(MOTOR_RL_PIN, 2), // RL
  Motor(MOTOR_RR_PIN, 3)  // RR
};

// ===================== Global Control State =====================
float manual_throttle = 0.0f;
uint32_t last_micros = 0;

// Vertical damping (accel-hold style, body-Z specific force in g)
float z_accel_set_g = 1.0f;
float z_kp = 3.0f;
float z_ki = 0.25f;
float z_kd = 0.35f;
float z_lpf_cutoff_hz = 6.0f;
float z_err_deadband_g = 0.02f;
float z_int_limit = 2.0f;
float z_trim_limit = 6.0f;

float z_accel_filt_g = 1.0f;
float z_integral = 0.0f;
float z_prev_error = 0.0f;

// ===================== Telemetry Snapshot =====================
struct Telemetry {
  uint32_t t_ms = 0;
  float roll = 0, pitch = 0, yaw = 0;
  float m0 = 0, m1 = 0, m2 = 0, m3 = 0;   // 0..100 (%)
  float throttle = 0;                     // 0..100 (%)
  float throttle_in = 0;                  // RC command (%), before z trim
  float az = 0;                           // measured body-z accel (g)
  float az_f = 0;                         // filtered body-z accel (g)
  float z_err = 0;                        // z accel error (g)
  float z_trim = 0;                       // throttle trim from z loop (%)
  float dt = 0;                           // seconds
};

Telemetry telem;
uint32_t last_ws_send_ms = 0;
static constexpr uint32_t WS_PERIOD_MS = 20; // 50 Hz

static String telemetryJson(const Telemetry& s) {
  // Minimal JSON, no ArduinoJson dependency.
  String out;
  out.reserve(200);
  out += "{";
  out += "\"t\":";  out += s.t_ms;
  out += ",\"r\":"; out += String(s.roll, 2);
  out += ",\"p\":"; out += String(s.pitch, 2);
  out += ",\"y\":"; out += String(s.yaw, 2);
  out += ",\"th\":"; out += String(s.throttle, 1);
  out += ",\"th_in\":"; out += String(s.throttle_in, 1);
  out += ",\"az\":"; out += String(s.az, 3);
  out += ",\"azf\":"; out += String(s.az_f, 3);
  out += ",\"ze\":"; out += String(s.z_err, 3);
  out += ",\"zt\":"; out += String(s.z_trim, 2);
  out += ",\"dt\":"; out += String(s.dt, 4);
  out += ",\"m0\":"; out += String(s.m0, 1);
  out += ",\"m1\":"; out += String(s.m1, 1);
  out += ",\"m2\":"; out += String(s.m2, 1);
  out += ",\"m3\":"; out += String(s.m3, 1);
  out += "}";
  return out;
}

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected\n", client->id());
  }
}

// ===================== HTML Dashboard =====================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Drone Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: #eee; }
    .card { background: #2d2d2d; padding: 15px; margin: 10px auto; max-width: 520px; border-radius: 10px; }
    .slider { width: 90%; height: 20px; margin: 15px 0; }
    input[type=number] { width: 70px; background: #444; color: #fff; border: 1px solid #666; padding: 3px; }
    button { padding: 10px 20px; margin: 5px; border-radius: 5px; border: none; cursor: pointer; }
    .kill { background: #ff4444; color: white; font-weight: bold; width: 100%; }
    .save { background: #44bb44; color: white; }
    .row { display: flex; justify-content: center; gap: 20px; flex-wrap: wrap; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
    hr { border: 0; height: 1px; background: #444; margin: 12px 0; }
  </style>
</head><body>
  <h2>Hover Control</h2>


  <div class="card">
    <h3>Live Telemetry</h3>
    <div class="row mono">
      <div>Roll: <span id="roll">--</span> deg</div>
      <div>Pitch: <span id="pitch">--</span> deg</div>
      <div>Yaw: <span id="yaw">--</span> deg</div>
    </div>
    <hr>
    <div class="row mono">
      <div>FL: <span id="m0">--</span>%</div>
      <div>FR: <span id="m1">--</span>%</div>
      <div>RL: <span id="m2">--</span>%</div>
      <div>RR: <span id="m3">--</span>%</div>
    </div>
    <div class="mono" style="margin-top:10px;">dt: <span id="dt">--</span> s</div>
  </div>

  <div class="card">
    <h3>CSV Logging</h3>
    <button class="save" onclick="startLog()">Start Logging</button>
    <button onclick="stopLog()">Stop Logging</button>
    <button onclick="downloadCSV()">Download CSV</button>
    <div style="margin-top:10px;">Rows: <span id="rows">0</span></div>
  </div>

  <div class="card">
    <h3>PID Tuning</h3>
    <div>Pitch: P <input id="pp" type="number" step="0.001" value="0.06">
         I <input id="pi" type="number" step="0.001" value="0.00">
         D <input id="pd" type="number" step="0.001" value="0.065"></div><br>

    <div>Roll:  P <input id="rp" type="number" step="0.001" value="0.07">
         I <input id="ri" type="number" step="0.001" value="0.00">
         D <input id="rd" type="number" step="0.001" value="0.065"></div><br>

    <div>Yaw:   P <input id="yp" type="number" step="0.001" value="0.005">
         I <input id="yi" type="number" step="0.001" value="0.00">
         D <input id="yd" type="number" step="0.01" value="0.00"></div><br>

    <button class="save" onclick="savePID()">Update Constants</button>
  </div>

  <div class="card">
    <h3>Z Damping Tuning</h3>
    <div>Z Set (g) <input id="zset" type="number" step="0.001" value="1.000">
         Kp <input id="zkp" type="number" step="0.01" value="3.00">
         Ki <input id="zki" type="number" step="0.01" value="0.25">
         Kd <input id="zkd" type="number" step="0.01" value="0.35"></div><br>

    <div>LPF Hz <input id="zcut" type="number" step="0.1" value="6.0">
         Deadband (g) <input id="zdb" type="number" step="0.001" value="0.020">
         Int Lim <input id="zint" type="number" step="0.1" value="2.0">
         Trim Lim (%) <input id="ztrim" type="number" step="0.1" value="6.0"></div><br>

    <button class="save" onclick="saveZ()">Update Z Params</button>
  </div>

<script>

  function savePID() {
    let p = `pp=${document.getElementById('pp').value}&pi=${document.getElementById('pi').value}&pd=${document.getElementById('pd').value}` +
            `&rp=${document.getElementById('rp').value}&ri=${document.getElementById('ri').value}&rd=${document.getElementById('rd').value}` +
            `&yp=${document.getElementById('yp').value}&yi=${document.getElementById('yi').value}&yd=${document.getElementById('yd').value}`;
    fetch("/pid?" + p);
  }

  function saveZ() {
    let p = `zset=${document.getElementById('zset').value}` +
            `&zkp=${document.getElementById('zkp').value}` +
            `&zki=${document.getElementById('zki').value}` +
            `&zkd=${document.getElementById('zkd').value}` +
            `&zcut=${document.getElementById('zcut').value}` +
            `&zdb=${document.getElementById('zdb').value}` +
            `&zint=${document.getElementById('zint').value}` +
            `&ztrim=${document.getElementById('ztrim').value}`;
    fetch("/zpid?" + p);
  }

  // ---------- WebSocket Telemetry ----------
  let ws;
  let logging = false;
  let logRows = [];
  const header = ["t_ms","roll_deg","pitch_deg","yaw_deg","throttle_pct","throttle_in_pct","az_g","azf_g","z_err_g","z_trim_pct","dt_s","m0_pct","m1_pct","m2_pct","m3_pct"];

  function connectWS() {
    const url = `ws://${location.host}/ws`;
    ws = new WebSocket(url);

    ws.onopen = () => console.log("[WS] connected", url);
    ws.onclose = () => { console.log("[WS] disconnected; retrying..."); setTimeout(connectWS, 500); };
    ws.onerror = (e) => console.log("[WS] error", e);

    ws.onmessage = (evt) => {
      try {
        const d = JSON.parse(evt.data);

        // UI updates
        document.getElementById("roll").textContent  = Number(d.r).toFixed(2);
        document.getElementById("pitch").textContent = Number(d.p).toFixed(2);
        document.getElementById("yaw").textContent   = Number(d.y).toFixed(2);
        document.getElementById("dt").textContent    = Number(d.dt).toFixed(4);

        document.getElementById("m0").textContent = Number(d.m0).toFixed(1);
        document.getElementById("m1").textContent = Number(d.m1).toFixed(1);
        document.getElementById("m2").textContent = Number(d.m2).toFixed(1);
        document.getElementById("m3").textContent = Number(d.m3).toFixed(1);

        // CSV logging
        if (logging) {
          logRows.push([d.t, d.r, d.p, d.y, d.th, d.th_in, d.az, d.azf, d.ze, d.zt, d.dt, d.m0, d.m1, d.m2, d.m3]);
          document.getElementById("rows").textContent = logRows.length;
        }
      } catch (e) {
        console.log("Bad telemetry:", e);
      }
    };
  }

  function startLog() { logging = true; }
  function stopLog()  { logging = false; }

  function downloadCSV() {
    let csv = header.join(",") + "\n";
    for (const r of logRows) {
      csv += r.map(v => (v === undefined || v === null) ? "" : v).join(",") + "\n";
    }
    const blob = new Blob([csv], { type: "text/csv" });
    const a = document.createElement("a");
    const ts = new Date().toISOString().replaceAll(":", "-");
    a.href = URL.createObjectURL(blob);
    a.download = `drone_log_${ts}.csv`;
    document.body.appendChild(a);
    a.click();
    a.remove();
  }

  window.addEventListener("load", connectWS);
</script></body></html>
)rawliteral";

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);

  // 1) Start Wi-Fi AP
  WiFi.softAP(ssid, password);
  Serial.print("Connect to Wi-Fi: "); Serial.println(ssid);
  Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());

  // 2) Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  RCControl::begin();
  Serial.println("SUCCESS: RC input initialized.");

  // server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
  //   if (request->hasParam("throttle")) {
  //     manual_throttle = request->getParam("throttle")->value().toFloat();
  //     if (manual_throttle < 0) manual_throttle = 0;
  //     if (manual_throttle > 100) manual_throttle = 100;
  //   }
  //   request->send(200, "text/plain", "OK");
  // });

  server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request){
    controller.updateGains(
      request->arg("pp").toFloat(), request->arg("pi").toFloat(), request->arg("pd").toFloat(),
      request->arg("rp").toFloat(), request->arg("ri").toFloat(), request->arg("rd").toFloat(),
      request->arg("yp").toFloat(), request->arg("yi").toFloat(), request->arg("yd").toFloat()
    );
    request->send(200, "text/plain", "Gains Applied");
  });

  server.on("/zpid", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasArg("zset")) z_accel_set_g = request->arg("zset").toFloat();
    if (request->hasArg("zkp")) z_kp = request->arg("zkp").toFloat();
    if (request->hasArg("zki")) z_ki = request->arg("zki").toFloat();
    if (request->hasArg("zkd")) z_kd = request->arg("zkd").toFloat();
    if (request->hasArg("zcut")) z_lpf_cutoff_hz = request->arg("zcut").toFloat();
    if (request->hasArg("zdb")) z_err_deadband_g = request->arg("zdb").toFloat();
    if (request->hasArg("zint")) z_int_limit = request->arg("zint").toFloat();
    if (request->hasArg("ztrim")) z_trim_limit = request->arg("ztrim").toFloat();

    if (z_lpf_cutoff_hz < 0.1f) z_lpf_cutoff_hz = 0.1f;
    if (z_err_deadband_g < 0.0f) z_err_deadband_g = 0.0f;
    if (z_int_limit < 0.0f) z_int_limit = 0.0f;
    if (z_trim_limit < 0.0f) z_trim_limit = 0.0f;

    request->send(200, "text/plain", "Z Params Applied");
  });

  // 3) WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();

  // 4) Hardware init
  if (!imu.begin()) {
    Serial.println("FATAL ERROR: ICM20948 IMU NOT FOUND! Check wiring (SDA/SCL).");
    while (1) { delay(1000); }
  }
  Serial.println("SUCCESS: IMU Initialized.");

  filter.begin(100.0f);
  Serial.println("SUCCESS: Filter Initialized.");

  for (int i = 0; i < 4; i++) {
    motors[i].begin(2000);   // keep your existing motor init param
  }

  last_micros = micros();
  Serial.println("Setup complete.");
}

// ===================== Main Loop =====================
void loop() {
  // We keep loop lightweight: update IMU/filter/control; stream telemetry at fixed rate.

  // const bool rc_ok = RCControl::isReceiverActive(100000); // 0 ms timeout
  const uint16_t chThrottle = RCControl::readChannelRaw(2);

  float throttle_cmd = (static_cast<float>(chThrottle) - 1000.0f) / 10.0f;
  if (throttle_cmd < 0.0f)   throttle_cmd = 0.0f;
  if (throttle_cmd > 100.0f) throttle_cmd = 100.0f;
  
  if (!imu.update()) {
    // Still keep WS alive / cleanup
    ws.cleanupClients();
    return;
  }

  if (!filter.update9(imu.data())) {
    ws.cleanupClients();
    return;
  }

  // Timing
  uint32_t current_micros = micros();
  float dt = (current_micros - last_micros) * 1e-6f;
  last_micros = current_micros;

  // Safety dt clamp
  if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;

  euler_t current_angle = filter.euler9();

  // Maintain heading, level roll/pitch
  euler_t target = {0.0f, 0.0f, current_angle.yaw_deg};

  // Kill logic
  // if (manual_throttle < 5.0f) {
  //   for (int i = 0; i < 4; i++) motors[i].stop();
  //   controller.resetIntegrals();
  // } else {
  //   controller.compute(target, current_angle, manual_throttle, dt);
  //   for (int i = 0; i < 4; i++) {
  //     motors[i].setPercent(controller.getMotorOutput(i)); // expects 0..100
  //   }
  // }

  // if (current_angle.pitch_deg > 45 || current_angle.pitch_deg < -45 ||
  //     current_angle.roll_deg > 45 || current_angle.roll_deg < -45) {
  //   for (int i = 0; i < 4; i++) motors[i].stop();
  //   controller.resetIntegrals();
  // } else
  float z_trim = 0.0f;
  float z_error = 0.0f;
  const float az_body_g = imu.data().az;

  if (throttle_cmd < 5.0f) {
      z_accel_filt_g = az_body_g;
      z_integral = 0.0f;
      z_prev_error = 0.0f;

      for (int i = 0; i < 4; i++) motors[i].stop();
      controller.resetIntegrals();
  } else {
      const float tau = 1.0f / (2.0f * (float)M_PI * z_lpf_cutoff_hz);
      const float alpha = tau / (tau + dt);
      z_accel_filt_g = alpha * z_accel_filt_g + (1.0f - alpha) * az_body_g;

      z_error = z_accel_set_g - z_accel_filt_g;
      if (fabsf(z_error) < z_err_deadband_g) z_error = 0.0f;

      z_integral += z_error * dt;
      if (z_integral > z_int_limit) z_integral = z_int_limit;
      if (z_integral < -z_int_limit) z_integral = -z_int_limit;

      const float z_derivative = (z_error - z_prev_error) / dt;
      z_prev_error = z_error;

      z_trim = (z_kp * z_error) +
               (z_ki * z_integral) +
               (z_kd * z_derivative);
      if (z_trim > z_trim_limit) z_trim = z_trim_limit;
      if (z_trim < -z_trim_limit) z_trim = -z_trim_limit;

      float throttle_stab = throttle_cmd + z_trim;
      if (throttle_stab < 0.0f) throttle_stab = 0.0f;
      if (throttle_stab > 100.0f) throttle_stab = 100.0f;

      controller.compute(target, current_angle, throttle_stab, dt);
      for (int i = 0; i < 4; i++) {
          motors[i].setPercent(controller.getMotorOutput(i));
      }

      telem.throttle = throttle_stab;
  }

  // Update telemetry snapshot
  telem.t_ms = millis();
  telem.roll = current_angle.roll_deg;
  telem.pitch = current_angle.pitch_deg;
  telem.yaw = current_angle.yaw_deg;
  telem.throttle_in = throttle_cmd;
  if (throttle_cmd < 5.0f) telem.throttle = throttle_cmd;
  telem.az = az_body_g;
  telem.az_f = z_accel_filt_g;
  telem.z_err = z_error;
  telem.z_trim = z_trim;
  telem.dt = dt;
  telem.m0 = controller.getMotorOutput(0);
  telem.m1 = controller.getMotorOutput(1);
  telem.m2 = controller.getMotorOutput(2);
  telem.m3 = controller.getMotorOutput(3);

  // WebSocket broadcast at 50 Hz
  uint32_t now_ms = telem.t_ms;
  if (now_ms - last_ws_send_ms >= WS_PERIOD_MS) {
    last_ws_send_ms = now_ms;
    ws.textAll(telemetryJson(telem));
    ws.cleanupClients();
  }

  // Serial monitoring (optional)
  static uint32_t last_print = 0;
  if (millis() - last_print > 100) {
    last_print = millis();
    Serial.print("Ang-> R:"); Serial.print(current_angle.roll_deg, 1);
    Serial.print(" P:"); Serial.print(current_angle.pitch_deg, 1);
    Serial.print(" Y:"); Serial.print(current_angle.yaw_deg, 1);
    Serial.print(" | Az:"); Serial.print(telem.az, 3);
    Serial.print(" AzF:"); Serial.print(telem.az_f, 3);
    Serial.print(" Zt:"); Serial.print(telem.z_trim, 2);
    Serial.print(" | Motors-> FL:"); Serial.print(telem.m0, 1);
    Serial.print("% FR:"); Serial.print(telem.m1, 1);
    Serial.print("% RL:"); Serial.print(telem.m2, 1);
    Serial.print("% RR:"); Serial.print(telem.m3, 1);
    Serial.println("%");
  }
}
