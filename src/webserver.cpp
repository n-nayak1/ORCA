#include "webserver.hpp"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

namespace {

AsyncWebServer g_server(80);
AsyncWebSocket g_ws("/ws");

PidGainsHandler g_pid_handler = nullptr;
AltitudeParamsHandler g_alt_handler = nullptr;

String telemetryJson(const TelemetrySnapshot& s) {
  String out;
  out.reserve(240);
  out += "{";
  out += "\"t\":"; out += s.t_ms;
  out += ",\"r\":"; out += String(s.roll, 2);
  out += ",\"p\":"; out += String(s.pitch, 2);
  out += ",\"y\":"; out += String(s.yaw, 2);
  out += ",\"th\":"; out += String(s.throttle, 1);
  out += ",\"th_in\":"; out += String(s.throttle_in, 1);
  out += ",\"as\":"; out += String(s.alt_set_mm, 1);
  out += ",\"am\":"; out += String(s.alt_meas_mm, 1);
  out += ",\"af\":"; out += String(s.alt_filt_mm, 1);
  out += ",\"ae\":"; out += String(s.alt_err_mm, 1);
  out += ",\"at\":"; out += String(s.alt_trim, 2);
  out += ",\"ah\":"; out += String(s.hover_throttle, 1);
  out += ",\"axp\":"; out += String(s.alt_expo, 2);
  out += ",\"dt\":"; out += String(s.dt, 4);
  out += ",\"m0\":"; out += String(s.m0, 1);
  out += ",\"m1\":"; out += String(s.m1, 1);
  out += ",\"m2\":"; out += String(s.m2, 1);
  out += ",\"m3\":"; out += String(s.m3, 1);
  out += ",\"aok\":"; out += String(s.alt_ok);
  out += "}";
  return out;
}

void onWsEvent(AsyncWebSocket* server,
               AsyncWebSocketClient* client,
               AwsEventType type,
               void* arg,
               uint8_t* data,
               size_t len) {
  (void)server;
  (void)arg;
  (void)data;
  (void)len;
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected\n", client->id());
  }
}

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
    <div class="mono" style="margin-top:10px;">Altitude Set: <span id="alt_set">--</span> mm</div>
    <div class="mono" style="margin-top:10px;">Altitude Meas: <span id="alt_meas">--</span> mm (<span id="alt_ok">--</span>)</div>
    <div class="mono" style="margin-top:10px;">Altitude Trim: <span id="alt_trim">--</span>% | Hover Base: <span id="alt_hover">--</span>% | Expo: <span id="alt_expo">--</span></div>
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
    <h3>Altitude Hold Tuning</h3>
    <div>Alt Set (mm) <input id="zset" type="number" step="10" value="0">
         Kp <input id="zkp" type="number" step="0.001" value="0.0001">
         Ki <input id="zki" type="number" step="0.0001" value="0.000">
         Kd <input id="zkd" type="number" step="0.001" value="0.000"></div><br>

    <div>Deadband (mm) <input id="zdb" type="number" step="1" value="20">
         Int Lim <input id="zint" type="number" step="10" value="2000">
         Trim Lim (%) <input id="ztrim" type="number" step="0.1" value="20.0"></div><br>
    <div>Hover Throttle (%) <input id="zhover" type="number" step="0.1" value="5.0">
         Setpoint Expo <input id="zexpo" type="number" step="0.1" value="2.0"></div><br>

    <button class="save" onclick="saveZ()">Update Altitude Params</button>
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
            `&zdb=${document.getElementById('zdb').value}` +
            `&zint=${document.getElementById('zint').value}` +
            `&ztrim=${document.getElementById('ztrim').value}` +
            `&zhover=${document.getElementById('zhover').value}` +
            `&zexpo=${document.getElementById('zexpo').value}`;
    fetch("/zpid?" + p);
  }

  let ws;
  let logging = false;
  let logRows = [];
  const header = ["t_ms","roll_deg","pitch_deg","yaw_deg","throttle_pct","throttle_in_pct","alt_set_mm","alt_meas_mm","alt_filt_mm","alt_err_mm","alt_trim_pct","hover_throttle_pct","alt_expo","alt_ok","dt_s","m0_pct","m1_pct","m2_pct","m3_pct"];

  function connectWS() {
    const url = `ws://${location.host}/ws`;
    ws = new WebSocket(url);

    ws.onopen = () => console.log("[WS] connected", url);
    ws.onclose = () => { console.log("[WS] disconnected; retrying..."); setTimeout(connectWS, 500); };
    ws.onerror = (e) => console.log("[WS] error", e);

    ws.onmessage = (evt) => {
      try {
        const d = JSON.parse(evt.data);

        document.getElementById("roll").textContent  = Number(d.r).toFixed(2);
        document.getElementById("pitch").textContent = Number(d.p).toFixed(2);
        document.getElementById("yaw").textContent   = Number(d.y).toFixed(2);
        document.getElementById("dt").textContent    = Number(d.dt).toFixed(4);

        document.getElementById("m0").textContent = Number(d.m0).toFixed(1);
        document.getElementById("m1").textContent = Number(d.m1).toFixed(1);
        document.getElementById("m2").textContent = Number(d.m2).toFixed(1);
        document.getElementById("m3").textContent = Number(d.m3).toFixed(1);
        document.getElementById("alt_set").textContent = Number(d.as).toFixed(1);
        document.getElementById("alt_meas").textContent = Number(d.am).toFixed(1);
        document.getElementById("alt_trim").textContent = Number(d.at).toFixed(2);
        document.getElementById("alt_hover").textContent = Number(d.ah).toFixed(1);
        document.getElementById("alt_expo").textContent = Number(d.axp).toFixed(2);
        document.getElementById("alt_ok").textContent = Number(d.aok) ? "ok" : "stale";

        if (logging) {
          logRows.push([d.t, d.r, d.p, d.y, d.th, d.th_in, d.as, d.am, d.af, d.ae, d.at, d.ah, d.axp, d.aok, d.dt, d.m0, d.m1, d.m2, d.m3]);
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

} // namespace

void WebDashboard::begin(const char* ssid,
                         const char* password,
                         PidGainsHandler pid_handler,
                         AltitudeParamsHandler alt_handler) {
  g_pid_handler = pid_handler;
  g_alt_handler = alt_handler;

  WiFi.softAP(ssid, password);
  Serial.print("Connect to Wi-Fi: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  g_server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });

  g_server.on("/pid", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (g_pid_handler != nullptr) {
      PidGainsRequest gains;
      gains.pp = request->arg("pp").toFloat();
      gains.pi = request->arg("pi").toFloat();
      gains.pd = request->arg("pd").toFloat();
      gains.rp = request->arg("rp").toFloat();
      gains.ri = request->arg("ri").toFloat();
      gains.rd = request->arg("rd").toFloat();
      gains.yp = request->arg("yp").toFloat();
      gains.yi = request->arg("yi").toFloat();
      gains.yd = request->arg("yd").toFloat();
      g_pid_handler(gains);
    }
    request->send(200, "text/plain", "Gains Applied");
  });

  g_server.on("/zpid", HTTP_GET, [this](AsyncWebServerRequest* request) {
    if (request->hasArg("zset")) alt_params_.set_mm = request->arg("zset").toFloat();
    if (request->hasArg("zkp")) alt_params_.kp = request->arg("zkp").toFloat();
    if (request->hasArg("zki")) alt_params_.ki = request->arg("zki").toFloat();
    if (request->hasArg("zkd")) alt_params_.kd = request->arg("zkd").toFloat();
    if (request->hasArg("zdb")) alt_params_.deadband_mm = request->arg("zdb").toFloat();
    if (request->hasArg("zint")) alt_params_.int_limit = request->arg("zint").toFloat();
    if (request->hasArg("ztrim")) alt_params_.trim_limit = request->arg("ztrim").toFloat();
    if (request->hasArg("zhover")) alt_params_.hover_throttle = request->arg("zhover").toFloat();
    if (request->hasArg("zexpo")) alt_params_.set_expo = request->arg("zexpo").toFloat();

    if (g_alt_handler != nullptr) g_alt_handler(alt_params_);
    request->send(200, "text/plain", "Altitude Params Applied");
  });

  g_ws.onEvent(onWsEvent);
  g_server.addHandler(&g_ws);
  g_server.begin();
}

void WebDashboard::publishTelemetry(const TelemetrySnapshot& telem) {
  g_ws.textAll(telemetryJson(telem));
}

void WebDashboard::cleanupClients() {
  g_ws.cleanupClients();
}
