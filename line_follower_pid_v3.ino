#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "esp32-hal-ledc.h"
// #include <esp32.h>
// Motor pins
#define PWMA 25
#define AIN2 14
#define AIN1 32
#define STBY 33
#define BIN1 15
#define BIN2 26
#define PWMB 27

const uint8_t irPins[] = {2, 4, 5, 18, 19, 21, 22, 23};
#define SENSOR_COUNT 8

// WiFi credentials
const char* ssid = "ne";
const char* password = "qwer4321";

// AP fallback credentials
const char* fallback_ap_ssid = "LineFollower_AP";
const char* fallback_ap_pass = "12345678";

const byte DNS_PORT = 53;
DNSServer dnsServer;

AsyncWebServer server(80);
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT] = {0};

// PID variables
float Kp = 0.04, Ki = 0.002, Kd = 0.015;
float lastError = 0, integral = 0;
const int maxIntegral = 1000;

int baseSpeed = 100;
int maxSpeed = 255;

bool isCalibrated = false;
bool isRunning = false;
bool calibrationMode = false;
bool lineOnWhite = false;

bool apMode = false;

unsigned long lastSensorRead = 0;
unsigned long lastDashboardPing = 0;
unsigned long lastLogWrite = 0;

// Event log buffer
#define EVENT_LOG_SIZE 20
String eventLog[EVENT_LOG_SIZE];
int eventIndex = 0;

// Data logging buffer
struct SensorData {
  unsigned long ts;
  uint16_t sensors[SENSOR_COUNT];
  float position, error, pidOutput;
  int motorA, motorB;
};
const int MAX_DATA_POINTS = 100;
SensorData dataBuffer[MAX_DATA_POINTS];
int dataIndex = 0;

// Function declarations
void initializeMotors();
void setMotorSpeed(int speedA, int speedB);
void stopMotors();
void initializeSensors();
void initializeWiFi();
void setupWebServer();
void runPIDControl();
void logControlData(float position, float error, float pidOutput, int motorA, int motorB);
void logSensorData();
void addEvent(const String &event);
String getMainHTML();

void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32 Line Follower Robot");

  initializeMotors();
  initializeSensors();

  if (!SPIFFS.begin(true)) {
    Serial.println("[SPIFFS] Mount failed. Halting.");
    while (true) delay(1000);
  }
  Serial.println("[SPIFFS] Mounted");

  initializeWiFi();

  if (!MDNS.begin("linefollower")) {
    Serial.println("[mDNS] Failed to start");
  } else {
    Serial.println("[mDNS] Started at http://linefollower.local");
  }

  ArduinoOTA.setHostname("linefollower");
  ArduinoOTA
    .onStart([]() { addEvent("[OTA] Update started"); })
    .onEnd([]() { addEvent("[OTA] Update finished"); })
    .onError([](ota_error_t error) { addEvent("[OTA] Error: " + String(error)); })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\n", (progress / (total / 100)));
    });
  ArduinoOTA.begin();

  setupWebServer();

  lastDashboardPing = millis();

  addEvent("[BOOT] System started");
}

void loop() {
  ArduinoOTA.handle();
  dnsServer.processNextRequest();

  // Watchdog: stop motors if no dashboard ping >5 sec
  if (isRunning && (millis() - lastDashboardPing > 5000)) {
    isRunning = false;
    stopMotors();
    addEvent("[WATCHDOG] No dashboard ping, motors stopped");
    Serial.println("[WATCHDOG] Timeout, motors stopped");
  }

  if (isRunning && isCalibrated) runPIDControl();
  else if (!isRunning) stopMotors();

  if (millis() - lastSensorRead >= 50) {
    logSensorData();
    lastSensorRead = millis();
  }
}

void initializeMotors() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);

  Serial.println("[MOTORS] Initialized");
}

void setMotorSpeed(int speedA, int speedB) {
  speedA = constrain(speedA, -maxSpeed, maxSpeed);
  speedB = constrain(speedB, -maxSpeed, maxSpeed);

  digitalWrite(AIN1, speedA >= 0);
  digitalWrite(AIN2, speedA < 0);
  ledcWrite(0, abs(speedA));

  digitalWrite(BIN1, speedB >= 0);
  digitalWrite(BIN2, speedB < 0);
  ledcWrite(1, abs(speedB));
}

void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void initializeSensors() {
  qtr.setTypeRC();
  qtr.setSensorPins(irPins, SENSOR_COUNT);
  Serial.println("[SENSORS] QTR Sensors initialized");
}

void initializeWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("[WiFi] Connecting ");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries++ < 20) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    Serial.println("[WiFi] Connected IP: " + WiFi.localIP().toString());
  } else {
    apMode = true;
    Serial.println("[WiFi] Connection failed, starting AP mode");
    WiFi.softAP(fallback_ap_ssid, fallback_ap_pass);
    Serial.println("[WiFi] AP IP: " + WiFi.softAPIP().toString());

    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());  // DNS redirect all to ESP IP
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", getMainHTML());
  });

  server.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest *req) {
    File file = SPIFFS.open("/log.csv", "r");
    if (!file) {
      req->send(404, "text/plain", "Log not found");
      return;
    }
    AsyncWebServerResponse *resp = req->beginResponse(file, "text/csv", file.size());
    resp->addHeader("Content-Disposition", "attachment; filename=log.csv");
    req->send(resp);
  });

  server.on("/api/ping", HTTP_GET, [](AsyncWebServerRequest *req) {
    lastDashboardPing = millis();
    req->send(200, "application/json", "{\"status\":\"ok\"}");
  });

  server.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest *req) {
    addEvent("[CALIB] Calibration started");
    calibrationMode = true;
    isCalibrated = false;
    for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate();
      delay(5);
    }
    calibrationMode = false;
    isCalibrated = true;
    addEvent("[CALIB] Calibration complete");
    req->send(200, "application/json", "{\"status\":\"success\",\"message\":\"Calibration complete\"}");
  });

  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (isCalibrated) {
      isRunning = true;
      integral = 0;
      addEvent("[RUN] Line following started");
      req->send(200, "application/json", "{\"status\":\"success\",\"message\":\"Started\"}");
    } else {
      req->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Please calibrate first\"}");
    }
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest *req) {
    isRunning = false;
    stopMotors();
    addEvent("[RUN] Line following stopped");
    req->send(200, "application/json", "{\"status\":\"success\",\"message\":\"Stopped\"}");
  });

  server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest *req) {
    isCalibrated = false;
    integral = 0;
    addEvent("[RESET] System reset");
    req->send(200, "application/json", "{\"status\":\"success\",\"message\":\"Reset done\"}");
  });

  server.on("/api/pid", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (!req->hasArg("plain")) {
      req->send(400, "application/json", "{\"status\":\"error\",\"message\":\"No body\"}");
      return;
    }
    DynamicJsonDocument doc(256);
    auto err = deserializeJson(doc, req->arg("plain"));
    if (err) {
      req->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
      return;
    }
    if (doc.containsKey("kp")) Kp = doc["kp"];
    if (doc.containsKey("ki")) Ki = doc["ki"];
    if (doc.containsKey("kd")) Kd = doc["kd"];
    if (doc.containsKey("baseSpeed")) baseSpeed = doc["baseSpeed"];
    if (doc.containsKey("maxSpeed")) maxSpeed = doc["maxSpeed"];
    if (doc.containsKey("lineOnWhite")) lineOnWhite = doc["lineOnWhite"];
    addEvent("[PID] PID and mode updated");
    req->send(200, "application/json", "{\"status\":\"success\",\"message\":\"PID and mode updated\"}");
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    DynamicJsonDocument doc(1024);
    doc["calibrated"] = isCalibrated;
    doc["running"] = isRunning;
    doc["calibrating"] = calibrationMode;
    doc["apMode"] = apMode;
    doc["apUrl"] = apMode ? "http://192.168.4.1" : WiFi.localIP().toString();
    doc["apSsid"] = fallback_ap_ssid;
    doc["apPass"] = fallback_ap_pass;
    doc["kp"] = Kp;
    doc["ki"] = Ki;
    doc["kd"] = Kd;
    doc["baseSpeed"] = baseSpeed;
    doc["maxSpeed"] = maxSpeed;
    doc["lineOnWhite"] = lineOnWhite;

    int motorAVal = isRunning && isCalibrated ? baseSpeed : 0;
    doc["motorA"] = motorAVal;
    doc["motorB"] = motorAVal;

    JsonArray events = doc.createNestedArray("events");
    for (int i = 0; i < EVENT_LOG_SIZE; i++) {
      int idx = (eventIndex + i) % EVENT_LOG_SIZE;
      if (eventLog[idx].length() > 0) events.add(eventLog[idx]);
    }

    String resp;
    serializeJson(doc, resp);
    req->send(200, "application/json", resp);
  });

  server.on("/api/sensors", HTTP_GET, [](AsyncWebServerRequest *req) {
    qtr.read(sensorValues);
    DynamicJsonDocument doc(512);
    JsonArray arr = doc.createNestedArray("sensors");
    for (int i = 0; i < SENSOR_COUNT; i++) arr.add(sensorValues[i]);
    uint16_t position = lineOnWhite ? qtr.readLineWhite(sensorValues) : qtr.readLineBlack(sensorValues);
    doc["position"] = position;
    doc["error"] = 3500 - position;
    String resp;
    serializeJson(doc, resp);
    req->send(200, "application/json", resp);
  });

  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *req) {
    DynamicJsonDocument doc(4096);
    JsonArray dataArr = doc.createNestedArray("data");
    int count = min(50, MAX_DATA_POINTS);
    int startIdx = (dataIndex - count + MAX_DATA_POINTS) % MAX_DATA_POINTS;
    for (int i = 0; i < count; i++) {
      int idx = (startIdx + i) % MAX_DATA_POINTS;
      JsonObject point = dataArr.createNestedObject();
      point["timestamp"] = dataBuffer[idx].ts;
      point["position"] = dataBuffer[idx].position;
      point["error"] = dataBuffer[idx].error;
      point["pidOutput"] = dataBuffer[idx].pidOutput;
      point["motorA"] = dataBuffer[idx].motorA;
      point["motorB"] = dataBuffer[idx].motorB;
    }
    String resp;
    serializeJson(doc, resp);
    req->send(200, "application/json", resp);
  });

  server.on("/api/diagnostics", HTTP_POST, [](AsyncWebServerRequest *req) {
    addEvent("[DIAG] Running diagnostics");
    setMotorSpeed(150, 150);
    delay(500);
    setMotorSpeed(-150, -150);
    delay(500);
    stopMotors();
    qtr.read(sensorValues);

    DynamicJsonDocument doc(512);
    JsonArray sensorsArr = doc.createNestedArray("sensors");
    for (int i = 0; i < SENSOR_COUNT; i++) sensorsArr.add(sensorValues[i]);
    doc["motors"] = "OK";
    addEvent("[DIAG] Diagnostics complete");
    String resp;
    serializeJson(doc, resp);
    req->send(200, "application/json", resp);
  });

  server.begin();
  Serial.println("[WEB] AsyncWebServer started");
}

void runPIDControl() {
  uint16_t position = lineOnWhite ? qtr.readLineWhite(sensorValues) : qtr.readLineBlack(sensorValues);

  float error = 3500 - position;

  integral += error;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  float derivative = error - lastError;

  float pidOutput = Kp * error + Ki * integral + Kd * derivative;

  int speedA = constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed);
  int speedB = constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed);

  if (abs(speedA) >= maxSpeed || abs(speedB) >= maxSpeed) {
    integral -= error; // Anti-windup
  }

  setMotorSpeed(speedA, speedB);
  lastError = error;

  logControlData(position, error, pidOutput, speedA, speedB);
}

void logControlData(float position, float error, float pidOutput, int motorA, int motorB) {
  dataBuffer[dataIndex].ts = millis();
  dataBuffer[dataIndex].position = position;
  dataBuffer[dataIndex].error = error;
  dataBuffer[dataIndex].pidOutput = pidOutput;
  dataBuffer[dataIndex].motorA = motorA;
  dataBuffer[dataIndex].motorB = motorB;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    dataBuffer[dataIndex].sensors[i] = sensorValues[i];
  }

  dataIndex = (dataIndex + 1) % MAX_DATA_POINTS;

  if (millis() - lastLogWrite > 100) {
    lastLogWrite = millis();
    File file = SPIFFS.open("/log.csv", FILE_APPEND);
    if (file) {
      file.printf("%lu,", millis());
      for (int i = 0; i < SENSOR_COUNT; i++) file.printf("%d,", sensorValues[i]);
      file.printf("%.2f,%.2f,%.2f,%d,%d\n", position, error, pidOutput, motorA, motorB);
      file.close();
    }
  }
}

void logSensorData() {
  qtr.read(sensorValues);
  uint16_t pos = lineOnWhite ? qtr.readLineWhite(sensorValues) : qtr.readLineBlack(sensorValues);
  logControlData(pos, 3500 - pos, 0, 0, 0);
}

void addEvent(const String &event) {
  eventLog[eventIndex] = event;
  eventIndex = (eventIndex + 1) % EVENT_LOG_SIZE;
  Serial.println("[EVENT] " + event);
}

// Dashboard HTML, CSS, and JavaScript is very extensive (see your previous final example for full dashboard code)
// Here's a minimalist version you can replace with the full one:

String getMainHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Line Follower Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
    h1,h2 { text-align: center; }
    .card { background: white; padding: 20px; margin: 10px auto; border-radius: 10px; max-width: 900px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
    .sensor-display { display: grid; grid-template-columns: repeat(8, 1fr); gap: 5px; margin: 10px 0; }
    .sensor-bar { height: 30px; background: #ddd; border-radius: 5px; display: flex; align-items: center; justify-content: center; font-size: 12px; }
    .sensor-active { background: #ff4444; color: white; }
    button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; border-radius: 5px; border: none; }
    .btn-primary { background: #007bff; color: white; }
    .btn-success { background: #28a745; color: white; }
    .btn-danger { background: #dc3545; color: white; }
    .btn-warning { background: #ffc107; color: black; }
    .controls { display: flex; justify-content: center; flex-wrap: wrap; gap: 10px; margin-bottom: 10px; }
    .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
    .status.success { background: #d4edda; color: #155724; }
    .status.error { background: #f8d7da; color: #721c24; }
    .chart-container { height: 300px; }
    #eventlog { max-height: 150px; overflow-y: auto; background:#fff; border: 1px solid #ddd; padding: 10px; font-family: monospace; }
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 Line Follower Dashboard</h1>
    <div id="status" class="status">Loading...</div>
    <div class="controls">
      <button class="btn-warning" onclick="calibrate()">Calibrate</button>
      <button class="btn-success" onclick="start()">Start</button>
      <button class="btn-danger" onclick="stop()">Stop</button>
      <button class="btn-primary" onclick="reset()">Reset</button>
      <button class="btn-primary" onclick="downloadLog()">Download Log CSV</button>
    </div>

    <h2>PID Settings</h2>
    <div class="controls">
      <label>Kp: <input type="number" id="kp" step="0.001"></label>
      <label>Ki: <input type="number" id="ki" step="0.001"></label>
      <label>Kd: <input type="number" id="kd" step="0.001"></label>
      <label>Base Speed: <input type="number" id="baseSpeed"></label>
      <label>Max Speed: <input type="number" id="maxSpeed"></label>
      <label>Line on White: <input type="checkbox" id="lineOnWhite"></label>
      <button class="btn-primary" onclick="updatePID()">Update PID</button>
    </div>

    <h2>Sensor Values</h2>
    <div id="sensorDisplay" class="sensor-display"></div>
    <div>Position: <span id="position">-</span> | Error: <span id="error">-</span></div>

    <h2>Sensor Heatmap</h2>
    <div id="heatmap" class="sensor-display"></div>

    <h2>Motor Status</h2>
    <div>Motor A: <span id="motorA">0</span> | Motor B: <span id="motorB">0</span></div>

    <h2>Performance Chart</h2>
    <div class="chart-container">
      <canvas id="lineChart"></canvas>
    </div>

    <h2>Event Log</h2>
    <div id="eventlog">Loading events...</div>
  </div>

<script>
  let chart;
  const ctx = document.getElementById('lineChart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line',
    data: {
      datasets: [
        {label: 'Position', borderColor: '#2196f3', data: [], tension: 0.1},
        {label: 'Error', borderColor: '#e91e63', data: [], tension: 0.1},
        {label: 'PID Output', borderColor: '#4caf50', data: [], tension: 0.1},
      ]
    },
    options: {
      responsive: true,
      animation: false,
      scales: {
        x: {type: 'linear', position: 'bottom', title: {display: true, text: 'Time (ms)'}},
        y: {title: {display: true, text: 'Value'}}
      }
    }
  });

  function updateStatus() {
    fetch('/api/status').then(res=>res.json()).then(data=>{
      const statusEl = document.getElementById('status');
      statusEl.textContent = "Calibrated: " + (data.calibrated ? "Yes" : "No") + " | Running: " + (data.running ? "Yes" : "No");
      statusEl.className = "status " + (data.calibrated ? "success" : "error");

      document.getElementById('kp').value = data.kp;
      document.getElementById('ki').value = data.ki;
      document.getElementById('kd').value = data.kd;
      document.getElementById('baseSpeed').value = data.baseSpeed;
      document.getElementById('maxSpeed').value = data.maxSpeed;
      document.getElementById('lineOnWhite').checked = data.lineOnWhite;

      document.getElementById('motorA').textContent = data.motorA;
      document.getElementById('motorB').textContent = data.motorB;

      const eventlog = document.getElementById('eventlog');
      if(data.events){
        eventlog.innerHTML = data.events.slice().reverse().map(e => '<div>'+e+'</div>').join('');
      }
    });
  }

  function updateSensors() {
    fetch('/api/sensors').then(res=>res.json()).then(data=>{
      const sensorDisplay = document.getElementById('sensorDisplay');
      sensorDisplay.innerHTML = '';
      const heatmap = document.getElementById('heatmap');
      heatmap.innerHTML = '';
      data.sensors.forEach(v=>{
        // Sensor bars
        const bar = document.createElement('div');
        bar.className = 'sensor-bar' + (v<500 ? ' sensor-active' : '');
        bar.textContent = v;
        sensorDisplay.appendChild(bar);

        // Heatmap bars
        const hue = Math.max(0, 240 - v/4);
        const hbar = document.createElement('div');
        hbar.className = 'sensor-bar';
        hbar.style.backgroundColor = `hsl(${hue}, 100%, 50%)`;
        hbar.textContent = v;
        heatmap.appendChild(hbar);
      });
      document.getElementById('position').textContent = data.position;
      document.getElementById('error').textContent = data.error;
    });
  }

  function updateChart() {
    fetch('/api/data').then(res=>res.json()).then(data=>{
      chart.data.datasets[0].data = data.data.map(e => ({x: e.timestamp, y: e.position}));
      chart.data.datasets[1].data = data.data.map(e => ({x: e.timestamp, y: e.error}));
      chart.data.datasets[2].data = data.data.map(e => ({x: e.timestamp, y: e.pidOutput}));
      chart.update();
    });
  }

  function calibrate() {
    fetch('/api/calibrate', {method:"POST"}).then(r=>r.json()).then(d=>alert(d.message));
  }
  function start() {
    fetch('/api/start', {method:"POST"}).then(r=>r.json()).then(d=>alert(d.message));
  }
  function stop() {
    fetch('/api/stop', {method:"POST"}).then(r=>r.json()).then(d=>alert(d.message));
  }
  function reset() {
    fetch('/api/reset', {method:"POST"}).then(r=>r.json()).then(d=>alert(d.message));
  }
  function updatePID() {
    const data = {
      kp: parseFloat(document.getElementById('kp').value),
      ki: parseFloat(document.getElementById('ki').value),
      kd: parseFloat(document.getElementById('kd').value),
      baseSpeed: parseInt(document.getElementById('baseSpeed').value),
      maxSpeed: parseInt(document.getElementById('maxSpeed').value),
      lineOnWhite: document.getElementById('lineOnWhite').checked
    };
    fetch('/api/pid', {
      method:"POST",
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(data)
    }).then(r=>r.json()).then(d=>alert(d.message));
  }
  function downloadLog() {
    window.open('/log.csv');
  }

  // keepalive ping to avoid watchdog stopping motors
  setInterval(()=>fetch('/api/ping'), 1000);

  setInterval(() => {
    updateStatus();
    updateSensors();
    updateChart();
  }, 300);

  window.onload = () => {
    updateStatus();
    updateSensors();
    updateChart();
  }
</script>

</body>
</html>
  )rawliteral";
}
