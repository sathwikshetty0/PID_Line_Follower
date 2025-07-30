#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>

// =============== CONFIGURATION ===============
#define PWMA 25
#define AIN2 14
#define AIN1 32
#define STBY 33
#define BIN1 15
#define BIN2 26
#define PWMB 27

const uint8_t irPins[] = {2,4,5,18,19,21,22,23};
#define SENSOR_COUNT 8

const char* ssid = "ne";
const char* password = "qwer4321";

// =============== GLOBAL VARIABLES ===============
QTRSensors qtr;
WebServer server(80);
uint16_t sensorValues[SENSOR_COUNT];

float Kp = 0.04, Ki = 0.002, Kd = 0.015, lastError = 0, integral = 0;
int baseSpeed = 100, maxSpeed = 255;
bool isCalibrated = false, isRunning = false, calibrationMode = false;
uint16_t sensorMin[SENSOR_COUNT], sensorMax[SENSOR_COUNT], sensorThreshold[SENSOR_COUNT];
unsigned long sensorActivityCount[SENSOR_COUNT] = {0};

struct SensorData {
  unsigned long timestamp = 0;
  uint16_t sensors[SENSOR_COUNT] = {0};
  float position = 0, error = 0, pidOutput = 0, speed = 0, acceleration = 0;
  int motorA = 0, motorB = 0;
};

const int MAX_DATA_POINTS = 100;
SensorData dataBuffer[MAX_DATA_POINTS];
int dataIndex = 0;

int manualMotorA = 0, manualMotorB = 0;
bool manualControlActive = false;

// Direction visualization state
int motorADirection = 0, motorBDirection = 0;
unsigned long lastSensorRead = 0;

// =============== FUNCTION DECLARATIONS ===============
void handleRoot(), handleCalibrate(), handleStart(), handleStop(), handlePIDUpdate(), handleStatus();
void handleSensors(), handleData(), handleManualMotor(), handleSetManualMode(), handleCalibrationData();
void handleGetThresholds(), handleSetThresholds(), handleHeatMap(), handleAutoTune();

void initializeMotors(), initializeSensors(), initializeWiFi(), setupWebServer();
void runPIDControl(), setMotorSpeed(int speedA, int speedB), stopMotors();
void logControlData(float position, float error, float pidOutput, int motorA, int motorB), logSensorData();
String getMainHTML();

// =============== SETUP & LOOP ===============
void setup() {
  Serial.begin(115200);
  initializeMotors();
  initializeSensors();
  initializeWiFi();
  setupWebServer();
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4000;  sensorMax[i] = 0; sensorThreshold[i] = 2000; sensorActivityCount[i] = 0;
  }
}

void loop() {
  server.handleClient();
  if (isRunning && isCalibrated && !manualControlActive && !calibrationMode) runPIDControl();
  else if (manualControlActive) setMotorSpeed(manualMotorA, manualMotorB);
  else stopMotors();
  if (millis() - lastSensorRead > 50) { logSensorData(); lastSensorRead = millis(); }
}

// =============== INITIALIZATION ===============
void initializeMotors() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT); digitalWrite(STBY, HIGH); stopMotors();
  Serial.println("Motors initialized");
}
void initializeSensors() { qtr.setTypeRC(); qtr.setSensorPins(irPins, SENSOR_COUNT); Serial.println("QTR sensors initialized"); }
void initializeWiFi() {
  WiFi.mode(WIFI_STA); WiFi.begin(ssid, password); int attempts = 0; Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); Serial.print("."); attempts++; }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi connection failed! Restarting..."); ESP.restart(); }
  Serial.println("WiFi connected! IP: " + WiFi.localIP().toString());
}
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/pid", HTTP_POST, handlePIDUpdate);
  server.on("/api/autotune", HTTP_POST, handleAutoTune);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/sensors", HTTP_GET, handleSensors);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/manual", HTTP_POST, handleManualMotor);
  server.on("/api/manualmode", HTTP_POST, handleSetManualMode);
  server.on("/api/calibrationdata", HTTP_GET, handleCalibrationData);
  server.on("/api/thresholds", HTTP_GET, handleGetThresholds);
  server.on("/api/thresholds", HTTP_POST, handleSetThresholds);
  server.on("/api/heatmap", HTTP_GET, handleHeatMap);
  server.begin();
  Serial.println("Web server started");
}

// =============== WEB HANDLERS ===============
void handleRoot()                { server.send(200, "text/html", getMainHTML()); }
void handleCalibrate() {
  if (calibrationMode) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Calibration already running\"}"); return; }
  calibrationMode = true; isCalibrated = false; Serial.println("Starting calibration...");
  for (int i = 0; i < SENSOR_COUNT; i++) { sensorMin[i] = 4000; sensorMax[i] = 0; }
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate(); qtr.read(sensorValues);
    for (int s = 0; s < SENSOR_COUNT; s++) {
      if (sensorValues[s] < sensorMin[s]) sensorMin[s] = sensorValues[s];
      if (sensorValues[s] > sensorMax[s]) sensorMax[s] = sensorValues[s];
    }
    if (i % 40 == 0) Serial.print("."); delay(5);
  }
  for (int i = 0; i < SENSOR_COUNT; i++) sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  isCalibrated = true; calibrationMode = false; Serial.println("\nCalibration complete");
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Calibration complete\"}");
}
void handleStart() {
  if (!isCalibrated) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Calibrate first\"}"); return; }
  isRunning = true; manualControlActive = false; integral = 0;
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Started line following\"}");
}
void handleStop() {
  isRunning = false; manualControlActive = false; stopMotors();
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Stopped\"}");
}
void handlePIDUpdate() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}"); return; }
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}"); return; }
  if (doc.containsKey("kp")) Kp = doc["kp"]; if (doc.containsKey("ki")) Ki = doc["ki"];
  if (doc.containsKey("kd")) Kd = doc["kd"]; if (doc.containsKey("baseSpeed")) baseSpeed = doc["baseSpeed"];
  if (doc.containsKey("maxSpeed")) maxSpeed = doc["maxSpeed"];
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"PID updated\"}");
}
void handleAutoTune() {
  // Placeholder for auto-tune routine (user can expand with algorithm)
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Auto-tune PID (not implemented)\"}");
}
void handleStatus() {
  DynamicJsonDocument doc(512);
  doc["calibrated"] = isCalibrated; doc["running"] = isRunning; doc["calibrating"] = calibrationMode;
  doc["kp"] = Kp; doc["ki"] = Ki; doc["kd"] = Kd; doc["baseSpeed"] = baseSpeed; doc["maxSpeed"] = maxSpeed;
  doc["manualControl"] = manualControlActive; doc["motorADirection"] = motorADirection; doc["motorBDirection"] = motorBDirection;
  doc["manualMotorA"] = manualMotorA; doc["manualMotorB"] = manualMotorB;
  String response; serializeJson(doc, response); server.send(200, "application/json", response);
}
void handleSensors() {
  qtr.read(sensorValues);
  for (int i = 0; i < SENSOR_COUNT; i++) if (sensorValues[i] < sensorThreshold[i]) sensorActivityCount[i]++;
  DynamicJsonDocument doc(2048);
  JsonArray sensorsRaw = doc.createNestedArray("sensorsRaw");
  JsonArray sensorsThresholds = doc.createNestedArray("thresholds");
  for (int i = 0; i < SENSOR_COUNT; i++) { sensorsRaw.add(sensorValues[i]); sensorsThresholds.add(sensorThreshold[i]); }
  if (isCalibrated) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    doc["position"] = position; doc["error"] = 3500 - position;
  }
  String response; serializeJson(doc, response); server.send(200, "application/json", response);
}
void handleData() {
  DynamicJsonDocument doc(6144);
  JsonArray dataArray = doc.createNestedArray("data");
  int count = min(50, MAX_DATA_POINTS); int startIndex = (dataIndex - count + MAX_DATA_POINTS) % MAX_DATA_POINTS;
  for (int i = 0; i < count; i++) {
    int idx = (startIndex + i) % MAX_DATA_POINTS;
    JsonObject point = dataArray.createNestedObject();
    point["timestamp"] = dataBuffer[idx].timestamp;
    point["position"] = dataBuffer[idx].position; point["error"] = dataBuffer[idx].error; point["pidOutput"] = dataBuffer[idx].pidOutput;
    point["motorA"] = dataBuffer[idx].motorA; point["motorB"] = dataBuffer[idx].motorB; point["speed"] = dataBuffer[idx].speed; point["acceleration"] = dataBuffer[idx].acceleration;
  }
  String response; serializeJson(doc, response); server.send(200, "application/json", response);
}
void handleManualMotor() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}"); return; }
  DynamicJsonDocument doc(256); DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}"); return; }
  manualMotorA = doc["motorA"]; manualMotorB = doc["motorB"]; manualControlActive = true; isRunning = false;
  setMotorSpeed(manualMotorA, manualMotorB); server.send(200, "application/json", "{\"status\":\"success\"}");
}
void handleSetManualMode() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}"); return; }
  DynamicJsonDocument doc(64); DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}"); return; }
  manualControlActive = doc["manual"] == true; if (!manualControlActive) stopMotors();
  server.send(200, "application/json", "{\"status\":\"success\"}");
}
void handleCalibrationData() {
  DynamicJsonDocument doc(512);
  JsonArray minVals = doc.createNestedArray("minValues"); JsonArray maxVals = doc.createNestedArray("maxValues"); JsonArray thresholds = doc.createNestedArray("thresholds");
  for (int i = 0; i < SENSOR_COUNT; i++) { minVals.add(sensorMin[i]); maxVals.add(sensorMax[i]); thresholds.add(sensorThreshold[i]); }
  String response; serializeJson(doc, response); server.send(200, "application/json", response);
}
void handleGetThresholds() {
  DynamicJsonDocument doc(512); JsonArray arr = doc.createNestedArray("thresholds");
  for (int i = 0; i < SENSOR_COUNT; i++) arr.add(sensorThreshold[i]);
  String response; serializeJson(doc, response); server.send(200, "application/json", response);
}
void handleSetThresholds() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}"); return; }
  DynamicJsonDocument doc(512); DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}"); return; }
  if (doc.containsKey("thresholds")) {
    JsonArray arr = doc["thresholds"];
    for (int i = 0; i < SENSOR_COUNT && i < arr.size(); i++) { int t = arr[i]; if (t >= 0 && t <= 4000) sensorThreshold[i] = t; }
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Thresholds updated\"}"); return;
  }
  server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload\"}");
}
void handleHeatMap() {
  DynamicJsonDocument doc(1024); JsonArray activity = doc.createNestedArray("activity");
  for (int i = 0; i < SENSOR_COUNT; i++) activity.add(sensorActivityCount[i]);
  String response; serializeJson(doc, response); server.send(200, "application/json", response);
  for (int i = 0; i < SENSOR_COUNT; i++) sensorActivityCount[i] = sensorActivityCount[i] / 2;
}

// =============== CONTROL FUNCTIONS ===============
void runPIDControl() {
  qtr.read(sensorValues); uint16_t position = qtr.readLineBlack(sensorValues);
  float error = 3500 - position;
  unsigned long now = millis();
  unsigned long prevTime = (dataIndex > 0) ? dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].timestamp : now;
  float dt = (now - prevTime) / 1000.0f;
  float prevPosition = (dataIndex > 0) ? dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].position : position;
  float speed = (dt > 0) ? (position - prevPosition) / dt : 0;
  float prevSpeed = (dataIndex > 0) ? dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].speed : 0;
  float acceleration = (dt > 0) ? (speed - prevSpeed) / dt : 0;
  integral = constrain(integral + error, -1000, 1000); float derivative = error - lastError;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  int motorSpeedA = constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed);
  int motorSpeedB = constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed);

  setMotorSpeed(motorSpeedA, motorSpeedB); lastError = error;
  motorADirection = (motorSpeedA > 10) ? 1 : (motorSpeedA < -10) ? -1 : 0;
  motorBDirection = (motorSpeedB > 10) ? 1 : (motorSpeedB < -10) ? -1 : 0;
  logControlData(position, error, pidOutput, motorSpeedA, motorSpeedB);

  dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].speed = speed;
  dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].acceleration = acceleration;
  for (int i = 0; i < SENSOR_COUNT; i++) if (sensorValues[i] < sensorThreshold[i]) sensorActivityCount[i]++;
}
void setMotorSpeed(int speedA, int speedB) {
  digitalWrite(AIN1, speedA >= 0); digitalWrite(AIN2, speedA < 0); analogWrite(PWMA, abs(speedA));
  digitalWrite(BIN1, speedB >= 0); digitalWrite(BIN2, speedB < 0); analogWrite(PWMB, abs(speedB));
}
void stopMotors() { analogWrite(PWMA, 0); analogWrite(PWMB, 0); }
void logControlData(float position, float error, float pidOutput, int motorA, int motorB) {
  unsigned long now = millis();
  dataBuffer[dataIndex].timestamp = now;
  dataBuffer[dataIndex].position = position;
  dataBuffer[dataIndex].error = error;
  dataBuffer[dataIndex].pidOutput = pidOutput;
  dataBuffer[dataIndex].motorA = motorA;
  dataBuffer[dataIndex].motorB = motorB;
  for (int i = 0; i < SENSOR_COUNT; i++) dataBuffer[dataIndex].sensors[i] = sensorValues[i];
  dataIndex = (dataIndex + 1) % MAX_DATA_POINTS;
}
void logSensorData() {
  if (!isCalibrated) return;
  qtr.read(sensorValues); uint16_t position = qtr.readLineBlack(sensorValues);
  logControlData(position, 3500 - position, 0, 0, 0);
}

// =============== HTML DASHBOARD ===============
String getMainHTML() {
return R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
  <meta charset="UTF-8" />
  <title>Line Follower Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1" />
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@500;700&display=swap" rel="stylesheet">
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
  <style>
    /* Modern minimal styling similar to PID-6, but with dashboard expansion... */
    :root { --bg: #141518; --fg: #edeef1; --accent: #1fc191; --card: #1a1b21; --border: #22232b; --primary: #2196f3; --err: #eb5757; --space: 1rem; }
    body { background: var(--bg); color: var(--fg); margin: 0; font-family: 'Inter', sans-serif; min-height: 100vh; }
    .container { max-width: 540px; margin: 0 auto; padding: var(--space); }
    .card { background: var(--card); border-radius: 12px; margin-bottom: 1.2em; padding: 1.1em 1.1em 1em; box-shadow: 0 3px 18px #0004; }
    h1 { font-size: 1.22rem; font-weight: 700; margin: 1.1em 0 0.6em; letter-spacing: 0.03em; text-align:center; }
    .status { font-size: 1.1em; padding: .6em 0; text-align: center; border-radius: 8px; margin-bottom: 1em; font-weight: 600; background: #22242a; color: #bedcff; }
    .status.running { color: #9cf5c6; background: #1e3930; }
    .status.stopped { color: #ff8787; background: #332025; }
    .status.manual { color: #ffe38b; background: #282111; }
    button { border: none; border-radius: 7px; padding: 0.65em 0; font-size: 1em; color: #fff; background: var(--primary); font-weight: 600; cursor: pointer; box-shadow: 0 1px 4px #0012; margin-right:.4em; }
    button:active { background: #186dae; }
    .btn-row { display: flex; gap: 0.65em; margin-bottom: .7em; }
    .arrow-grid { display: grid; grid-template-columns: 50px 50px 50px; grid-template-rows: 50px 50px 50px; gap: 6px; justify-content: center; margin: 1.1em 0 0.3em; user-select:none;}
    .arrow-btn { font-size: 1.72em; display: flex; align-items: center; justify-content: center; border: none; background: #1c1e23; color: var(--accent); border-radius: 9px; transition: background 0.1s; cursor: pointer; box-shadow: 0 1px 3px #0002; }
    .arrow-btn:active { background: #273043; }
    .arrow-btn.empty { background: none; box-shadow: none; cursor: default; }
    .manual-row-status { text-align:center; margin-top:.15em; font-size:.98em; color:#8cc1d8; }
    /* Sensors, heatmap, bar chart etc... */
    .sensor-bars { display: grid; grid-template-columns: repeat(8, 1fr); gap: 3px; margin-bottom: .5em;}
    .sensor-bar { height: 22px; border-radius: 7px; background: #222c; color: #cfdfff; text-align: center; font-size: .88em; font-weight: 600; display: flex; align-items: center; justify-content: center; transition: background .3s; }
    .sensor-bar.active { background: #1fc191c6; color: #15332a; }
    .info-row { display: flex; font-size: .91em; color: #7fa9f2; flex-wrap:wrap; }
    .info-row span {flex:1 0 120px; padding-bottom:.2em;}
    #heatmap {display:grid;grid-template-columns:repeat(8,1fr);gap:4px;}
    .heat-cell { height: 18px; border-radius: 6px; text-align: center; font-size: .83em; background: #232834; color: #b9bccc; transition: background .25s; font-weight:500; }
    .motor-vis {text-align:center; font-size:1.2em; margin-bottom:.7em;}
    .motor-arrow {font-size:2em;vertical-align:middle;margin:0 1em;}
    canvas {width:100%!important;max-height:170px!important;background:#131616;border-radius:9px;}
    .calib-row span {font-size:0.93em;display:block;margin-top:3px;}
    .threshold-controls label {margin-right:9px;display:inline-block;}
    .dash-title {font-size:1.05em;color:#6be178;}
    @media (min-width:700px) {.container{max-width:780px;}}
  </style>
</head>
<body>
<div class="container">
  <h1>Line Follower Dashboard Pro</h1>
  <div id="status" class="status stopped">Loading status...</div>

  <div class="card">
    <div class="btn-row">
      <button id="btnCalib">Calib</button>
      <button id="btnStart">Start</button>
      <button id="btnStop">Stop</button>
      <button id="btnAutoTune">Auto Tune</button>
    </div>
    <div style="margin:.7em 0 .11em; font-size:.98em;">Manual control</div>
    <div class="arrow-grid" id="arrowGrid">
      <button class="arrow-btn empty"></button>
      <button id="arrUp" class="arrow-btn">&#9650;</button>
      <button class="arrow-btn empty"></button>
      <button id="arrLeft" class="arrow-btn">&#9664;</button>
      <button class="arrow-btn empty"></button>
      <button id="arrRight" class="arrow-btn">&#9654;</button>
      <button class="arrow-btn empty"></button>
      <button id="arrDown" class="arrow-btn">&#9660;</button>
      <button class="arrow-btn empty"></button>
    </div>
    <div class="manual-row-status" id="motorOut">A: 0 &nbsp;B: 0</div>
    <div style="text-align:center;">
      <label><input type="checkbox" id="manualToggle"/> Manual Mode</label>
    </div>
  </div>

  <div class="card">
    <div class="dash-title">Calibration Data Visualization</div>
    <div class="calib-row"><span id="calMin"></span></div>
    <div class="calib-row"><span id="calMax"></span></div>
    <div class="calib-row"><span id="calThresh"></span></div>
  </div>

  <div class="card">
    <div class="dash-title">Custom Sensor Thresholds</div>
    <div id="thresholdControls"></div>
    <button id="btnSetThresh">Update Thresholds</button>
  </div>

  <div class="card">
    <div class="sensor-bars" id="sensorBars"></div>
    <div class="info-row"><span id="sensorRaw"></span><span id="sensorThresh"></span></div>
    <div style="font-size:.96em;color:#86e3f5;"><span id="position"></span> <span id="error"></span></div>
  </div>

  <div class="card">
    <div class="motor-vis" id="motorVisual">
      Motor A: <span id="motorADir" class="motor-arrow">⬜</span>
      Motor B: <span id="motorBDir" class="motor-arrow">⬜</span>
    </div>
    <div class="dash-title">Sensor Heatmap</div>
    <div id="heatmap"></div>
  </div>

  <div class="card">
    <div class="dash-title">Line Path Replay</div>
    <canvas id="lineReplayCanvas" width="420" height="140"></canvas>
    <div style="text-align:center;margin-top:.5em;"><button id="btnReplayReset">Reset Replay</button></div>
  </div>

  <div class="card">
    <form id="pidForm">
      <label>Kp <input id="kpIn" type="number" value="0.04" step="0.001"></label>
      <label>Ki <input id="kiIn" type="number" value="0.002" step="0.001"></label>
      <label>Kd <input id="kdIn" type="number" value="0.015" step="0.001"></label>
      <label>Base <input id="bSp" type="number" value="100"></label>
      <label>Max <input id="mSp" type="number" value="255"></label>
      <button type="submit">Set</button>
    </form>
  </div>

  <div class="card">
    <div class="dash-title">Performance Graph</div>
    <canvas id="chartPID"></canvas>
  </div>
  <div class="card">
    <div class="dash-title">Speed vs Acceleration Map</div>
    <canvas id="chartSpeed"></canvas>
  </div>
</div>
<script>
// Helper
const $=id=>document.getElementById(id);

// ==== UI FUNCTION HOOKS ====
// System status
function statusRefresh() {
  fetch('/api/status').then(r=>r.json()).then(data=>{
    const s=$('status');
    if(!data.calibrated) {s.textContent="Not calibrated";s.className='status stopped';}
    else if(data.running) {s.textContent="Running";s.className='status running';}
    else if(data.manualControl) {s.textContent="Manual";s.className='status manual';}
    else {s.textContent="Stopped";s.className='status stopped';}
    $('kpIn').value=data.kp.toFixed(3); $('kiIn').value=data.ki.toFixed(3); $('kdIn').value=data.kd.toFixed(3);
    $('bSp').value=data.baseSpeed; $('mSp').value=data.maxSpeed; $('manualToggle').checked=!!data.manualControl;
    $('motorADir').textContent = data.motorADirection > 0 ? '➡️' : (data.motorADirection<0 ? '⬅️':'⬜');
    $('motorBDir').textContent = data.motorBDirection > 0 ? '➡️' : (data.motorBDirection<0 ? '⬅️':'⬜');
  });
}
function sensorsRefresh() {
  fetch('/api/sensors').then(r=>r.json()).then(d=>{
    let bars=$('sensorBars'),rw=[],th=[];bars.innerHTML='';
    d.sensorsRaw.forEach((v,i)=>{
      let el=document.createElement('div');
      el.className='sensor-bar'+(v<d.thresholds[i]?' active':'');el.textContent=v;bars.appendChild(el);
      rw.push(v);th.push(d.thresholds[i]);
    });
    $('sensorRaw').textContent='Raw: '+rw.join(', ');
    $('sensorThresh').textContent=' | Thresh: '+th.join(', ');
    $('position').textContent = d.position?'Pos:'+d.position:''; $('error').textContent = d.error?'Err:'+d.error:'';
  });
}
function heatmapRefresh() {
  fetch('/api/heatmap').then(r=>r.json()).then(d=>{
    let h=$('heatmap');h.innerHTML='';let max=Math.max(...d.activity);
    d.activity.forEach((v,i)=>{
      let e=document.createElement('div');
      e.className='heat-cell';
      e.style.background=`hsl(${(v/max||0)*120},80%,45%)`;
      e.textContent = v>0?i:'';
      h.appendChild(e);
    });
  });
}
// Calibration visualization
function loadCalibrationData() {
  fetch('/api/calibrationdata').then(res=>res.json()).then(data=>{
    $('calMin').textContent='Sensor Min: '+data.minValues.join(', ');
    $('calMax').textContent='Sensor Max: '+data.maxValues.join(', ');
    $('calThresh').textContent='Sensor Thresh: '+data.thresholds.join(', ');
    // Build threshold inputs UI
    const container = $('thresholdControls'); container.innerHTML='';
    for(let i=0;i<data.thresholds.length;i++){
      const input=document.createElement('input'); input.type='number'; input.min=0; input.max=4000; input.value=data.thresholds[i]; input.id='threshold_'+i; input.style.width='60px';
      const label=document.createElement('label');label.textContent=`${i}: `;label.appendChild(input);container.appendChild(label);
    }
  });
}
$('btnSetThresh').onclick=function(){
  const thresholds=[];for(let i=0;i<8;i++) thresholds.push(parseInt($('threshold_'+i).value));
  fetch('/api/thresholds',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({thresholds})}).then(r=>r.json()).then(d=>{alert(d.message);loadCalibrationData();});
};

// Manual
function setManual(a,b){fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motorA:a,motorB:b})});$('motorOut').textContent='A: '+a+'  B: '+b;}
['arrUp','arrDown','arrLeft','arrRight'].forEach(id=>{
  let btn=$(id),mapping={arrUp:[180,180],arrDown:[-180,-180],arrLeft:[-100,100],arrRight:[100,-100]};
  ['touchstart','mousedown'].forEach(evt=>btn.addEventListener(evt,e=>{e.preventDefault();if($('manualToggle').checked)setManual(...mapping[id]);}));
  ['touchend','mouseup','mouseleave'].forEach(evt=>btn.addEventListener(evt,e=>{e.preventDefault();if($('manualToggle').checked)setManual(0,0);}));
});
$('manualToggle').addEventListener('input', e=>{
  fetch('/api/manualmode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({manual:!!e.target.checked})});
});

// Main buttons
$('btnCalib').onclick = ()=>fetch('/api/calibrate',{method:'POST'}).then(r=>r.json()).then(d=>{alert(d.message);loadCalibrationData();});
$('btnStart').onclick = ()=>fetch('/api/start',{method:'POST'}).then(()=>{statusRefresh();});
$('btnStop').onclick = ()=>fetch('/api/stop',{method:'POST'}).then(()=>{statusRefresh();});
$('btnAutoTune').onclick = ()=>fetch('/api/autotune',{method:'POST'}).then(r=>r.json()).then(d=>alert(d.message));

// PID tuning
$('pidForm').onsubmit = e=>{
  e.preventDefault();
  let d={kp:parseFloat($('kpIn').value),ki:parseFloat($('kiIn').value),kd:parseFloat($('kdIn').value),baseSpeed:parseInt($('bSp').value),maxSpeed:parseInt($('mSp').value)};
  fetch('/api/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});
  return false;
};

// CHARTS
let pidChart,speedChart;
window.onload=()=>{
  pidChart=new Chart($('chartPID').getContext('2d'),{type:'line',data:{datasets:[
    {label:'Position',borderColor:'#3bf',data:[],tension:.11,fill:!1},
    {label:'Error',borderColor:'#ea3636',data:[],tension:.09,fill:!1},
    {label:'PID',borderColor:'#14b8a6',data:[],tension:.06,fill:!1}
  ]},options:{responsive:1,animation:!1,scales:{x:{type:'linear',title:{display:0}}}}});
  speedChart=new Chart($('chartSpeed').getContext('2d'),{type:'line',data:{datasets:[
    {label:'Speed',borderColor:'#d9d44c',data:[],tension:.11,fill:!1},
    {label:'Accel',borderColor:'#7add8c',data:[],tension:.08,fill:!1}
  ]},options:{responsive:1,animation:!1,scales:{x:{type:'linear',title:{display:0}}}}});
  setInterval(()=>{
    statusRefresh(); sensorsRefresh(); heatmapRefresh(); dataRefresh(); drawReplay();
  },300);
  loadCalibrationData();
};

// PID/Speed+Accel Data
function dataRefresh() {
  fetch('/api/data').then(r=>r.json()).then(cd=>{
    let ts=cd.data.map(x=>x.timestamp/1000);
    [0,1,2].forEach((i,idx)=>{pidChart.data.datasets[idx].data = cd.data.map((p,ii)=>({x:ts[ii],y:[p.position,p.error,p.pidOutput][idx]}));});
    [0,1].forEach((i,idx)=>{speedChart.data.datasets[idx].data = cd.data.map((p,ii)=>({x:ts[ii],y:[p.speed||0,p.acceleration||0][idx]}));});
    pidChart.update('none');speedChart.update('none');
  });
}
// Line Path Replay
const replayCanvas=$('lineReplayCanvas'),replayCtx=replayCanvas.getContext('2d');
function drawReplay() {
  fetch('/api/data').then(res=>res.json()).then(data=>{
    replayCtx.clearRect(0,0,replayCanvas.width,replayCanvas.height);
    replayCtx.beginPath();replayCtx.strokeStyle='blue';replayCtx.lineWidth=2;
    if(data.data.length<2)return;
    const maxPos=7000,scaleY=replayCanvas.height/maxPos;
    for(let i=0;i<data.data.length;i++){
      let x=(i/data.data.length)*replayCanvas.width;
      let y=replayCanvas.height-(data.data[i].position*scaleY);
      if(i===0){replayCtx.moveTo(x,y);}else{replayCtx.lineTo(x,y);}
    }
    replayCtx.stroke();
    replayCtx.strokeStyle='gray';
    replayCtx.beginPath();
    let midY=replayCanvas.height-(3500*scaleY);
    replayCtx.moveTo(0,midY); replayCtx.lineTo(replayCanvas.width,midY); replayCtx.stroke();
  });
}
$('btnReplayReset').onclick=()=>drawReplay();
</script>
</body></html>
)rawliteral";
}
