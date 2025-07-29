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

// PID Control Variables
float Kp = 0.04;
float Ki = 0.002;
float Kd = 0.015;
float lastError = 0;
float integral = 0;
int baseSpeed = 100;
int maxSpeed = 255;

bool isCalibrated = false;
bool isRunning = false;
bool calibrationMode = false;

// Min/Max sensor values and thresholds tracked manually
uint16_t sensorMin[SENSOR_COUNT];
uint16_t sensorMax[SENSOR_COUNT];
uint16_t sensorThreshold[SENSOR_COUNT];

// Sensor activity count for heatmap
unsigned long sensorActivityCount[SENSOR_COUNT] = {0};

// Data logging structure
struct SensorData {
  unsigned long timestamp = 0;
  uint16_t sensors[SENSOR_COUNT] = {0};
  float position = 0;
  float speed = 0;
  float acceleration = 0;
  float error = 0;
  float pidOutput = 0;
  int motorA = 0;
  int motorB = 0;
};

const int MAX_DATA_POINTS = 100;
SensorData dataBuffer[MAX_DATA_POINTS];
int dataIndex = 0;

// Manual motor control variables
int manualMotorA = 0;
int manualMotorB = 0;
bool manualControlActive = false;

// Timing
unsigned long lastSensorRead = 0;

// =============== FUNCTION DECLARATIONS ===============
void handleRoot();
void handleCalibrate();
void handleStart();
void handleStop();
void handlePIDUpdate();
void handleStatus();
void handleSensors();
void handleData();
void handleManualMotor();
void handleSetManualMode();
void handleCalibrationData();
void handleGetThresholds();
void handleSetThresholds();
void handleHeatMap();

void initializeMotors();
void initializeSensors();
void initializeWiFi();
void setupWebServer();

void runPIDControl();
void setMotorSpeed(int speedA, int speedB);
void stopMotors();

void logControlData(float position, float error, float pidOutput, int motorA, int motorB);
void logSensorData();

String getMainHTML();

// =============== SETUP & LOOP ===============
void setup() {
  Serial.begin(115200);
  initializeMotors();
  initializeSensors();
  initializeWiFi();
  setupWebServer();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4000;  // Large initial min
    sensorMax[i] = 0;     // Small initial max
    sensorThreshold[i] = 2000;  // Initial threshold guess
    sensorActivityCount[i] = 0;
  }
}

void loop() {
  server.handleClient();

  if (isRunning && isCalibrated && !manualControlActive && !calibrationMode) {
    runPIDControl();
  } else if (manualControlActive) {
    setMotorSpeed(manualMotorA, manualMotorB);
  } else {
    stopMotors();
  }

  if (millis() - lastSensorRead > 50) {
    logSensorData();
    lastSensorRead = millis();
  }
}

// =============== INITIALIZATION ===============
void initializeMotors() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  Serial.println("Motors initialized");
}

void initializeSensors() {
  qtr.setTypeRC();
  qtr.setSensorPins(irPins, SENSOR_COUNT);
  Serial.println("QTR sensors initialized");
}

void initializeWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int attempts = 0;
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed! Restarting...");
    ESP.restart();
  }
  Serial.println("WiFi connected! IP: " + WiFi.localIP().toString());
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/pid", HTTP_POST, handlePIDUpdate);
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

void handleRoot() {
  server.send(200, "text/html", getMainHTML());
}

void handleCalibrate() {
  if (calibrationMode) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Calibration already running\"}");
    return;
  }
  calibrationMode = true;
  isCalibrated = false;

  Serial.println("Starting calibration...");

  // Reset min/max arrays
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4000;
    sensorMax[i] = 0;
  }

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    qtr.read(sensorValues);

    for (int s = 0; s < SENSOR_COUNT; s++) {
      if (sensorValues[s] < sensorMin[s]) sensorMin[s] = sensorValues[s];
      if (sensorValues[s] > sensorMax[s]) sensorMax[s] = sensorValues[s];
    }
    if (i % 40 == 0) Serial.print(".");
    delay(5);
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  isCalibrated = true;
  calibrationMode = false;
  Serial.println("\nCalibration complete");

  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Calibration complete\"}");
}

void handleStart() {
  if (!isCalibrated) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Calibrate first\"}");
    return;
  }
  isRunning = true;
  manualControlActive = false;
  integral = 0;
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Started line following\"}");
}

void handleStop() {
  isRunning = false;
  manualControlActive = false;
  stopMotors();
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Stopped\"}");
}

void handlePIDUpdate() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}");
    return;
  }
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }
  if (doc.containsKey("kp")) Kp = doc["kp"];
  if (doc.containsKey("ki")) Ki = doc["ki"];
  if (doc.containsKey("kd")) Kd = doc["kd"];
  if (doc.containsKey("baseSpeed")) baseSpeed = doc["baseSpeed"];
  if (doc.containsKey("maxSpeed")) maxSpeed = doc["maxSpeed"];

  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"PID updated\"}");
}

void handleStatus() {
  DynamicJsonDocument doc(512);
  doc["calibrated"] = isCalibrated;
  doc["running"] = isRunning;
  doc["calibrating"] = calibrationMode;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["baseSpeed"] = baseSpeed;
  doc["maxSpeed"] = maxSpeed;
  doc["manualControl"] = manualControlActive;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSensors() {
  qtr.read(sensorValues);
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (sensorValues[i] < sensorThreshold[i]) sensorActivityCount[i]++;
  DynamicJsonDocument doc(2048);
  JsonArray sensorsRaw = doc.createNestedArray("sensorsRaw");
  JsonArray sensorsThresholds = doc.createNestedArray("thresholds");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorsRaw.add(sensorValues[i]);
    sensorsThresholds.add(sensorThreshold[i]);
  }
  if (isCalibrated) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    doc["position"] = position;
    doc["error"] = 3500 - position;
  }
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleData() {
  DynamicJsonDocument doc(6144);
  JsonArray dataArray = doc.createNestedArray("data");
  int count = min(50, MAX_DATA_POINTS);
  int startIndex = (dataIndex - count + MAX_DATA_POINTS) % MAX_DATA_POINTS;
  for (int i = 0; i < count; i++) {
    int idx = (startIndex + i) % MAX_DATA_POINTS;
    JsonObject point = dataArray.createNestedObject();
    point["timestamp"] = dataBuffer[idx].timestamp;
    point["position"] = dataBuffer[idx].position;
    point["error"] = dataBuffer[idx].error;
    point["pidOutput"] = dataBuffer[idx].pidOutput;
    point["motorA"] = dataBuffer[idx].motorA;
    point["motorB"] = dataBuffer[idx].motorB;
    point["speed"] = dataBuffer[idx].speed;
    point["acceleration"] = dataBuffer[idx].acceleration;
  }
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleManualMotor() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}");
    return;
  }
  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }
  manualMotorA = doc["motorA"];
  manualMotorB = doc["motorB"];
  manualControlActive = true;
  isRunning = false;
  setMotorSpeed(manualMotorA, manualMotorB);

  server.send(200, "application/json", "{\"status\":\"success\"}");
}

void handleSetManualMode() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}");
    return;
  }
  DynamicJsonDocument doc(64);
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }
  manualControlActive = doc["manual"] == true;
  if (!manualControlActive) stopMotors();
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

void handleCalibrationData() {
  DynamicJsonDocument doc(512);
  JsonArray minVals = doc.createNestedArray("minValues");
  JsonArray maxVals = doc.createNestedArray("maxValues");
  JsonArray thresholds = doc.createNestedArray("thresholds");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    minVals.add(sensorMin[i]);
    maxVals.add(sensorMax[i]);
    thresholds.add(sensorThreshold[i]);
  }
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetThresholds() {
  DynamicJsonDocument doc(512);
  JsonArray arr = doc.createNestedArray("thresholds");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    arr.add(sensorThreshold[i]);
  }
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetThresholds() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}");
    return;
  }
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }
  if (doc.containsKey("thresholds")) {
    JsonArray arr = doc["thresholds"];
    for (int i = 0; i < SENSOR_COUNT && i < arr.size(); i++) {
      int t = arr[i];
      if (t >= 0 && t <= 4000) sensorThreshold[i] = t;
    }
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Thresholds updated\"}");
    return;
  }
  server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload\"}");
}

void handleHeatMap() {
  DynamicJsonDocument doc(1024);
  JsonArray activity = doc.createNestedArray("activity");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    activity.add(sensorActivityCount[i]);
  }
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);

  // Decay counts after sending
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorActivityCount[i] = sensorActivityCount[i] / 2;
  }
}

// =============== CONTROL FUNCTIONS ===============
void runPIDControl() {
  qtr.read(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);
  float error = 3500 - position;

  unsigned long now = millis();
  unsigned long prevTime = (dataIndex > 0) ? dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].timestamp : now;
  float dt = (now - prevTime) / 1000.0f;

  float prevPosition = (dataIndex > 0) ? dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].position : position;
  float speed = (dt > 0) ? (position - prevPosition) / dt : 0;

  float prevSpeed = (dataIndex > 0) ? dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].speed : 0;
  float acceleration = (dt > 0) ? (speed - prevSpeed) / dt : 0;

  integral = constrain(integral + error, -1000, 1000);
  float derivative = error - lastError;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;

  int motorSpeedA = constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed);
  int motorSpeedB = constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed);

  setMotorSpeed(motorSpeedA, motorSpeedB);
  lastError = error;

  logControlData(position, error, pidOutput, motorSpeedA, motorSpeedB);
  
  // Store speed and acceleration in last logged data
  dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].speed = speed;
  dataBuffer[(dataIndex - 1 + MAX_DATA_POINTS) % MAX_DATA_POINTS].acceleration = acceleration;

  for (int i = 0; i < SENSOR_COUNT; i++)
    if (sensorValues[i] < sensorThreshold[i]) sensorActivityCount[i]++;
}

void setMotorSpeed(int speedA, int speedB) {
  digitalWrite(AIN1, speedA >= 0);
  digitalWrite(AIN2, speedA < 0);
  analogWrite(PWMA, abs(speedA));

  digitalWrite(BIN1, speedB >= 0);
  digitalWrite(BIN2, speedB < 0);
  analogWrite(PWMB, abs(speedB));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

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
  qtr.read(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);

  logControlData(position, 3500 - position, 0, 0, 0);
}

// =============== HTML DASHBOARD ===============

String getMainHTML() {
  // Full premium dashboard HTML + CSS + JS code, including:
  // - Modern design with Google Fonts + Font Awesome icons
  // - Responsive cards for status, manual control toggle + joystick,
  // - Sensor bars + heatmap,
  // - PID and speed/acceleration charts with Chart.js,
  // - PID tuning inputs,
  // - Calibration data display
  // - Joystick using nipplejs

  // Due to size, this is a compressed string literal.  
  // For complete code and any customization, just ask or use the last provided version from previous answer.

  return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<title>Line Follower Premium Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link href="https://fonts.googleapis.com/css?family=Montserrat:400,700&display=swap" rel="stylesheet">
<link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.2/css/all.min.css" rel="stylesheet"/>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script src="https://cdn.jsdelivr.net/npm/nipplejs@0.9.0/dist/nipplejs.min.js"></script>
<style>
/* Styles from previous detailed premium dashboard */
body { background: linear-gradient(120deg,#3d4151 0,#06182a 100%);	margin:0; font-family:'Montserrat',sans-serif;}
.container { max-width:1000px; margin:40px auto; padding:16px;}
.card { background:rgba(255,255,255,0.97); border-radius:16px; box-shadow:0 4px 44px 0 rgba(44,62,80,0.21); padding:24px; margin-bottom:28px;}
h1 { text-align:center; margin:0 0 40px; color:#004aad; text-shadow:1px 2px 8px rgba(0,42,64,.18);font-size:2.1rem;}
h2 { margin-top:0; color:#1a232a;}
.btn {padding:11px 30px; border:none; border-radius:6px; font-weight:600; box-shadow:0 4px 20px 0 #b4e0ff39; background:#0977ff; color:#fff; margin:6px; cursor:pointer; transition:background .18s;}
.btn:hover {background:#034ea1;}
.status { font-weight:600; margin-bottom:14px; padding:8px 13px; border-radius:3px;}
.status.success { background: #e7faeb; color: #187a32;}
.status.error { background: #ffeaea; color: #a82121;}
.switch {position:relative;display:inline-block;width:48px;height:30px;vertical-align:middle}
.switch input {opacity:0;width:0;height:0}
.switch .slider {position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#aaa;-webkit-transition:.4s;transition:.4s;border-radius:30px}
.switch .slider:before {position:absolute;content:"";height:22px;width:22px;left:4px;bottom:4px;background:white;-webkit-transition:.4s;transition:.4s;border-radius:50%}
.switch input:checked+.slider {background:#0ca56e;}
.switch input:checked+.slider:before {transform:translateX(18px);}
.sensor-bar { height: 32px; background: #def4ff; border-radius: 7px; display: flex; align-items: center; justify-content: center; font-size: 15px; font-weight:bold;}
.sensor-active { background: #e94b21!important; color: #fff; }
.sensor-display { display: grid; grid-template-columns: repeat(8, 1fr); gap: 6px; margin: 5px 0 12px 0;}
#joystick { width:110px; height:110px; margin:11px auto;}
#heatmap { display: flex; gap: 8px;}
.heat-bar { width:35px; height:35px; border-radius:7px;display:flex;justify-content:center;align-items:center; font-weight:bold; color:#222;}
.chart-container { position:relative; height:300px; margin-bottom:20px; background:rgba(255,255,255,0.95); border-radius:10px; box-shadow:0 2px 6px #c8e1ff38;}
input[type="number"], input[type="range"] { border:1px solid #adeaff; border-radius:3px; width:70px; padding:5px;}
label span {font-size:14px;color:#555;}
button i {margin-right:8px;}
@media(max-width:700px){.container{padding:2px;}
  .card{padding:10px;}
}
</style>
</head>
<body>
<div class="container">
<h1>Line Follower Premium Dashboard</h1>

<div class="card">
  <h2>Status & Controls</h2>
  <div id="status" class="status">Loading...</div>
  <button class="btn" onclick="calibrate()"><i class="fa fa-cubes"></i>Calibrate</button>
  <button class="btn" onclick="start()"><i class="fa fa-play"></i>Start</button>
  <button class="btn" onclick="stop()"><i class="fa fa-stop"></i>Stop</button>
</div>

<div class="card">
  <h2>Manual Motor Control</h2>
  <label class="switch">
    <input type="checkbox" id="manualModeToggle" onchange="toggleManualMode()"><span class="slider"></span>
  </label>
  <span style="font-weight:500; margin-left:14px;">Enable manual mode</span>
  <div id="manualJoystickPanel" style="display:none; text-align:center;">
    <div style="margin:8px;"><strong>Joystick:</strong></div>
    <div id="joystick"></div>
    <div style="font-size:15px;">
      <b>Motor A:</b> <span id="motorAout">0</span> &nbsp;
      <b>Motor B:</b> <span id="motorBout">0</span>
    </div>
  </div>
</div>

<div class="card">
  <h2>Sensors & Thresholds</h2>
  <div id="sensorDisplay" class="sensor-display"></div>
  <div style="margin-bottom:5px;">
    <b>Raw:</b> <span id="sensorRaw"></span>
  </div>
  <div>
    <b>Thresholds:</b> <span id="sensorThreshDisplay"></span>
  </div>
</div>

<div class="card">
  <h2>Sensor Activity Heatmap</h2>
  <div id="heatmap"></div>
</div>

<div class="card">
  <h2>Speed &amp; Acceleration</h2>
  <div class="chart-container">
    <canvas id="speedAccChart"></canvas>
  </div>
</div>

<div class="card">
  <h2>PID/Movement Data</h2>
  <div class="chart-container">
    <canvas id="dataChart"></canvas>
  </div>
  <div>
    <b>Position:</b> <span id="position">-</span> | <b>Error:</b> <span id="error">-</span>
  </div>
</div>

<div class="card">
  <h2>Tune PID</h2>
  <label>Kp:<input type="number" id="kp" step="0.001" value="0.04"></label>
  <label>Ki:<input type="number" id="ki" step="0.001" value="0.002"></label>
  <label>Kd:<input type="number" id="kd" step="0.001" value="0.015"></label>
  <label>Base:<input type="number" id="baseSpeed" value="100"></label>
  <label>Max Speed:<input type="number" id="maxSpeed" value="255"></label>
  <button class="btn" onclick="updatePID()">Update PID</button>
</div>

<div class="card">
  <h2>Calibration Data</h2>
  <div><b>Min:</b> <span id="calMin"></span></div>
  <div><b>Max:</b> <span id="calMax"></span></div>
  <div><b>Thresholds:</b> <span id="calThresh"></span></div>
</div>

</div>
<script>
let chart, chartSA;
function $(id) {return document.getElementById(id);}
function updateStatus() {
  fetch('/api/status').then(res=>res.json()).then(data=>{
    let t="Calibrated: "+(data.calibrated?"✅":"❎")+
    " | Running: "+(data.running?"▶️":"⏹️")+
    (data.manualControl?" | Manual: 🕹️":"");
    $("status").textContent=t;
    $("kp").value=data.kp; $("ki").value=data.ki; $("kd").value=data.kd;
    $("baseSpeed").value=data.baseSpeed; $("maxSpeed").value=data.maxSpeed;
    $("manualModeToggle").checked=data.manualControl;
    $("manualJoystickPanel").style.display = data.manualControl ? "block" : "none";
    if (!data.manualControl) {
      $("motorAout").textContent="0";$("motorBout").textContent="0";
    }
  });
}
function updateSensors() {
  fetch('/api/sensors').then(res=>res.json()).then(data=>{
    $("sensorDisplay").innerHTML="";
    for(let i=0;i<data.sensorsRaw.length;i++){
      let v=data.sensorsRaw[i],t=data.thresholds[i];
      let bar=document.createElement("div");
      bar.className="sensor-bar"+(v<t?" sensor-active":"");
      bar.textContent=v; $("sensorDisplay").appendChild(bar);
    }
    $("sensorRaw").textContent=data.sensorsRaw.join(", ");
    $("sensorThreshDisplay").textContent=data.thresholds.join(", ");
    if(data.position!=null)$("position").textContent=data.position;
    if(data.error!=null)$("error").textContent=data.error;
  });
}
function updateChart() {
  fetch('/api/data').then(res=>res.json()).then(data=>{
    let ts=data.data.map(d=>d.timestamp/1000);
    chart.data.datasets[0].data = data.data.map((d,i)=>({x:ts[i],y:d.position}));
    chart.data.datasets[1].data = data.data.map((d,i)=>({x:ts[i],y:d.error}));
    chart.data.datasets[2].data = data.data.map((d,i)=>({x:ts[i],y:d.pidOutput}));
    chart.update('none');

    chartSA.data.datasets[0].data = data.data.map((d,i)=>({x:ts[i],y:d.speed||0}));
    chartSA.data.datasets[1].data = data.data.map((d,i)=>({x:ts[i],y:d.acceleration||0}));
    chartSA.update('none');
  });
}
function updateHeatMap() {
  fetch('/api/heatmap').then(r=>r.json()).then(data=>{
    let c=$("heatmap");
    c.innerHTML="";
    let max=Math.max(...data.activity);
    data.activity.forEach((val,idx)=>{
      let perc=max?val/max:0;
      let hue=perc*120; // green to red
      let div=document.createElement('div');
      div.className='heat-bar';div.style.background=`hsl(${hue},80%,60%)`;
      div.textContent=(val==0?"":idx);
      c.appendChild(div);
    });
  });
}
window.onload=()=>{
  let ctx=$("dataChart").getContext('2d');
  chart=new Chart(ctx, {type:'line',data:{
    datasets:[
      {label:"Position",borderColor:'#0056d6',data:[],tension:0.1},
      {label:"Error",borderColor:'#de2e36',data:[],tension:0.1},
      {label:"PID Output",borderColor:'#00ca94',data:[],tension:0.08}
    ]
  }, options:{responsive:true,maintainAspectRatio:false,scales:{x:{type:'linear',title:{display:true,text:'Time (s)'}}}}});
  let ctxSA=$("speedAccChart").getContext('2d');
  chartSA=new Chart(ctxSA, {type:'line',data:{
    datasets:[
      {label:"Speed",borderColor:'#fa912d',data:[],tension:0.14},
      {label:"Acceleration",borderColor:'#1e0077',data:[],tension:0.12}
    ]
  },options:{responsive:true,maintainAspectRatio:false,scales:{x:{type:'linear',title:{display:true,text:'Time (s)'}}}}});
  setInterval(()=>{
    updateStatus(); updateSensors(); updateChart(); updateHeatMap(); loadCalibrationData();
  },260);
};

let joystick=null;
function toggleManualMode(){
  let enabled=$("manualModeToggle").checked;
  fetch('/api/manualmode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({manual:enabled})});
  $("manualJoystickPanel").style.display=enabled?"block":"none";
  if(enabled){
    if(!joystick){
      joystick=nipplejs.create({zone:$("joystick"),mode:"static",position:{left:"50%",top:"50%"},color:'blue'});
      joystick.on('move',(evt,data)=>{
        if(data&&data.angle){
          let deg=data.angle.degree,dist=data.distance||0;
          let fwd=Math.cos((deg-90)*Math.PI/180)*dist;
          let turn=Math.cos(deg*Math.PI/180)*dist;
          let mA=Math.round(constrain(fwd+turn,-255,255));
          let mB=Math.round(constrain(fwd-turn,-255,255));
          $("motorAout").textContent=mA; $("motorBout").textContent=mB;
          fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motorA:mA,motorB:mB})});
        }
      });
      joystick.on('end',()=>{ $("motorAout").textContent="0";$("motorBout").textContent="0";
        fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motorA:0,motorB:0})});
      });
    }
  }else if(joystick){joystick.destroy(); joystick=null;}
}
function constrain(v,a,b){return Math.max(a,Math.min(b,v));}
function calibrate(){ fetch('/api/calibrate',{method:'POST'}).then(r=>r.json()).then(d=>alert(d.message)); }
function start(){ fetch('/api/start',{method:'POST'}); }
function stop(){ fetch('/api/stop',{method:'POST'}); }
function updatePID(){
  let data={kp:parseFloat($("kp").value),ki:parseFloat($("ki").value),
    kd:parseFloat($("kd").value),baseSpeed:parseInt($("baseSpeed").value),maxSpeed:parseInt($("maxSpeed").value)};
  fetch('/api/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
}
function loadCalibrationData(){
  fetch('/api/calibrationdata').then(r=>r.json()).then(data=>{
    $("calMin").textContent=data.minValues.join(", ");
    $("calMax").textContent=data.maxValues.join(", ");
    $("calThresh").textContent=data.thresholds.join(", ");
  });
}
</script>
</body>
</html>
)rawliteral";
}

