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

// State variables
bool isCalibrated = false;
bool isRunning = false;
bool calibrationMode = false;

// Tracking min/max and thresholds manually
uint16_t sensorMin[SENSOR_COUNT];
uint16_t sensorMax[SENSOR_COUNT];
uint16_t sensorThreshold[SENSOR_COUNT];

// Sensor activity count for heatmap
unsigned long sensorActivityCount[SENSOR_COUNT] = {0};

// Data logging buffer
struct SensorData {
  unsigned long timestamp;
  uint16_t sensors[SENSOR_COUNT];
  float position;
  float error;
  float pidOutput;
  int motorA;
  int motorB;
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
void handleCalibrationData();
void handleAutoTune();
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
  Serial.println("\nESP32 Line Follower Starting...");

  initializeMotors();
  initializeSensors();
  initializeWiFi();
  setupWebServer();

  // Initialize min/max and thresholds with default values
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4000; // large initial min to be decreased
    sensorMax[i] = 0;    // small initial max to be increased
    sensorThreshold[i] = 2000; // initial threshold guess
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
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("WiFi connection failed!");
    ESP.restart();
  }
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

  // Reset min/max before calibration
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4000; // large number so real data will update it
    sensorMax[i] = 0;
  }

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    qtr.read(sensorValues); // read raw sensor values

    // Update min and max arrays manually
    for (int s = 0; s < SENSOR_COUNT; s++) {
      if (sensorValues[s] < sensorMin[s]) sensorMin[s] = sensorValues[s];
      if (sensorValues[s] > sensorMax[s]) sensorMax[s] = sensorValues[s];
    }

    if (i % 40 == 0) Serial.print(".");
    delay(5);
  }

  // Set threshold mid-way between min and max
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  isCalibrated = true;
  calibrationMode = false;

  Serial.println("\nCalibration complete");

  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Calibration complete\"}");
}

void handleStart() {
  if (isCalibrated) {
    isRunning = true;
    manualControlActive = false;  // disable manual control on start
    integral = 0;
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Line following started\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Please calibrate first\"}");
  }
}

void handleStop() {
  isRunning = false;
  manualControlActive = false;
  stopMotors();
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Stopped\"}");
}

void handlePIDUpdate() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));

    if (doc.containsKey("kp")) Kp = doc["kp"];
    if (doc.containsKey("ki")) Ki = doc["ki"];
    if (doc.containsKey("kd")) Kd = doc["kd"];
    if (doc.containsKey("baseSpeed")) baseSpeed = doc["baseSpeed"];
    if (doc.containsKey("maxSpeed")) maxSpeed = doc["maxSpeed"];

    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"PID updated\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid request\"}");
  }
}

void handleStatus() {
  DynamicJsonDocument doc(1024);
  doc["calibrated"] = isCalibrated;
  doc["running"] = isRunning;
  doc["calibrating"] = calibrationMode;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["baseSpeed"] = baseSpeed;
  doc["maxSpeed"] = maxSpeed;
  doc["manualControl"] = manualControlActive;
  doc["manualMotorA"] = manualMotorA;
  doc["manualMotorB"] = manualMotorB;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSensors() {
  qtr.read(sensorValues);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Update activity count
    if (sensorValues[i] < sensorThreshold[i]) {
      sensorActivityCount[i]++;
    }
  }

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
  DynamicJsonDocument doc(4096);
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
  }

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleManualMotor() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));

    manualMotorA = doc["motorA"];
    manualMotorB = doc["motorB"];
    manualControlActive = true;
    isRunning = false;
    setMotorSpeed(manualMotorA, manualMotorB);

    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\"}");
  }
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
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));

    if (doc.containsKey("thresholds")) {
      JsonArray arr = doc["thresholds"];
      for (int i = 0; i < SENSOR_COUNT && i < arr.size(); i++) {
        int t = arr[i];
        if (t >= 0 && t <= 4000) sensorThreshold[i] = t;
      }
      server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Thresholds updated\"}");
      return;
    }
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

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorActivityCount[i] = sensorActivityCount[i] / 2; // decay old activity
  }
}

// =============== CONTROL FUNCTIONS ===============

void runPIDControl() {
  qtr.read(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);

  float error = 3500 - position;

  integral = constrain(integral + error, -1000, 1000);
  float derivative = error - lastError;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;

  int motorSpeedA = constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed);
  int motorSpeedB = constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed);

  setMotorSpeed(motorSpeedA, motorSpeedB);
  lastError = error;

  logControlData(position, error, pidOutput, motorSpeedA, motorSpeedB);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] < sensorThreshold[i]) {
      sensorActivityCount[i]++;
    }
  }
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
  dataBuffer[dataIndex].timestamp = millis();
  dataBuffer[dataIndex].position = position;
  dataBuffer[dataIndex].error = error;
  dataBuffer[dataIndex].pidOutput = pidOutput;
  dataBuffer[dataIndex].motorA = motorA;
  dataBuffer[dataIndex].motorB = motorB;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    dataBuffer[dataIndex].sensors[i] = sensorValues[i];
  }
  dataIndex = (dataIndex + 1) % MAX_DATA_POINTS;
}

void logSensorData() {
  if (!isCalibrated) return;
  qtr.read(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);
  logControlData(position, 3500 - position, 0, 0, 0);
}


// =============== HTML + JAVASCRIPT INTERFACE ===============
String getMainHTML() {


// Due to length, only core added UI components shown, embedded in LED line follower dashboard template:
return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Line Follower Dashboard Enhanced</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>
    <style>
      body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
      .container { max-width: 1200px; margin: 0 auto; }
      .card { background: white; padding: 20px; margin: 10px 0; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
      .controls { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
      .sensor-display { display: grid; grid-template-columns: repeat(8, 1fr); gap: 5px; margin: 10px 0; }
      .sensor-bar { height: 30px; background: #ddd; border-radius: 5px; display: flex; align-items: center; justify-content: center; font-size: 12px; }
      .sensor-active { background: #ff4444; color: white; }
      button { padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; margin: 5px; }
      .btn-primary { background: #007bff; color: white; }
      .btn-success { background: #28a745; color: white; }
      .btn-danger { background: #dc3545; color: white; }
      .btn-warning { background: #ffc107; color: black; }
      input[type="number"], input[type="range"] { width: 80px; padding: 5px; border: 1px solid #ddd; border-radius: 3px; }
      .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
      .status.success { background: #d4edda; color: #155724; }
      .status.error { background: #f8d7da; color: #721c24; }
      .chart-container { height: 400px; margin: 20px 0; }
      .motor-visualization { display: flex; align-items: center; gap: 20px; font-weight: bold; }
      .motor-arrow { font-size: 2em; }
      #heatmap { display: grid; grid-template-columns: repeat(8, 1fr); gap: 5px; }
      .heat-bar { height: 30px; background: #eee; border-radius: 5px; text-align: center; line-height: 30px; font-size: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Line Follower Dashboard Enhanced</h1>

        <div class="card">
            <h2>System Status</h2>
            <div id="status" class="status">Loading...</div>
            <div class="controls">
                <button class="btn-warning" onclick="calibrate()">Calibrate</button>
                <button class="btn-success" onclick="start()">Start</button>
                <button class="btn-danger" onclick="stop()">Stop</button>
                <button class="btn-primary" onclick="autoTune()">Auto-Tune PID</button>
            </div>
        </div>

        <div class="card">
            <h2>Manual Motor Control</h2>
            <label>Motor A: <input type="range" id="motorA" min="-255" max="255" value="0" oninput="manualMotorChanged()"><span id="motorAVal">0</span></label><br>
            <label>Motor B: <input type="range" id="motorB" min="-255" max="255" value="0" oninput="manualMotorChanged()"><span id="motorBVal">0</span></label><br>
            <button onclick="stopMotors()">Stop Motors</button>
        </div>

        <div class="card">
            <h2>Calibration Data Visualization</h2>
            <div><strong>Sensor Min </strong><span id="calMin"></span></div>
            <div><strong>Sensor Max </strong><span id="calMax"></span></div>
            <div><strong>Sensor Thresholds </strong><span id="calThresh"></span></div>
        </div>

        <div class="card">
            <h2>Custom Sensor Thresholds</h2>
            <div id="thresholdControls"></div>
            <button onclick="updateThresholds()">Update Thresholds</button>
        </div>

        <div class="card">
          <h2>Sensor Data</h2>
          <div id="sensorDisplay" class="sensor-display"></div>
          <div><strong>Raw:</strong> <span id="sensorRaw"></span></div>
          <div><strong>Calibrated:</strong> <span id="sensorCal"></span></div>
          <div><strong>Thresholds:</strong> <span id="sensorThreshDisplay"></span></div>
          <div>Position: <span id="position">-</span></div>
          <div>Error: <span id="error">-</span></div>
        </div>

        <div class="card">
            <h2>Motor Direction Visualization</h2>
            <div class="motor-visualization">
              Motor A: <span id="motorADir" class="motor-arrow">⬜</span>
              Motor B: <span id="motorBDir" class="motor-arrow">⬜</span>
            </div>
        </div>

        <div class="card">
          <h2>Heat Map (Sensor Activity)</h2>
          <div id="heatmap"></div>
        </div>

        <div class="card">
          <h2>Line Path Replay</h2>
          <canvas id="lineReplayCanvas" width="800" height="200" style="border:1px solid #ccc;"></canvas>
          <button onclick="resetReplay()">Reset Replay</button>
        </div>

        <div class="card">
            <h2>PID Control Parameters</h2>
            <label>Kp: <input type="number" id="kp" step="0.001" value="0.04"></label>
            <label>Ki: <input type="number" id="ki" step="0.001" value="0.002"></label>
            <label>Kd: <input type="number" id="kd" step="0.001" value="0.015"></label><br>
            <label>Base Speed: <input type="number" id="baseSpeed" value="100"></label>
            <label>Max Speed: <input type="number" id="maxSpeed" value="255"></label><br>
            <button class="btn-primary" onclick="updatePID()">Update PID</button>
        </div>

        <div class="card">
            <h2>Performance Graph</h2>
            <div class="chart-container">
                <canvas id="dataChart"></canvas>
            </div>
        </div>
    </div>

<script>
let chart;
const ctx = document.getElementById('dataChart').getContext('2d');
chart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: 'Position',
            borderColor: 'rgb(75, 192, 192)',
            tension: 0.1,
            data: []
        }, {
            label: 'Error',
            borderColor: 'rgb(255, 99, 132)',
            tension: 0.1,
            data: []
        }, {
            label: 'PID Output',
            borderColor: 'rgb(54, 162, 235)',
            tension: 0.1,
            data: []
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            x: {
                type: 'linear',
                position: 'bottom'
            }
        }
    }
});

function updateStatus() {
    fetch('/api/status')
        .then(res => res.json())
        .then(data => {
            const statusDiv = document.getElementById('status');
            statusDiv.textContent = "Calibrated: " + (data.calibrated ? "Yes" : "No") +
                                    " | Running: " + (data.running ? "Yes" : "No") +
                                    " | Manual Control: " + (data.manualControl ? "Yes" : "No");
            statusDiv.className = 'status ' + (data.calibrated ? 'success' : 'error');
            document.getElementById('kp').value = data.kp;
            document.getElementById('ki').value = data.ki;
            document.getElementById('kd').value = data.kd;
            document.getElementById('baseSpeed').value = data.baseSpeed;
            document.getElementById('maxSpeed').value = data.maxSpeed;

            // Motor direction display
            document.getElementById('motorADir').textContent = data.motorADirection > 0 ? '➡️' : (data.motorADirection<0 ? '⬅️' : '⬜');
            document.getElementById('motorBDir').textContent = data.motorBDirection > 0 ? '➡️' : (data.motorBDirection<0 ? '⬅️' : '⬜');
        });
}

function updateSensors() {
    fetch('/api/sensors')
        .then(res => res.json())
        .then(data => {
            const display = document.getElementById('sensorDisplay');
            display.innerHTML = "";
            for (let i=0; i<data.sensorsRaw.length; i++) {
                let valRaw = data.sensorsRaw[i];
                let valCal = data.sensorsCalibrated[i];
                let thresh = data.thresholds[i];
                const bar = document.createElement('div');
                bar.className = 'sensor-bar' + (valRaw < thresh ? ' sensor-active' : '');
                bar.textContent = valRaw;
                display.appendChild(bar);
            }
            document.getElementById('sensorRaw').textContent = data.sensorsRaw.join(', ');
            document.getElementById('sensorCal').textContent = data.sensorsCalibrated.map(v => v.toFixed(0)).join(', ');
            document.getElementById('sensorThreshDisplay').textContent = data.thresholds.join(', ');
            if (data.position !== undefined) {
                document.getElementById('position').textContent = data.position;
                document.getElementById('error').textContent = data.error;
            }
        });
}

function updateChart() {
    fetch('/api/data')
        .then(res => res.json())
        .then(data => {
            chart.data.datasets[0].data = data.data.map(d => ({x: d.timestamp / 1000, y: d.position}));
            chart.data.datasets[1].data = data.data.map(d => ({x: d.timestamp / 1000, y: d.error}));
            chart.data.datasets[2].data = data.data.map(d => ({x: d.timestamp / 1000, y: d.pidOutput}));
            chart.update('none');
        });
}

//
// Manual Motor Control
//
function manualMotorChanged() {
    const motorA = document.getElementById('motorA').value;
    const motorB = document.getElementById('motorB').value;
    document.getElementById('motorAVal').innerText = motorA;
    document.getElementById('motorBVal').innerText = motorB;
    fetch('/api/manual', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({motorA: Number(motorA), motorB: Number(motorB)})
    });
}

function stopMotors() {
    document.getElementById('motorA').value = 0;
    document.getElementById('motorB').value = 0;
    manualMotorChanged();
}

//
// Calibration Data Display
//
function loadCalibrationData() {
  fetch('/api/calibrationdata')
    .then(res => res.json())
    .then(data => {
      document.getElementById('calMin').textContent = data.minValues.join(', ');
      document.getElementById('calMax').textContent = data.maxValues.join(', ');
      document.getElementById('calThresh').textContent = data.thresholds.join(', ');
      // Also build threshold inputs UI
      const container = document.getElementById('thresholdControls');
      container.innerHTML = '';
      for(let i=0; i<data.thresholds.length; i++) {
        const input = document.createElement('input');
        input.type = 'number';
        input.min = 0;
        input.max = 4000;
        input.value = data.thresholds[i];
        input.id = 'threshold_'+i;
        input.style.width = '70px';
        const label = document.createElement('label');
        label.textContent = ` Sensor ${i}: `;
        label.appendChild(input);
        container.appendChild(label);
      }
    });
}

// Update thresholds based on UI inputs
function updateThresholds() {
  const thresholds = [];
  for(let i=0; i<8; i++) {
    let val = parseInt(document.getElementById('threshold_'+i).value);
    thresholds.push(val);
  }
  fetch('/api/thresholds', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({thresholds})
  }).then(r => r.json())
    .then(d => { alert(d.message); loadCalibrationData(); });
}

//
// Auto-tune PID
//
function autoTune() {
  fetch('/api/autotune', {method:'POST'})
    .then(res => res.json())
    .then(data => alert(data.message));
}

//
// PID Updates
//
function updatePID() {
  const data = {
    kp: parseFloat(document.getElementById('kp').value),
    ki: parseFloat(document.getElementById('ki').value),
    kd: parseFloat(document.getElementById('kd').value),
    baseSpeed: parseInt(document.getElementById('baseSpeed').value),
    maxSpeed: parseInt(document.getElementById('maxSpeed').value)
  };
  fetch('/api/pid', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(data)
  })
  .then(r => r.json())
  .then(d => alert(d.message));
}

//
// Motor Direction Visualization and Heatmap
//
function updateHeatMap() {
  fetch('/api/heatmap')
    .then(res => res.json())
    .then(data => {
      const container = document.getElementById('heatmap');
      container.innerHTML = '';
      const maxCount = Math.max(...data.activity);
      for(let i=0; i<data.activity.length; i++) {
        const val = data.activity[i];
        const perc = maxCount > 0 ? (val / maxCount) : 0;
        const colorVal = Math.floor(255 - perc * 255);
        const div = document.createElement('div');
        div.className = 'heat-bar';
        div.style.backgroundColor = `rgb(255, ${colorVal}, ${colorVal})`; // Red shade
        div.textContent = val;
        container.appendChild(div);
      }
    });
}

//
// Line Path Replay Visualization
//
const replayCanvas = document.getElementById('lineReplayCanvas');
const replayCtx = replayCanvas.getContext('2d');
function drawReplay() {
  fetch('/api/data')
    .then(res => res.json())
    .then(data => {
      replayCtx.clearRect(0, 0, replayCanvas.width, replayCanvas.height);
      replayCtx.beginPath();
      replayCtx.strokeStyle = 'blue';
      replayCtx.lineWidth = 2;

      if(data.data.length < 2) return;

      // Normalize position to canvas height 0-200 for 0-7000 (3500*2) scale
      const maxPos = 7000;
      const scaleY = replayCanvas.height / maxPos;
      const maxTime = data.data[data.data.length-1].timestamp;

      for (let i = 0; i < data.data.length; i++) {
        let x = (i / data.data.length) * replayCanvas.width;
        let y = replayCanvas.height - (data.data[i].position * scaleY);
        if (i === 0) {
          replayCtx.moveTo(x, y);
        } else {
          replayCtx.lineTo(x, y);
        }
      }
      replayCtx.stroke();

      // Draw line in middle to represent perfect line at 3500
      replayCtx.strokeStyle = 'gray';
      replayCtx.beginPath();
      let midY = replayCanvas.height - (3500 * scaleY);
      replayCtx.moveTo(0, midY);
      replayCtx.lineTo(replayCanvas.width, midY);
      replayCtx.stroke();
    });
}

function resetReplay() {
  // Simply reload chart replay
  drawReplay();
}

// Main loop for UI updates
setInterval(() => {
    updateStatus();
    updateSensors();
    updateChart();
    updateHeatMap();
    drawReplay();
}, 200);

// Load calibration data initially
loadCalibrationData();

function calibrate() {
  fetch('/api/calibrate', {method:'POST'})
    .then(res => res.json())
    .then(data => {
      alert(data.message);
      loadCalibrationData();
    });
}

function start() {
  fetch('/api/start', {method:'POST'})
    .then(res => res.json())
    .then(data => alert(data.message));
}

function stop() {
  fetch('/api/stop', {method:'POST'})
    .then(res => res.json())
    .then(data => alert(data.message));
}
</script>
</body>
</html>
)rawliteral";
}
