/*
 * ESP32 Line Follower Robot
 * Author: sathwikshetty0
 * Created: 2025-07-29 05:58:39 UTC
 * 
 * Features:
 * - Real-time PID control
 * - Web dashboard with live graphs
 * - Sensor calibration
 * - Data logging
 * - Proper pin assignments for ESP32
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>

// =============== CONFIGURATION ===============
// Motor Driver TB6612FNG Pins - Using correct output-capable pins
#define PWMA 25  // Changed from 34 (input-only) to 25
#define AIN2 14  // Changed from 35 (input-only) to 14
#define AIN1 32
#define STBY 33
#define BIN1 15
#define BIN2 26
#define PWMB 27

// QTR-8RC Sensor Configuration
const uint8_t irPins[] = {2, 4, 5, 18, 19, 21, 22, 23};
#define SENSOR_COUNT 8

// WiFi Settings
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

// System State
bool isCalibrated = false;
bool isRunning = false;
bool calibrationMode = false;
unsigned long lastSensorRead = 0;
unsigned long lastPIDUpdate = 0;

// Data Logging Structure
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

// =============== SETUP & MAIN LOOP ===============
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 Line Follower Starting...");
  
  initializeMotors();
  initializeSensors();
  initializeWiFi();
  setupWebServer();
}

void loop() {
  server.handleClient();
  
  if (isRunning && isCalibrated) {
    runPIDControl();
  } else if (!isRunning) {
    stopMotors();
  }
  
  if (millis() - lastSensorRead >= 50) {
    logSensorData();
    lastSensorRead = millis();
  }
}

// =============== INITIALIZATION FUNCTIONS ===============
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
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed!");
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
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/sensors", HTTP_GET, handleSensors);
  
  server.begin();
  Serial.println("Web server started");
}

// =============== WEB HANDLERS ===============
void handleRoot() {
  server.send(200, "text/html", getMainHTML());
}

void handleCalibrate() {
  calibrationMode = true;
  isCalibrated = false;
  
  Serial.println("Starting calibration...");
  
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    if (i % 40 == 0) Serial.print(".");
    delay(5);  // Short delay to prevent watchdog reset
  }
  
  isCalibrated = true;
  calibrationMode = false;
  Serial.println("\nCalibration complete");
  
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Calibration complete\"}");
}

void handleStart() {
  if (isCalibrated) {
    isRunning = true;
    integral = 0;
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Line following started\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Please calibrate first\"}");
  }
}

void handleStop() {
  isRunning = false;
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
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSensors() {
  qtr.read(sensorValues);
  
  DynamicJsonDocument doc(1024);
  JsonArray sensors = doc.createNestedArray("sensors");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors.add(sensorValues[i]);
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

// =============== CONTROL FUNCTIONS ===============
void runPIDControl() {
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
}

void setMotorSpeed(int speedA, int speedB) {
  // Motor A
  digitalWrite(AIN1, speedA >= 0);
  digitalWrite(AIN2, speedA < 0);
  analogWrite(PWMA, abs(speedA));
  
  // Motor B
  digitalWrite(BIN1, speedB >= 0);
  digitalWrite(BIN2, speedB < 0);
  analogWrite(PWMB, abs(speedB));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// =============== DATA LOGGING ===============
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

// =============== HTML INTERFACE ===============
String getMainHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Line Follower Dashboard</title>
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
        input[type="number"] { width: 80px; padding: 5px; border: 1px solid #ddd; border-radius: 3px; }
        .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
        .status.success { background: #d4edda; color: #155724; }
        .status.error { background: #f8d7da; color: #721c24; }
        .chart-container { height: 400px; margin: 20px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Line Follower Dashboard</h1>
        
        <div class="card">
            <h2>System Status</h2>
            <div id="status" class="status">Loading...</div>
            <div class="controls">
                <button class="btn-warning" onclick="calibrate()">Calibrate</button>
                <button class="btn-success" onclick="start()">Start</button>
                <button class="btn-danger" onclick="stop()">Stop</button>
            </div>
        </div>

        <div class="card">
            <h2>Sensor Data</h2>
            <div id="sensorDisplay" class="sensor-display"></div>
            <div>Position: <span id="position">-</span></div>
            <div>Error: <span id="error">-</span></div>
        </div>

        <div class="card">
            <h2>PID Control</h2>
            <div class="controls">
                <div>
                    <label>Kp: <input type="number" id="kp" step="0.001" value="0.04"></label>
                    <label>Ki: <input type="number" id="ki" step="0.001" value="0.002"></label>
                    <label>Kd: <input type="number" id="kd" step="0.001" value="0.015"></label>
                </div>
                <div>
                    <label>Base Speed: <input type="number" id="baseSpeed" value="100"></label>
                    <label>Max Speed: <input type="number" id="maxSpeed" value="255"></label>
                    <button class="btn-primary" onclick="updatePID()">Update PID</button>
                </div>
            </div>
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
        
        // Initialize chart
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
                .then(response => response.json())
                .then(data => {
                    const statusDiv = document.getElementById('status');
                    statusDiv.textContent = "Calibrated: " + (data.calibrated ? "Yes" : "No") + 
                                         " | Running: " + (data.running ? "Yes" : "No");
                    statusDiv.className = 'status ' + (data.calibrated ? 'success' : 'error');
                    
                    document.getElementById('kp').value = data.kp;
                    document.getElementById('ki').value = data.ki;
                    document.getElementById('kd').value = data.kd;
                    document.getElementById('baseSpeed').value = data.baseSpeed;
                    document.getElementById('maxSpeed').value = data.maxSpeed;
                });
        }

        function updateSensors() {
            fetch('/api/sensors')
                .then(response => response.json())
                .then(data => {
                    const sensorDisplay = document.getElementById('sensorDisplay');
                    sensorDisplay.innerHTML = "";
                    
                    data.sensors.forEach((value, index) => {
                        const bar = document.createElement('div');
                        bar.className = 'sensor-bar' + (value < 500 ? ' sensor-active' : '');
                        bar.textContent = value;
                        sensorDisplay.appendChild(bar);
                    });
                    
                    if (data.position !== undefined) {
                        document.getElementById('position').textContent = data.position;
                        document.getElementById('error').textContent = data.error;
                    }
                });
        }

        function updateChart() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    chart.data.datasets[0].data = data.data.map(d => ({x: d.timestamp, y: d.position}));
                    chart.data.datasets[1].data = data.data.map(d => ({x: d.timestamp, y: d.error}));
                    chart.data.datasets[2].data = data.data.map(d => ({x: d.timestamp, y: d.pidOutput}));
                    chart.update('none');
                });
        }

        function calibrate() {
            fetch('/api/calibrate', {method: 'POST'})
                .then(response => response.json())
                .then(data => alert(data.message));
        }

        function start() {
            fetch('/api/start', {method: 'POST'})
                .then(response => response.json())
                .then(data => alert(data.message));
        }

        function stop() {
            fetch('/api/stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => alert(data.message));
        }

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
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(data)
            })
            .then(response => response.json())
            .then(data => alert(data.message));
        }

        // Start periodic updates
        setInterval(() => {
            updateStatus();
            updateSensors();
            updateChart();
        }, 100);
    </script>
</body>
</html>
)rawliteral";
}