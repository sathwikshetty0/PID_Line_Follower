#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>

// Motor Driver TB6612FNG Pin Configuration
#define PWMA 34
#define AIN2 35
#define AIN1 32
#define STBY 33
#define BIN1 15
#define BIN2 26
#define PWMB 27

// QTR-8RC Sensor Configuration
const uint8_t irPins[] = {2, 4, 5, 18, 19, 21, 22, 23};
#define SENSOR_COUNT 8
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// WiFi Configuration
const char* ssid = "ne";
const char* password = "qwer4321";

// Web Server
WebServer server(80);

// PID Variables
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

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // Enable motor driver
  digitalWrite(STBY, HIGH);
  
  // Initialize QTR sensors
  qtr.setTypeRC();
  qtr.setSensorPins(irPins, SENSOR_COUNT);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // Setup web server routes
  setupWebServer();
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
  
  if (isRunning && isCalibrated) {
    runPIDControl();
  } else if (!isRunning) {
    stopMotors();
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", getMainHTML());
  });
  
  server.on("/calibrate", HTTP_POST, []() {
    calibrationMode = true;
    isCalibrated = false;
    
    Serial.println("Starting calibration...");
    
    for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate();
      delay(25);
    }
    
    isCalibrated = true;
    calibrationMode = false;
    Serial.println("Calibration complete");
    
    server.send(200, "text/plain", "Calibration complete");
  });
  
  server.on("/start", HTTP_POST, []() {
    if (isCalibrated) {
      isRunning = true;
      integral = 0;
      server.send(200, "text/plain", "Started");
    } else {
      server.send(400, "text/plain", "Please calibrate first");
    }
  });
  
  server.on("/stop", HTTP_POST, []() {
    isRunning = false;
    stopMotors();
    server.send(200, "text/plain", "Stopped");
  });
  
  server.on("/pid", HTTP_POST, []() {
    if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
    if (server.hasArg("base")) baseSpeed = server.arg("base").toInt();
    if (server.hasArg("max")) maxSpeed = server.arg("max").toInt();
    
    Serial.printf("PID updated: Kp=%.4f, Ki=%.4f, Kd=%.4f, Base=%d, Max=%d\n", 
                  Kp, Ki, Kd, baseSpeed, maxSpeed);
    
    server.send(200, "text/plain", "PID updated");
  });
  
  server.on("/status", HTTP_GET, []() {
    String status = "Calibrated:" + String(isCalibrated) + 
                   ",Running:" + String(isRunning) + 
                   ",Kp:" + String(Kp) + 
                   ",Ki:" + String(Ki) + 
                   ",Kd:" + String(Kd) + 
                   ",Base:" + String(baseSpeed) + 
                   ",Max:" + String(maxSpeed);
    server.send(200, "text/plain", status);
  });
  
  server.on("/sensors", HTTP_GET, []() {
    qtr.read(sensorValues);
    String sensorData = "";
    for (int i = 0; i < SENSOR_COUNT; i++) {
      sensorData += String(sensorValues[i]);
      if (i < SENSOR_COUNT - 1) sensorData += ",";
    }
    
    if (isCalibrated) {
      uint16_t position = qtr.readLineBlack(sensorValues);
      sensorData += "|" + String(position) + "|" + String(3500 - position);
    }
    
    server.send(200, "text/plain", sensorData);
  });
}

void runPIDControl() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  float error = 3500 - position;
  
  integral += error;
  float derivative = error - lastError;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  // Limit integral windup
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  
  int motorSpeedA = constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed);
  int motorSpeedB = constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed);
  
  setMotorSpeed(motorSpeedA, motorSpeedB);
  
  lastError = error;
}

void setMotorSpeed(int speedA, int speedB) {
  // Motor A
  if (speedA > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speedA = -speedA;
  }
  analogWrite(PWMA, speedA);
  
  // Motor B
  if (speedB > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speedB = -speedB;
  }
  analogWrite(PWMB, speedB);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

String getMainHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>PID Line Follower</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 40px; background: #f1f1f1; }
        .container { max-width: 800px; margin: auto; background: white; padding: 30px; border-radius: 10px; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }
        button { padding: 15px 25px; margin: 10px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }
        .btn-blue { background: #2196F3; color: white; }
        .btn-green { background: #4CAF50; color: white; }
        .btn-red { background: #f44336; color: white; }
        .btn-orange { background: #ff9800; color: white; }
        input { padding: 8px; margin: 5px; width: 80px; }
        .sensors { display: grid; grid-template-columns: repeat(8, 1fr); gap: 5px; margin: 10px 0; }
        .sensor { height: 40px; background: #ddd; border-radius: 3px; display: flex; align-items: center; justify-content: center; }
        .sensor.active { background: #f44336; color: white; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
        .info { background: #e7f3ff; border: 1px solid #b3d9ff; }
    </style>
</head>
<body>
    <div class="container">
        <h1>PID Line Follower Dashboard</h1>
        
        <div class="section">
            <h2>System Control</h2>
            <div id="status" class="status info">Loading...</div>
            <button class="btn-orange" onclick="calibrate()">Calibrate Sensors</button>
            <button class="btn-green" onclick="start()">Start Following</button>
            <button class="btn-red" onclick="stop()">Stop</button>
        </div>

        <div class="section">
            <h2>Sensor Readings</h2>
            <div id="sensors" class="sensors"></div>
            <div id="position">Position: - | Error: -</div>
        </div>

        <div class="section">
            <h2>PID Parameters</h2>
            <div>
                Kp: <input type="number" id="kp" step="0.001" value="0.04">
                Ki: <input type="number" id="ki" step="0.001" value="0.002">
                Kd: <input type="number" id="kd" step="0.001" value="0.015">
            </div>
            <div>
                Base Speed: <input type="number" id="base" min="0" max="255" value="100">
                Max Speed: <input type="number" id="max" min="0" max="255" value="255">
            </div>
            <button class="btn-blue" onclick="updatePID()">Update PID</button>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/status')
                .then(response => response.text())
                .then(data => {
                    var parts = data.split(',');
                    var calibrated = parts[0].split(':')[1] === '1';
                    var running = parts[1].split(':')[1] === '1';
                    
                    document.getElementById('status').innerHTML = 
                        'Calibrated: ' + (calibrated ? 'YES' : 'NO') + 
                        ' | Running: ' + (running ? 'YES' : 'NO');
                    
                    if (parts.length > 2) {
                        document.getElementById('kp').value = parts[2].split(':')[1];
                        document.getElementById('ki').value = parts[3].split(':')[1];
                        document.getElementById('kd').value = parts[4].split(':')[1];
                        document.getElementById('base').value = parts[5].split(':')[1];
                        document.getElementById('max').value = parts[6].split(':')[1];
                    }
                });
        }

        function updateSensors() {
            fetch('/sensors')
                .then(response => response.text())
                .then(data => {
                    var parts = data.split('|');
                    var sensors = parts[0].split(',');
                    
                    var html = '';
                    for (var i = 0; i < 8; i++) {
                        var value = parseInt(sensors[i]) || 0;
                        var active = value < 500 ? ' active' : '';
                        html += '<div class="sensor' + active + '">' + value + '</div>';
                    }
                    document.getElementById('sensors').innerHTML = html;
                    
                    if (parts.length > 1) {
                        document.getElementById('position').innerHTML = 
                            'Position: ' + parts[1] + ' | Error: ' + parts[2];
                    }
                });
        }

        function calibrate() {
            document.getElementById('status').innerHTML = 'Calibrating...';
            fetch('/calibrate', { method: 'POST' })
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = data;
                });
        }

        function start() {
            fetch('/start', { method: 'POST' })
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = data;
                });
        }

        function stop() {
            fetch('/stop', { method: 'POST' })
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = data;
                });
        }

        function updatePID() {
            var kp = document.getElementById('kp').value;
            var ki = document.getElementById('ki').value;
            var kd = document.getElementById('kd').value;
            var base = document.getElementById('base').value;
            var max = document.getElementById('max').value;
            
            fetch('/pid?kp=' + kp + '&ki=' + ki + '&kd=' + kd + '&base=' + base + '&max=' + max, 
                  { method: 'POST' })
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = data;
                });
        }

        // Update every 500ms
        setInterval(function() {
            updateStatus();
            updateSensors();
        }, 500);

        // Initial load
        updateStatus();
        updateSensors();
    </script>
</body>
</html>
)rawliteral";
}