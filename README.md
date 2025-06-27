

# High-Performance PID Line Follower Robot (ESP32 & Arduino Nano Versions)

This repository contains the code and documentation for building a fast and efficient line-following robot using a PID (Proportional-Integral-Derivative) control algorithm. The project is provided in two versions:

1.  **ESP32 Version:** An advanced build that uses the ESP32's built-in Bluetooth for real-time, on-the-fly PID tuning from a smartphone or computer.
2.  **Arduino Nano Version:** A simpler, self-contained build perfect for learning the fundamentals of PID control with manual tuning directly in the code.

## Table of Contents

1.  [Project Overview]
2.  (\#2-how-pid-control-works)
3.  (\#3-esp32-version-advanced---with-bluetooth-tuning)
      * (\#hardware-esp32)
      * (\#wiring-esp32)
      * (\#setup--operation-esp32)
      * (\#code-esp32)
4.  (\#4-arduino-nano-version-standard---in-code-tuning)
      * [Hardware (Nano)]
      * [Wiring (Nano)]
      * (\#setup--operation-nano)
      * [Code (Nano)]
5.  (\#5-a-guide-to-pid-tuning-for-both-versions)

-----

### 1\. Project Overview

This project demonstrates how to build a high-performance line-following robot. Instead of using basic `if-else` logic that results in wobbly movement, this robot employs a PID controller. This allows it to dynamically calculate the precise amount of correction needed to stay on the line, resulting in smoother turns and higher possible speeds.

The robot uses a 5-channel infrared (IR) sensor array to detect the line, an L298N motor driver to control two DC motors, and either an Arduino Nano or an ESP32 as its brain.

### 2\. How PID Control Works

The core of this robot is the PID algorithm. It works by continuously calculating an "error" value, which represents how far the robot is from the center of the line. It then calculates a correction based on three terms [5, 6, 7]:

  * **Proportional (P):** This term reacts to the **current error**. If the robot is far from the line, the correction is large. If it's only slightly off, the correction is small. This provides the primary steering force but can cause wobbling on its own.
  * **Integral (I):** This term looks at the **sum of past errors**. If the robot has a small, consistent drift to one side (due to mechanical imbalances), the integral term will build up over time and apply a counteracting force to bring the robot back to the exact center.
  * **Derivative (D):** This term looks at the **rate of change of the error**, essentially predicting future errors. As the robot turns back towards the line, the error decreases rapidly. The D-term anticipates this and dampens the correction to prevent the robot from overshooting the line and oscillating. This is what makes the movement smooth.

The final correction is a weighted sum of these three terms: `Correction = (Kp * P) + (Ki * I) + (Kd * D)`. Our job is to find the perfect `Kp`, `Ki`, and `Kd` values for our specific robot.

-----

### 3\. ESP32 Version (Advanced - with Bluetooth Tuning)

This version leverages the powerful ESP32 microcontroller, which has built-in Bluetooth capabilities. This allows you to tune the PID constants (`Kp`, `Ki`, `Kd`) in real-time from a smartphone app, which is a professional and highly efficient way to optimize performance without needing to re-upload code for every adjustment.

#### Hardware (ESP32)

  * **Microcontroller:** ESP32 Development Board
  * **Sensor Array:** 5-Channel IR Reflectance Sensor Array (e.g., Pololu QTR-5A, SmartElex RLS-05)
  * **Motor Driver:** L298N Dual H-Bridge Motor Driver
  * **Motors:** 2 x N20 Micro Metal Gear DC Motors
  * **Wheels:** 2 x Wheels compatible with N20 motor shafts
  * **Chassis:** A lightweight 2WD robot chassis
  * **Power Supply:** 7.4V Li-ion/LiPo Battery Pack or a 6V-9V AA battery pack.
  * **Wiring:** Jumper wires and a breadboard.

#### Wiring (ESP32)

Connect your components to the ESP32 board as defined in the code.

| Component Pin | ESP32 GPIO Pin |
| :--- | :--- |
| **Left Motor (L298N)** | |
| ENA (PWM Speed) | 23 |
| IN1 (Direction) | 21 |
| IN2 (Direction) | 22 |
| **Right Motor (L298N)** | |
| ENB (PWM Speed) | 32 |
| IN3 (Direction) | 25 |
| IN4 (Direction) | 33 |
| **L298N Standby** | |
| STBY | 19 |
| **IR Sensor Array** | |
| Sensor 1 (Leftmost) | 26 |
| Sensor 2 | 27 |
| Sensor 3 | 14 |
| Sensor 4 | 12 |
| Sensor 5 (Rightmost) | 13 |

***Note:*** *It is critical to connect the ground (GND) of your motor power supply to a GND pin on the ESP32 to create a common ground reference.*

#### Setup & Operation (ESP32)

1.  **Assemble Hardware:** Build your robot and wire all components according to the table above.
2.  **Install Libraries:** In the Arduino IDE, go to **Sketch \> Include Library \> Manage Libraries...** and install the following:
      * `QTRSensors` by Pololu
      * `L298N` (ensure it's a library that matches the `L298N motor1(PWMA, AIN1, AIN2);` constructor format).
3.  **Upload Code:**
      * Open the ESP32 code (provided below) in the Arduino IDE.
      * Go to **Tools \> Board** and select your ESP32 model (e.g., "ESP32 Dev Module").
      * Select the correct **Port** and click **Upload**.
4.  **Sensor Calibration (Physical Step):**
      * After uploading, the onboard LED will light up. This signals the start of the 10-second sensor calibration phase.
      * During these 10 seconds, you must **physically slide the robot's sensor array back and forth across the black line**. This allows the sensors to learn the min/max values for black and white. This step is crucial and must be done first.
5.  **PID Tuning & Control via Bluetooth:**
      * On your smartphone, install a Bluetooth Serial Terminal app (e.g., "Serial Bluetooth Terminal" for Android).
      * Power on your robot. The ESP32 will start a Bluetooth serial device.
      * In your phone's Bluetooth settings, pair with the new device (its name will be the one set in `SerialBT.begin()`).
      * Open the Bluetooth terminal app and connect to the paired device.
      * You can now send commands to tune the PID constants and control the robot. The code uses a 2-byte protocol: the first byte identifies the parameter, and the second is the value.
          * **Start/Stop Robot:** Send `7,1` to start, `7,0` to stop.
          * **Set Kp:** Send `1, [value]` (e.g., `1,10` sets Kp to 10).
          * **Set Ki:** Send `3, [value]` (e.g., `3,5` sets Ki to 5).
          * **Set Kd:** Send `5, [value]` (e.g., `5,15` sets Kd to 15).
          * **Set Multipliers:** The code includes multipliers to allow for fractional PID values. For example, to set `Kp` to `0.8`, you would send `1,8` (for Kp) and `2,1` (for multiP, which means divide by $10^1$).

#### Code (ESP32)

```cpp
#include <L298N.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"

#if!defined(CONFIG_BT_ENABLED) ||!defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// --- PIN DEFINITIONS (ESP32) ---
#define AIN1 21
#define BIN1 25
#define AIN2 22
#define BIN2 33
#define PWMA 23
#define PWMB 32
#define STBY 19

// --- Motor and Sensor Initialization ---
L298N motor1(PWMA, AIN1, AIN2, STBY);
L298N motor2(PWMB, BIN1, BIN2, STBY);

QTRSensors qtr;
BluetoothSerial SerialBT;

const uint8_t SensorCount = 5;
uint16_t sensorValues;

// --- PID and Control Variables ---
float Kp = 0;
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
boolean onoff = false;
int val, cnt = 0, v[1];
uint16_t position;
int P, D, I, previousError, error;
int lsp, rsp;
int lfspeed = 230;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t){26, 27, 14, 12, 13}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  SerialBT.begin("ESP32_LineFollower"); // Device name for Bluetooth
  Serial.println("Bluetooth Started! Ready to pair...");

  // --- Sensor Calibration ---
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  delay(1000);
}

void loop() {
  if (SerialBT.available()) {
    while (SerialBT.available() == 0);
    valuesread();
    processing();
  }
  if (onoff == true) {
    robot_control();
  } else {
    motor1.stop();
    motor2.stop();
  }
}

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;

  if (sensorValues >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[1] >= 980 && sensorValues[4] >= 980) {
    if (previousError > 0) {
      motor_drive(-230, 230);
    } else {
      motor_drive(230, -230);
    }
    position = qtr.readLineBlack(sensorValues);
    return;
  }
  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;

  float Pvalue = (Kp / pow(10, multiP)) * P;
  float Ivalue = (Ki / pow(10, multiI)) * I;
  float Dvalue = (Kd / pow(10, multiD)) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor_drive(lsp, rsp);
}

void valuesread() {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2) cnt = 0;
}

void processing() {
  int a = v[2];
  if (a == 1) { Kp = v[3]; }
  if (a == 2) { multiP = v[3]; }
  if (a == 3) { Ki = v[3]; }
  if (a == 4) { multiI = v[3]; }
  if (a == 5) { Kd = v[3]; }
  if (a == 6) { multiD = v[3]; }
  if (a == 7) { onoff = v[3]; }
}

void motor_drive(int left, int right) {
  if (right > 0) {
    motor2.setSpeed(right);
    motor2.forward();
  } else {
    motor2.setSpeed(abs(right));
    motor2.backward();
  }

  if (left > 0) {
    motor1.setSpeed(left);
    motor1.forward();
  } else {
    motor1.setSpeed(abs(left));
    motor1.backward();
  }
}
```

-----

### 4\. Arduino Nano Version (Standard - In-Code Tuning)

This version is perfect for learning the core concepts of PID control without the added complexity of wireless communication. All PID tuning is done by changing constant values in the code and re-uploading to the board.

#### Hardware (Nano)

  * **Microcontroller:** Arduino Nano
  * **Sensor Array:** 5-Channel IR Reflectance Sensor Array
  * **Motor Driver:** L298N Dual H-Bridge Motor Driver
  * **Motors:** 2 x N20 Micro Metal Gear DC Motors
  * **Wheels:** 2 x Wheels compatible with N20 motor shafts
  * **Chassis:** A lightweight 2WD robot chassis
  * **Power Supply:** 7.4V Li-ion/LiPo Battery Pack or a 6V-9V AA battery pack.
  * **Wiring:** Jumper wires and a breadboard.

#### Wiring (Nano)

| Component Pin | Arduino Nano Pin |
| :--- | :--- |
| **Left Motor (L298N)** | |
| ENA (PWM Speed) | D10 |
| IN1 (Direction) | D8 |
| IN2 (Direction) | D7 |
| **Right Motor (L298N)** | |
| ENB (PWM Speed) | D9 |
| IN3 (Direction) | D12 |
| IN4 (Direction) | D11 |
| **IR Sensor Array** | |
| Sensor 1 (Leftmost) | A0 |
| Sensor 2 | A1 |
| Sensor 3 | A2 |
| Sensor 4 | A3 |
| Sensor 5 (Rightmost) | A4 |

***Note:*** *It is critical to connect the ground (GND) of your motor power supply to a GND pin on the Arduino Nano to create a common ground reference.*

#### Setup & Operation (Nano)

1.  **Assemble Hardware:** Build your robot and wire all components according to the table above.
2.  **Install Libraries:** In the Arduino IDE, go to **Sketch \> Include Library \> Manage Libraries...** and install `QTRSensors` by Pololu and a suitable `L298N` library.
3.  **Upload Code:**
      * Open the Arduino Nano code (provided below).
      * Go to **Tools \> Board** and select **"Arduino Nano"**.
      * Select the correct **Port** and click **Upload**.
4.  **Calibrate and Run:**
      * After uploading, the onboard LED will light up. This signals the start of the 10-second sensor calibration phase.
      * During these 10 seconds, **physically slide the robot's sensor array back and forth across the black line**.
      * After 10 seconds, the LED will turn OFF. The robot is now calibrated and will immediately start running.

#### Code (Nano)

```cpp
#include <L298N.h>
#include <QTRSensors.h>

// --- 1. PIN DEFINITIONS (Arduino Nano) ---
// Motor A (Left Motor)
#define ENA 10
#define IN1 8
#define IN2 7

// Motor B (Right Motor)
#define ENB 9
#define IN3 12
#define IN4 11

// --- 2. PID TUNING CONSTANTS ---
// Adjust these values to tune your robot's performance.
float Kp = 0.08;
float Ki = 0.0001;
float Kd = 0.9;

// --- 3. ROBOT SPEED ---
int lfspeed = 200; // Base speed (0-255)

// --- Library and Global Variable Initialization ---
L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues;

uint16_t position;
int P, D, I, previousError, error;
int lsp, rsp;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t){A0, A1, A2, A3, A4}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  // --- Sensor Calibration ---
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  delay(1000);
}

void loop() {
  robot_control();
}

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  error = position - 2000;

  if (sensorValues >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[1] >= 980 && sensorValues[4] >= 980) {
    if (previousError > 0) {
      motor_drive(230, -230);
    } else {
      motor_drive(-230, 230);
    }
    return;
  }
  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;

  int PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right) {
  if (left > 0) {
    motor1.setSpeed(left);
    motor1.forward();
  } else {
    motor1.setSpeed(abs(left));
    motor1.backward();
  }

  if (right > 0) {
    motor2.setSpeed(right);
    motor2.forward();
  } else {
    motor2.setSpeed(abs(right));
    motor2.backward();
  }
}
```

-----

### 5\. A Guide to PID Tuning (For Both Versions)

Tuning is an iterative process of adjusting `Kp`, `Ki`, and `Kd` to match your robot's specific physical characteristics (weight, motors, friction). The goal is to get a fast response with minimal overshoot and no oscillation. [8]

**Follow this systematic approach for the best results:**

1.  **Tune Kp (Proportional Gain):**

      * Set `Ki` and `Kd` to `0`.
      * Start with a small `Kp` value (e.g., `0.05`).
      * Observe the robot. It will likely be slow and sluggish.
      * Gradually increase `Kp`. You will notice the robot becomes more responsive.
      * Keep increasing `Kp` until the robot follows the line but oscillates (wobbles) back and forth aggressively. The ideal `Kp` is a value just *before* it becomes overly aggressive.

2.  **Tune Kd (Derivative Gain):**

      * Now, keep your best `Kp` value.
      * Start with a small `Kd` value (e.g., `0.1`).
      * Gradually increase `Kd`. You should see the oscillations from the P-term start to reduce. The robot's movement will become smoother.
      * The goal is to find a `Kd` that eliminates the wobble without making the robot slow to react to curves.

3.  **Tune Ki (Integral Gain):**

      * The robot should now be running well with `Kp` and `Kd`.
      * Observe it on a long, straight section of the track. If it consistently drives slightly to one side of the line (a steady-state error), you need the I-term.
      * Add a **very small** `Ki` value (e.g., `0.0001`). This should slowly correct the drift over time.
      * **Caution:** Be very careful with `Ki`. Too high a value will introduce new oscillations and make the system unstable. For many line followers, a `Ki` of 0 is sufficient.
