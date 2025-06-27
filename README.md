
# High-Performance PID Line Follower Robot

This repository contains the code and documentation for building a fast and efficient line-following robot using a PID (Proportional-Integral-Derivative) control algorithm. The project is provided in two versions, one for the **Arduino Nano** and one for the **ESP32**, allowing you to choose the microcontroller that best fits your needs.

The goal of this project is to move beyond simple, jerky line followers and create a robot that can navigate a track smoothly and at high speed.

## Table of Contents

1.  [Project Overview](https://www.google.com/search?q=%231-project-overview)
2.  (\#2-how-pid-control-works)
3.  [Arduino Nano Version](https://www.google.com/search?q=%233-arduino-nano-version)
      * [Hardware (Nano)](https://www.google.com/search?q=%23hardware-nano)
      * [Wiring (Nano)](https://www.google.com/search?q=%23wiring-nano)
      * [Code (Nano)](https://www.google.com/search?q=%23code-nano)
4.  (\#4-esp32-version)
      * (\#hardware-esp32)
      * (\#wiring-esp32)
      * (\#code-esp32)
5.  (\#5-setup-and-calibration-for-both-versions)
6.  (\#6-a-guide-to-pid-tuning)

-----

### 1\. Project Overview

This project demonstrates how to build a high-performance line-following robot. Instead of using basic `if-else` logic that results in wobbly movement, this robot employs a PID controller. This allows it to dynamically calculate the precise amount of correction needed to stay on the line, resulting in smoother turns and higher possible speeds.

The robot uses a 5-channel infrared (IR) sensor array to detect the line, an L298N motor driver to control two DC motors, and either an Arduino Nano or an ESP32 as its brain.

### 2\. How PID Control Works

The core of this robot is the PID algorithm. It works by continuously calculating an "error" value, which represents how far the robot is from the center of the line. It then calculates a correction based on three terms [5, 6]:

  * **Proportional (P):** This term reacts to the **current error**. If the robot is far from the line, the correction is large. If it's only slightly off, the correction is small. This provides the primary steering force but can cause wobbling on its own.[5, 6]
  * **Integral (I):** This term looks at the **sum of past errors**. If the robot has a small, consistent drift to one side (due to mechanical imbalances), the integral term will build up over time and apply a counteracting force to bring the robot back to the exact center.[5, 6]
  * **Derivative (D):** This term looks at the **rate of change of the error**, essentially predicting future errors. As the robot turns back towards the line, the error decreases rapidly. The D-term anticipates this and dampens the correction to prevent the robot from overshooting the line and oscillating. This is what makes the movement smooth.[5, 6]

The final correction is a weighted sum of these three terms: `Correction = (Kp * P) + (Ki * I) + (Kd * D)`. Our job is to find the perfect `Kp`, `Ki`, and `Kd` values for our specific robot.

-----

### 3\. Arduino Nano Version

This version is great for beginners or for projects where the advanced connectivity of the ESP32 is not required. The Arduino Nano is small, breadboard-friendly, and has a huge community for support.

#### Hardware (Nano)

  * **Microcontroller:** Arduino Nano
  * **Sensor Array:** 5-Channel IR Reflectance Sensor Array (e.g., Pololu QTR-5A, SmartElex RLS-05)
  * **Motor Driver:** L298N Dual H-Bridge Motor Driver
  * **Motors:** 2 x N20 Micro Metal Gear DC Motors
  * **Wheels:** 2 x Wheels compatible with N20 motor shafts
  * **Chassis:** A lightweight 2WD robot chassis
  * **Power Supply:** 7.4V Li-ion/LiPo Battery Pack or a 6V-9V AA battery pack.
  * **Wiring:** Jumper wires and a breadboard.

#### Wiring (Nano)

Connect your components to the Arduino Nano as defined in the code.

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

***Note:*** *It is critical to connect the ground (GND) of your motor power supply to a GND pin on the Arduino Nano to create a common ground reference*.[7]

#### Code (Nano)

This code has been refactored from the original to remove Bluetooth functionality and work directly with the Arduino Nano.

```cpp
#include <L298N.h>
#include <QTRSensors.h>

// --- 1. PIN DEFINITIONS (Arduino Nano) ---
// Motor A (Left Motor)
#define ENA 10  // PWM Speed Control Pin (must be a PWM pin)
#define IN1 8
#define IN2 7

// Motor B (Right Motor)
#define ENB 9   // PWM Speed Control Pin (must be a PWM pin)
#define IN3 12
#define IN4 11

// --- 2. PID TUNING CONSTANTS ---
// These values will need to be tuned for your specific robot.
float Kp = 0.08;
float Ki = 0.0001;
float Kd = 0.9;

// --- 3. ROBOT SPEED ---
// This is the base speed of the robot. 200 is a good starting point.
// Max speed is 255.
int lfspeed = 200;

// --- Library and Global Variable Initialization ---
L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;

void setup() {
  // Configure the QTR sensors
  qtr.setTypeAnalog();
  // Connect sensor pins to A0, A1, A2, A3, A4
  qtr.setSensorPins((const uint8_t){A0, A1, A2, A3, A4}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  // --- Sensor Calibration ---
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED to indicate calibration mode
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // Turn off the LED when calibration is complete

  delay(1000); // Wait a second before starting
}

void loop() {
  robot_control();
}

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  error = position - 2000;

  // --- Line Loss Logic ---
  if (sensorValues >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[4] >= 980) {
    if (previousError > 0) { // If it was last seen to the right
      motor_drive(230, -230); // Turn hard right to find it
    } else { // If it was last seen to the left
      motor_drive(-230, 230); // Turn hard left to find it
    }
    return;
  }

  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  // Constrain motor speeds to the valid PWM range
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

### 4\. ESP32 Version

This version is for more advanced projects. The ESP32 is a powerhouse with a dual-core processor, more memory, and built-in Wi-Fi and Bluetooth, making it ideal for robots that might later be upgraded with wireless control, more sensors, or more complex algorithms like maze-solving.

#### Hardware (ESP32)

  * **Microcontroller:** ESP32 Development Board [8]
  * **Sensor Array:** 5-Channel IR Reflectance Sensor Array (e.g., Pololu QTR-5A, SmartElex RLS-05)
  * **Motor Driver:** L298N Dual H-Bridge Motor Driver
  * **Motors:** 2 x N20 Micro Metal Gear DC Motors
  * **Wheels:** 2 x Wheels compatible with N20 motor shafts
  * **Chassis:** A lightweight 2WD robot chassis
  * **Power Supply:** 7.4V Li-ion/LiPo Battery Pack.
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

***Note:*** *It is critical to connect the ground (GND) of your motor power supply to a GND pin on the ESP32 to create a common ground reference.* [7]

#### Code (ESP32)

This code has been refactored from the original to remove Bluetooth tuning functionality.

```cpp
#include <L298N.h>
#include <QTRSensors.h>

// --- 1. PIN DEFINITIONS (ESP32) ---
// Motor A (e.g., Left Motor)
#define PWMA 23
#define AIN1 21
#define AIN2 22

// Motor B (e.g., Right Motor)
#define PWMB 32
#define BIN1 25
#define BIN2 33

// Standby Pin for the L298N Driver
#define STBY 19

// --- 2. PID TUNING CONSTANTS ---
float Kp = 0.08;
float Ki = 0.0001;
float Kd = 1.0;

// --- 3. ROBOT SPEED ---
int lfspeed = 200;

// --- Library and Global Variable Initialization ---
L298N motor1(PWMA, AIN1, AIN2, STBY);
L298N motor2(PWMB, BIN1, BIN2, STBY);

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues;

uint16_t position;
int P, D, I, previousError, error;
int lsp, rsp;

void setup() {
  // Configure the QTR sensors
  qtr.setTypeAnalog();
  // NOTE: Ensure these GPIO pins support ADC on your ESP32 model
  qtr.setSensorPins((const uint8_t){26, 27, 14, 12, 13}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  // --- Sensor Calibration ---
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // Turn off the LED

  delay(1000);
}

void loop() {
  robot_control();
}

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  error = position - 2000;

  // --- Line Loss Logic ---
  if (sensorValues >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[4] >= 980) {
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

  float PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  // Constrain motor speeds
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

### 5\. Setup and Calibration (For Both Versions)

#### Step 1: Assemble the Hardware

Build your robot chassis and wire all the components according to the correct table for your microcontroller (Nano or ESP32).

#### Step 2: Install Libraries

This project requires two libraries to be installed in the Arduino IDE.

1.  **QTRSensors:** Go to **Sketch \> Include Library \> Manage Libraries...**. Search for "QTRSensors" and install the library by Pololu.
2.  **L298N:** The code uses a specific `L298N.h` library. You can find several versions in the Library Manager. Ensure the one you install supports the constructor format used in the code.

#### Step 3: Upload the Code

1.  Open the correct `.ino` file for your board (Nano or ESP32).
2.  Connect your board to your computer.
3.  Go to **Tools \> Board** and select your board (e.g., "Arduino Nano" or "ESP32 Dev Module").
4.  Go to **Tools \> Port** and select the correct COM port.
5.  Click the **Upload** button.

#### Step 4: Calibrate the Sensors (Crucial Step)

For the PID algorithm to work, the robot must know the range of readings for the black line and the white surface.[8]

1.  After uploading the code, power your robot and place it on the track.
2.  The onboard LED on your microcontroller will turn ON. This signals the start of the 10-second calibration phase.
3.  During these 10 seconds, you must **physically slide the robot's sensor array back and forth across the black line**. Make sure every sensor passes over both the black line and the white surface multiple times.[9]
4.  After 10 seconds, the LED will turn OFF. The robot is now calibrated and will begin running.

### 6\. A Guide to PID Tuning

Tuning is an iterative process of adjusting the `Kp`, `Ki`, and `Kd` constants in the code to match your robot's specific physical characteristics (weight, motors, friction). The goal is to get a fast response with minimal overshoot and no oscillation.[10]

**Follow this systematic approach for the best results:** [11]

1.  **Tune Kp (Proportional Gain):**

      * In the code, set `Ki` and `Kd` to `0`.
      * Start with a small `Kp` value (e.g., `0.05`).
      * Upload the code and observe the robot. It will likely be slow and sluggish.
      * Gradually increase `Kp` and re-upload. You will notice the robot becomes more responsive.
      * Keep increasing `Kp` until the robot follows the line but oscillates (wobbles) back and forth aggressively. The ideal `Kp` is a value just *before* it becomes overly aggressive.[10, 11]

2.  **Tune Kd (Derivative Gain):**

      * Now, keep your best `Kp` value.
      * Start with a small `Kd` value (e.g., `0.1`).
      * Gradually increase `Kd`. You should see the oscillations from the P-term start to reduce. The robot's movement will become smoother.
      * The goal is to find a `Kd` that eliminates the wobble without making the robot slow to react to curves.[10, 11]

3.  **Tune Ki (Integral Gain):**

      * The robot should now be running well with `Kp` and `Kd`.
      * Observe it on a long, straight section of the track. If it consistently drives slightly to one side of the line (a steady-state error), you need the I-term.
      * Add a **very small** `Ki` value (e.g., `0.0001`). This should slowly correct the drift over time.
      * **Caution:** Be very careful with `Ki`. Too high a value will introduce new oscillations and make the system unstable. For many line followers, a `Ki` of 0 is sufficient.[12, 11]
