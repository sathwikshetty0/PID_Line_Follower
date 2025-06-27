
# Arduino PID Line Follower Robot

This repository contains the Arduino code for a fast and efficient line follower robot that uses a PID (Proportional-Integral-Derivative) control algorithm. This approach allows the robot to follow a black line on a white surface smoothly, correcting its path with precision and speed, which is a significant improvement over simple "bang-bang" controllers.

This guide will walk you through understanding the code, setting up the hardware, and tuning the robot for optimal performance.

## Table of Contents

1.  [How it Works](https://www.google.com/search?q=%23how-it-works)
2.  (\#hardware-required)
3.  [Wiring and Connections](https://www.google.com/search?q=%23wiring-and-connections)
4.  (\#step-by-step-guide-to-get-started)
    1.  [Assemble the Hardware](https://www.google.com/search?q=%23step-1-assemble-the-hardware)
    2.  [Upload the Code](https://www.google.com/search?q=%23step-2-upload-the-code)
    3.  (\#step-3-calibrate-the-sensors-crucial)
    4.  (\#step-4-run-the-robot)
5.  (\#tuning-the-pid-constants)
6.  (\#code-breakdown)

## How it Works

The robot uses an array of 5 infrared (IR) sensors to determine its position relative to the line. Instead of making a simple left/right decision, it calculates a precise position value. This value is then fed into a PID control algorithm. [1]

  * **Position Sensing:** The robot reads the values from the 5 IR sensors and calculates a weighted average. This gives a single continuous value representing the line's position (e.g., 0 for far left, 4000 for far right). The goal (or "setpoint") is to keep this position value at the center (e.g., 2000). [1]
  * **PID Control:** The difference between the current position and the setpoint is the **error**. The PID controller uses this error to calculate a correction value. [2, 3]
      * **Proportional (P):** Reacts to the *current* error. If the robot is far from the line, it makes a sharp correction. If it's close, the correction is gentle. This provides the main steering force.
      * **Integral (I):** Accumulates *past* errors. This helps correct for small, consistent drifts, ensuring the robot stays perfectly centered on straight lines.
      * **Derivative (D):** Reacts to the *rate of change* of the error. It anticipates the robot's future position, damping the oscillations caused by the P-term and preventing the robot from overshooting the line during corrections. This is what makes the movement smooth.
  * **Motor Control:** The final PID value is used to adjust the speed of the left and right motors. If the robot needs to turn right, it increases the speed of the left motor and decreases the speed of the right motor, and vice-versa.

## Hardware Required

  * **Microcontroller:** Arduino Nano (recommended for its small size) or Arduino Uno.
  * **Sensor Array:** A 5-channel IR reflectance sensor array (e.g., TCRT5000 based).
  * **Motor Driver:** A dual H-bridge motor driver like the L298N or the more efficient TB6612FNG.
  * **Motors:** 2 x N20 Micro Gear DC Motors with wheels.
  * **Chassis:** A lightweight 2WD robot chassis.
  * **Power Supply:** 7.4V LiPo Battery or a 6V-9V battery pack (e.g., 4-6 AA batteries).
  * **Wiring:** Jumper wires and a breadboard for connections.

## Wiring and Connections

Connect the components to your Arduino board according to the pin definitions in the code.

| Component Pin | Arduino Nano Pin |
| :--- | :--- |
| **Motor Driver (Left Motor)** | |
| PWM Pin (e.g., ENA) | D10 |
| Input 1 | D7 |
| Input 2 | D8 |
| **Motor Driver (Right Motor)** | |
| PWM Pin (e.g., ENB) | D9 |
| Input 1 | D12 |
| Input 2 | D11 |
| **IR Sensor Array** | |
| Sensor 1 (Leftmost) | A0 |
| Sensor 2 | A1 |
| Sensor 3 | A2 |
| Sensor 4 | A3 |
| Sensor 5 (Rightmost) | A4 |

*Note: Ensure you have a common ground (GND) between the motor power supply and the Arduino.*

## Step-by-Step Guide to Get Started

### Step 1: Assemble the Hardware

Connect all the components as shown in the wiring table above. Ensure the sensors are mounted at the front of the robot, facing the ground.

### Step 2: Upload the Code

1.  Open the `PID.ino` file in the Arduino IDE.
2.  Connect your Arduino board to your computer.
3.  Go to **Tools \> Board** and select your Arduino board (e.g., "Arduino Nano").
4.  Go to **Tools \> Port** and select the correct COM port.
5.  Click the "Upload" button.

### Step 3: Calibrate the Sensors (CRUCIAL)

This is the most important step for accurate line following. The robot needs to learn the difference between the black line and the white surface. [4]

1.  After uploading the code, open the Serial Monitor (**Tools \> Serial Monitor**).
2.  The code will initiate a 10-second calibration phase.
3.  During these 10 seconds, you must **physically slide your robot's sensor array back and forth across the black line**. Make sure every sensor passes over both the black line and the white surface multiple times.
4.  This process allows the robot to record the minimum and maximum reading for each sensor, which is essential for calculating its position accurately.

### Step 4: Run the Robot

After the 10-second calibration is complete, place the robot on the track. It will now start following the line using the PID algorithm.

## Tuning the PID Constants

The default values for `Kp`, `Ki`, and `Kd` in the code are a starting point. To get the best performance, you must "tune" these constants for your specific robot's weight, motors, and track surface. This is a process of trial and error. [5]

Follow this systematic approach for the best results:

1.  **Tune Kp (Proportional Gain):**

      * In the code, set `Ki` and `Kd` to `0`.
      * Start with a small `Kp` value.
      * Upload the code and observe the robot. It will likely be slow and sluggish.
      * Gradually increase `Kp` and re-upload. You will notice the robot becomes more responsive.
      * Keep increasing `Kp` until the robot follows the line but oscillates (wobbles) back and forth aggressively. The ideal `Kp` is a value just *before* it becomes overly aggressive. [5]

2.  **Tune Kd (Derivative Gain):**

      * Now, keep your best `Kp` value.
      * Start with a small `Kd` value.
      * Gradually increase `Kd`. You should see the oscillations from the P-term start to reduce. The robot's movement will become smoother.
      * The goal is to find a `Kd` that eliminates the wobble without making the robot slow to react to curves. [5]

3.  **Tune Ki (Integral Gain):**

      * The robot should now be running well with `Kp` and `Kd`.
      * Observe it on a long, straight section of the track. If it consistently drives slightly to one side of the line (a steady-state error), you need the I-term.
      * Add a **very small** `Ki` value (e.g., 0.01). This should slowly correct the drift over time.
      * **Caution:** Be very careful with `Ki`. Too high a value will introduce new oscillations and make the system unstable. For many line followers, a `Ki` of 0 is sufficient. [6]

## Code Breakdown

  * **Global Variables:** Defines the pins for motors and sensors, and initializes the PID constants (`Kp`, `Ki`, `Kd`) and other necessary variables.
  * `setup()`: Initializes the motor pins as outputs and starts serial communication for debugging. It immediately calls the `calibration()` function.
  * `loop()`: The main program loop that continuously executes three main tasks:
    1.  `read_sensor_values()`: Reads the sensors and calculates the robot's current position and error.
    2.  `calculate_pid()`: Computes the PID correction value based on the error.
    3.  `motor_control()`: Adjusts the left and right motor speeds based on the PID value.
  * `calibration()`: Runs for 10 seconds at startup to determine the min/max readings for each IR sensor.
  * `read_sensor_values()`: Reads all 5 sensors, maps their values based on the calibration data, and calculates a single weighted position value.
  * `calculate_pid()`: Implements the core PID formula: `PID_value = (Kp * P) + (Ki * I) + (Kd * D)`.
  * `motor_control()`: Applies the correction to the base motor speed, constrains the values to be within the 0-255 PWM range, and calls the appropriate functions to drive the motors.
  * `motor_forward()`, `motor_backward()`, etc.: Low-level functions that set the `digitalWrite` and `analogWrite` values for the motor driver to control each motor's direction and speed.
