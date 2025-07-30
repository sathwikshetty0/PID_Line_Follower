## 1. **Hardware Requirements**

- **ESP32 Dev Board**
- **QTR-8RC Reflectance Sensor Array** (or compatible, 8 IR sensors)
- **Dual Motor Driver** (e.g., L298N or DRV8833)
- **2 DC Motors**
- Proper power supply (battery)
- Connect QTR array: pins 2, 4, 5, 18, 19, 21, 22, 23 (can be reconfigured)
- Motor driver to pins: PWMA(25), AIN1(32), AIN2(14), STBY(33), BIN1(15), BIN2(26), PWMB(27)

## 2. **Flashing & Setup**

1. **Install Arduino IDE** (or PlatformIO).
2. Add the **ESP32 board** (via Board Manager). Select your ESP32 device.
3. Install libraries:
   - `QTRSensors`
   - `ArduinoJson`
   - `WiFi`
   - `WebServer`
4. Copy the code into your project, configure your **WiFi SSID/Password**.
5. Connect all hardware as per the pin assignments.
6. Flash the code onto your ESP32. Open Serial Monitor for diagnostics.
7. Wait for ESP32 to join WiFi; note the printed **IP address**.

## 3. **Accessing the Dashboard**

- Connect your phone/PC to the **same WiFi network**.
- Open a web browser, visit: `http://`
- The **Dashboard UI** appears.

## 4. **Dashboard: Features & Functions**

### A. **System Status & Main Controls**
- **Status Bar:** Shows whether the system is running, stopped, manual mode, or needs calibration.
- **Main Buttons:**
  - **Calib:** Triggers the sensor calibration routine.
  - **Start:** Starts line following (PID) mode.
  - **Stop:** Stops all motors and actions.
  - **Auto Tune:** (Placeholder) - For future auto-tuning of PID parameters.

### B. **Manual Control**

- **Manual Mode Toggle:** Enable to manually drive the robot.
- **Arrow Key Buttons:** Control motors (forward, backward, left, right) using on-screen arrows. Use touch, mouse, or keyboard.
- **Live Motor Output:** Shows live set speeds for both motors.
- **Motors return to STOP if you release the button or toggle manual mode off.**

### C. **Calibration Data Visualization**

- **Sensor Min:** Shows the lowest value seen by each sensor during calibration.
- **Sensor Max:** Shows the highest value seen by each sensor.
- **Sensor Thresholds:** The calculated threshold between min and max for each sensor.

### D. **Custom Sensor Thresholds**

- Input fields for each sensor let you **manually set thresholds**.
- Use **Update Thresholds** to apply changes. This can help adapt to new surfaces or sensor drift.

### E. **Sensor Data & Visualization**

- **Sensor Bars:** Live visualization (active/inactive) for all sensor readings.
- **Raw Values:** Numeric readout of all sensors.
- **Thresholds Line:** The active threshold per sensor (for reference).
- **Position/Error:** Shows current line-follow position and error from center.

### F. **Motor Direction Visualization**

- **Motor A/B Direction:** Large arrows or neutral blocks visually indicate the current direction each motor is being commanded.

### G. **Sensor Heatmap**

- **Live activity map:** Colored bars for each sensor indicate how often each is triggered (relative activity).
- This helps to debug sensor coverage and robot centering.

### H. **Line Path Replay**

- **Canvas Plot:** Shows the robot’s detected path over time (last positions).
- **Reset Replay:** Refresh for a new session (does not affect data buffers).

### I. **PID Control Parameter Tuning**

- **Input fields:** Adjust Kp, Ki, Kd (PID constants), Base Speed, and Max Speed.
- **Set Button:** Applies changes instantly (no need to reboot).

### J. **Performance and Speed Graphs**

- **Performance Graph:** Line chart (using Chart.js) displays:
  - Position
  - Error
  - PID Output—over recent time.
- **Speed vs. Acceleration:** Another chart maps current estimated values for quick diagnostics.

## 5. **API Endpoints**

Advanced users can access the **following endpoints**:

- `/api/calibrate` (**POST**): Starts calibration.
- `/api/start` (**POST**): Starts line following.
- `/api/stop` (**POST**): Stops robot.
- `/api/pid` (**POST**): Update PID params as JSON.
- `/api/autotune` (**POST**): Starts auto-tune (stub/placeholder).
- `/api/status` (**GET**): Current system status.
- `/api/sensors` (**GET**): Live sensor readings.
- `/api/manual` (**POST**): Set manual speeds via JSON (`motorA`, `motorB`).
- `/api/manualmode` (**POST**): Enable/disable manual mode.
- `/api/calibrationdata` (**GET**): Returns min/max/thresholds.
- `/api/thresholds` (**GET/POST**): Get or set sensor thresholds.
- `/api/heatmap` (**GET**): Sensor activity heatmap.
- `/api/data` (**GET**): Recent data log (position, error, PID output).

## 6. **Best Practice Usage Workflow**

1. **Power the robot, connect to WiFi.**
2. **Open the dashboard.**
3. **Press Calib**: (Robot should be moved to expose sensors to both black line and white background.)
4. Verify thresholds are reasonable (**Calibration Data**).
5. Use **Manual Mode** to check sensor bars/heatmap; adjust thresholds if needed.
6. **Start** robot—watch live plots and path replay.
7. **Tune PID** using performance graphs and controls.
8. Use **Stop** always before handling/moving robot.
9. (Optional: Enhance `Auto Tune` as per your needs.)

## 7. **Customization & Extensibility**

- You may edit pin assignments, threshold logic, or extend `/api/autotune`.
- The **dashboard HTML** (see `getMainHTML()`) can be restyled or expanded.
- All frontend code is embedded and loads instantly from the ESP32 (no external hosting needed).

## 8. **Troubleshooting & Tips**

- If dashboard does not load, ensure you have the correct IP and WiFi.
- If calibration or sensor values do not seem correct, check IR sensors and wiring.
- If motors run backward, swap wiring or invert pin assignment.
- For laggy plots, try reducing the number of sample points or UI polling rate.
