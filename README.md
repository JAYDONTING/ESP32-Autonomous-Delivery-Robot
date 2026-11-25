
# ESP32 Autonomous Line-Following & Ball-Retrieval Robot

A fully autonomous mobile robot powered by an **ESP32-WROOM-32D**, designed to navigate a black-line track with dashed gaps and 90° junctions, avoid obstacles, identify target ping-pong ball colors (`RED`, `GREEN`, or `BLUE`), and retrieve and deliver the matching ball.

## Key Features
* **Closed-Loop PID Line Tracking:** Uses a 5-channel IR sensor array with weighted error calculation, adaptive curve slowdown, and lost-line memory for smooth cornering and dashed-line recovery.
* **Ultrasonic Obstacle Avoidance:** Non-blocking HC-SR04 distance sensing automatically halts the robot when an obstacle is detected on the main path and resumes once cleared.
* **Autonomous Color Sorting & Retrieval:** Uses a TCS34725 RGB sensor on a dedicated I2C bus to inspect balls at each branch and an SG90 servo gripper to pick up and deliver the selected target ball.
* **Onboard Target Selector UI:** Built-in OLED display (`SSD1306`) and `BOOT` button interface allowing target color selection (`RED` / `GREEN` / `BLUE`) at startup without external wireless controllers.

## Hardware Components
* **Microcontroller:** ESP32-WROOM-32D
* **Line Tracking:** 5-Channel IR Sensor Array
* **Color Detection:** TCS34725 RGB Color Sensor
* **Obstacle Detection:** HC-SR04 Ultrasonic Sensor
* **Display:** 0.96" I2C OLED Display (SSD1306)
* **Actuators:** TB6612FNG Dual Motor Driver + DC Gear Motors, SG90 Servo Gripper, Active Buzzer

## Required Arduino Libraries
* `Adafruit_GFX`
* `Adafruit_SSD1306`
* `Adafruit_TCS34725`
* `ESP32Servo`
