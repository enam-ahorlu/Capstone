# NFT Hydroponics System with ESP32 Microcontrollers

## Overview

This project uses two ESP32 microcontrollers to monitor and control a Nutrient Film Technique (NFT) hydroponics system. The system features multiple sensors for monitoring environmental conditions and actuators for automating the control of water flow, nutrient levels, and other critical parameters. The two ESP32 devices communicate via the ESP-NOW protocol, with the Master ESP32 responsible for data collection and decision-making, and the Slave ESP32 responsible for executing commands to control various actuators.

Each ESP32 is equipped with an LCD display to provide real-time feedback on the system’s status. Additionally, alert buzzers are included to notify users of critical conditions requiring immediate attention.

## Components

### Required Components
- **2 x ESP32 Development Boards**
- **2 x DHT11 Sensors** (for temperature and humidity measurement)
- **1 x pH Sensor (PH4502C)** (for monitoring the pH level)
- **1 x TDS Sensor (GravityTDS)** (for measuring the concentration of nutrients in the water)
- **2 x Flow Sensors** (for monitoring the nutrient solution’s flow rate)
- **3 x Ultrasonic Distance Sensors (HC-SR04)** (for measuring liquid levels and preventing overflow/underflow)
- **3 x Servo Motors** (for controlling nutrient/valve adjustments)
- **2 x 4-Channel Relay Modules** (for controlling high-power devices such as pumps and solenoid valves)
- **2 x DC Pumps** (for circulating the nutrient solution)
- **2 x Submersible Pumps** (for managing water flow, typically in the reservoir)
- **1 x Solenoid Valve** (for controlling the water supply to the system)
- **2 x Buzzers** (for audible alerts)
- **2 x 20x4 I2C LCD Displays** (for displaying sensor readings and system status)
- **Assorted Jumper Wires** (for connections)
- **Power Supply** (5V USB for ESP32s and appropriate power for the pumps and solenoid valves)

### Circuit Setup

#### Master ESP32 Connections
1. **DHT11 Sensor:**
   - **VCC**: Connect to 3.3V
   - **GND**: Connect to GND
   - **Data**: Connect to GPIO4

2. **pH Sensor (PH4502C):**
   - **pH Signal**: Connect to GPIO33 (ADC input)
   - **Temperature Signal**: Connect to GPIO34 (ADC input for temperature compensation)

3. **TDS Sensor (GravityTDS):**
   - **TDS Signal**: Connect to GPIO35 (ADC input)

4. **Flow Sensors:**
   - **Flow Sensor 1 (Inlet Flow):** Data to GPIO25
   - **Flow Sensor 2 (Outlet Flow):** Data to GPIO26

5. **Ultrasonic Sensors (HC-SR04):**
   - **Sensor 1 (Liquid Level Monitoring 1):**
     - **Trig**: Connect to GPIO19
     - **Echo**: Connect to GPIO18
   - **Sensor 2 (Liquid Level Monitoring 2):**
     - **Trig**: Connect to GPIO14
     - **Echo**: Connect to GPIO27
   - **Sensor 3 (Liquid Level Monitoring 3):**
     - **Trig**: Connect to GPIO23
     - **Echo**: Connect to GPIO13

6. **LCD Display (I2C):**
   - **SDA**: Connect to GPIO21
   - **SCL**: Connect to GPIO22

#### Slave ESP32 Connections
1. **Servo Motors:**
   - **Servo 1 (pH Lowering Agent Dispenser):** Connect to GPIO5
   - **Servo 2 (pH Raising Agent Dispenser):** Connect to GPIO17
   - **Servo 3 (Nutrient Dispenser):** Connect to GPIO16

2. **Relays:**
   - **Relay 1 (Connected to DC Pump for Nutrient Circulation):**
     - **IN1 (Signal)**: Connect to GPIO15
     - **VCC**: Connect to 5V
     - **GND**: Connect to GND
     - **COM**: Connect to the pump’s positive terminal
     - **NO (Normally Open)**: Connect to the power source for the pump
   - **Relay 2 (Connected to Solenoid Valve):**
     - **IN2 (Signal)**: Connect to GPIO2
     - **VCC**: Connect to 5V
     - **GND**: Connect to GND
     - **COM**: Connect to the valve’s positive terminal
     - **NO (Normally Open)**: Connect to the power source for the valve

3. **Buzzers:**
   - **Buzzer 1 (Flow Difference Alert):** Connect to GPIO32
   - **Buzzer 2 (High Temperature Alert):** Connect to GPIO33

4. **LCD Display (I2C):**
   - **SDA**: Connect to GPIO21
   - **SCL**: Connect to GPIO22

### Power Supply
- Both ESP32 devices should be powered via a 5V USB supply. Ensure that the power supply has a sufficient current rating to power all sensors, relays, and actuators connected to the ESP32 devices. High-power components like pumps and solenoid valves should have their own power supplies, with relays acting as switches controlled by the ESP32.

## Code Explanation

### Master ESP32 Code

The Master ESP32 is the brain of the system, responsible for sensor data acquisition, decision-making, and communication with the Slave ESP32. Below is a detailed breakdown of the code’s logic and functionality:

1. **Initialization:**
   - All sensors, the LCD display, and the ESP-NOW communication protocol are initialized. The Master is set to continuously monitor the system’s environment.

2. **Sensor Data Collection:**
   - **DHT11 Sensor:** Reads temperature and humidity.
   - **pH Sensor:** Monitors the pH level of the nutrient solution.
   - **TDS Sensor:** Measures the concentration of dissolved nutrients.
   - **Flow Sensors:** Measures the flow rates at the inlet and outlet of the NFT system.
   - **Ultrasonic Sensors:** Monitors the liquid levels to prevent overflow or underflow.

3. **Decision Making:**
   - The Master determines when to activate actuators based on sensor readings:
     - **pH Control:**
       - If pH < 5.5, the Master sends a command to the Slave to activate Servo 1 (dispensing a pH-lowering agent).
       - If pH > 8.5, the Master sends a command to the Slave to activate Servo 2 (dispensing a pH-raising agent).
     - **Flow Rate Discrepancy:**
       - If the difference between inlet and outlet flow rates exceeds 1.5, the Master sends a command to activate Relay 1 (turning on a DC pump) and Buzzer 1 (alerting of a flow issue).
     - **TDS Control:**
       - If TDS < 800 ppm, the Master sends a command to activate Servo 3 (dispensing nutrients).
     - **Temperature Alert:**
       - If temperature > 30°C, the Master sends a command to activate Buzzer 2 (audible temperature alert).

4. **LCD Display (Master):**
   - The Master’s LCD provides a real-time display of critical sensor readings and system statuses, including:
     - Current pH level
     - TDS value
     - Flow rates
     - Temperature and humidity
   - The LCD also indicates when an actuator is triggered, displaying messages such as “pH Low - Dispensing” or “Flow Alert - Pump On.”

5. **Command Sending:**
   - Based on the above logic, the Master sends corresponding commands to the Slave ESP32 via ESP-NOW. Commands control the activation of servos, relays, and buzzers.

### Slave ESP32 Code

The Slave ESP32 is responsible for executing the commands received from the Master ESP32. Below is a detailed explanation of how it works:

1. **Initialization:**
   - The Slave initializes communication with the Master and sets up the control of actuators (servo motors, relays, buzzers) and the LCD display.

2. **Command Reception:**
   - The Slave receives commands from the Master that dictate specific actions:
     - **Servo Motors:**
       - Command `A`: Activate Servo 1 (pH-lowering agent dispenser).
       - Command `B`: Activate Servo 2 (pH-raising agent dispenser).
       - Command `E`: Activate Servo 3 (nutrient dispenser).
     - **Relays:**
       - Command `C`: Activate Relay 1 to turn on the DC pump.
     - **Buzzers:**
       - Command `D`: Activate Buzzer 1 for a flow rate discrepancy alert.
       - Command `F`: Activate Buzzer 2 for a high-temperature alert.

3. **Actuator Control:**
   - Upon receiving a command, the Slave triggers the specified actuator. For example, if the Master detects a low pH, it sends Command `A` to the Slave, which then activates Servo 1 to dispense a pH-lowering solution.

4. **LCD Display (Slave):**
   - The Slave’s LCD provides feedback on the actions it performs, including:
     - Displaying which actuator is currently active (e.g., “pH Adjusting - Servo 1 Active”).
     - Showing status messages when commands are executed, such as “Nutrient Dispensing” or “Flow Alert - Pump Activated.”
     - Indicating when the Slave is idle and waiting for new commands.

5. **Buzzer Alerts:**
   - **Buzzer 1 (Flow Difference Alert):**
     - Triggered when the difference between the inlet and outlet flow rates exceeds 1.5. This alert is crucial for identifying potential blockages or leaks in the system. When activated, Buzzer 1 emits a repetitive sound pattern to draw attention to the issue.
     - The Master also sends a command to activate Relay 1, turning on a DC pump to mitigate the issue.
   - **Buzzer 2 (High Temperature Alert):**
     - Triggered when the temperature exceeds 30°C. This alert is vital to prevent overheating, which can affect the nutrient solution and the overall health of the plants. When activated, Buzzer 2 produces a continuous tone to warn the user.
     - The Master displays a warning message on its LCD, indicating that the temperature is too high.

## Wiring Diagram

### Master ESP32 Circuit Diagram

```
  +----------------+         +------------+        +------------+
  | Master ESP32   |         | Sensors    |        | Actuators  |
  +----------------+         +------------+        +------------+
  | GPIO4    <---> DHT11     |             |       |            |
  | GPIO33   <---> pH Sensor |             |       |            |
  | GPIO34   <---> pH Temp   |             |       |            |
  | GPIO35   <---> TDS Sensor|             |       |            |
  | GPIO25   <---> Flow 1    |             |       |            |
  | GPIO26   <---> Flow 2    |             |       |            |
  | GPIO19   <---> Trig 1    |             |       |            |
  | GPIO18   <---> Echo 1    |             |       |            |
  | GPIO14   <---> Trig 2    |             |       |            |
  | GPIO27   <---> Echo 2    |             |       |            |
  | GPIO23   <---> Trig 3    |             |       |            |
  | GPIO13   <---> Echo 3    |             |       |            |
  | GPIO21   <---> SDA (I2C) |             |       |            |
  | GPIO22   <---> SCL (I2C) |             |       |            |
  +----------------+         +------------+        +------------+
  | GPIO21   <---> LCD SDA   |             |       |            |
  | GPIO22   <---> LCD SCL   |             |       |            |
  +----------------+         +------------+        +------------+
```

### Slave ESP32 Circuit Diagram

```
  +----------------+         +------------+        +------------+
  | Slave ESP32    |         | Relays     |        | Actuators  |
  +----------------+         +------------+        +------------+
  | GPIO5    <---> Servo 1   | IN1    <--> DC Pump (Relay 1)    |
  | GPIO17   <---> Servo 2   | IN2    <--> Solenoid Valve (Relay 2)|
  | GPIO16   <---> Servo 3   | VCC    <--> 5V                    |
  | GPIO15   <---> Relay 1   | GND    <--> GND                   |
  | GPIO2    <---> Relay 2   | NO/NC  <--> Actuator Power Lines  |
  | GPIO32   <---> Buzzer 1  | COM    <--> Actuator              |
  | GPIO33   <---> Buzzer 2  |                                   |
  +----------------+         +------------+        +------------+
  | GPIO21   <---> LCD SDA   |             |       |            |
  | GPIO22   <---> LCD SCL   |             |       |            |
  +----------------+         +------------+        +------------+
```

## Operation Flow

1. **System Initialization:**
   - Power on both ESP32 devices. The Master begins monitoring all connected sensors, while the Slave waits for commands from the Master.

2. **Sensor Monitoring and Decision Making (Master):**
   - The Master reads data from all sensors and makes decisions based on the programmed logic. If any sensor reading exceeds or falls below its threshold, the Master sends a command to the Slave to activate the appropriate actuator.

3. **Actuator Control and Feedback (Slave):**
   - The Slave receives commands from the Master and activates the corresponding actuator, whether it’s adjusting pH, dispensing nutrients, or responding to a flow rate discrepancy. 
   - The Slave provides visual feedback on its LCD, showing which actuator is active and the current action being performed.

4. **System Alerts:**
   - Both buzzers provide audible alerts for critical conditions. Buzzer 1 warns of flow discrepancies, while Buzzer 2 signals when the temperature is too high. These alerts ensure that issues are promptly addressed to maintain the system’s optimal operation.

5. **System Feedback (LCD Displays):**
   - The Master’s LCD shows real-time sensor readings, system status, and any ongoing operations. For example, it displays current pH levels, TDS, flow rates, and temperature, along with messages indicating when an actuator is triggered.
   - The Slave’s LCD provides feedback on the actions it is performing, such as “pH Adjusting - Servo 1 Active” or “Flow Alert - Pump Activated.”

## Compiling, Installing, and Deploying the Code

### Required Libraries

Before compiling the code, ensure you have the following libraries installed in your Arduino IDE:

1. **ESP32 Board Package:**
   - Install the ESP32 board package in the Arduino IDE by going to `File > Preferences` and adding the following URL to the "Additional Board Manager URLs" field: `https://dl.espressif.com/dl/package_esp32_index.json`. Then, go to `Tools > Board > Boards Manager`, search for "ESP32," and install the package.

2. **ESP-NOW Library:**
   - The ESP-NOW protocol is included in the ESP32 core, so no additional installation is needed. Just ensure your ESP32 core is up to date.

3. **DHT Sensor Library:**
   - Install the "DHT sensor library" by Adafruit from the Library Manager (`Sketch > Include Library > Manage Libraries`). Search for "DHT sensor library" and install it.

4. **LiquidCrystal I2C Library:**
   - Install the "LiquidCrystal I2C" library by Frank de Brabander from the Library Manager (`Sketch > Include Library > Manage Libraries`). Search for "LiquidCrystal I2C" and install it.

5. **OneWire and DallasTemperature Libraries:**
   - Install the "OneWire" and "DallasTemperature" libraries from the Library Manager. These are needed for the TDS and temperature sensor interfaces.

6. **GravityTDS Library:**
   - You may need to manually install the "GravityTDS" library from the DFRobot GitHub repository. Download the library and add it to the Arduino IDE via `Sketch > Include Library > Add .ZIP Library`.

7. **pH4502C Sensor Library:**
   - This library might be specific to your sensor. If it’s a custom library, ensure it is included in the `libraries` folder of your Arduino setup.

### Steps for Compiling and Uploading

1. **Open the Arduino IDE:**
   - Ensure you have the latest version of the Arduino IDE installed.

2. **Select the ESP32 Board:**
   - Go to `Tools > Board > ESP32 Arduino > Your ESP32 Board Model` (e.g., "ESP32 Dev Module").

3. **Install Necessary Libraries:**
   - Use the Library Manager (`Sketch > Include Library > Manage Libraries`) to install all the required libraries listed above.

4. **Open the Code Files:**
   - Open the provided `esp_master_code.ino` and `esp_slave_code.ino` files in separate Arduino IDE windows.

5. **Compile and Upload the Master Code:**
   - Select the correct COM port for the Master ESP32 (`Tools > Port > COMx`) and click the Upload button. Wait for the code to compile and upload.

6. **Compile and Upload the Slave Code:**
   - Select the correct COM port for the Slave ESP32 (`Tools > Port > COMy`) and click the Upload button. Wait for the code to compile and upload.

7. **Verify Communication:**
   - Once both codes are uploaded, power the ESP32 devices and ensure they communicate successfully via ESP-NOW. The Master should start sending commands based on sensor data, and the Slave should respond accordingly.

### Deployment

1. **Connect the Sensors and Actuators:**
   - Ensure all sensors, relays, servos, buzzers, and displays are connected as per the circuit diagrams provided.

2. **Power Up the System:**
   - Power on both ESP32 devices and ensure they are connected to their respective components. Both LCDs should display system status information.

3. **Monitor and Adjust:**
   - Monitor the system via the LCDs. If needed, calibrate the sensors and adjust the threshold values in the code to fine-tune the system's response.

## Troubleshooting

1. **Communication Issues:**
   - Ensure the MAC addresses in the code match the actual MAC addresses of your ESP32 devices.
   - Verify that both ESP32 devices are within the communication range for ESP-NOW.

2. **Sensor Calibration:**
   - Calibrate the pH and TDS sensors according to the manufacturer's instructions before deploying the system.

3. **Power Supply:**
   - Ensure that the power supply provides sufficient current for all connected devices. Inadequate power can cause erratic behavior.

4. **Actuator Malfunction:**
   - Double-check the wiring connections for all actuators and ensure they are connected to the correct GPIO pins.

## Conclusion

This README provides a detailed overview of the NFT hydroponics system controlled by two ESP32 microcontrollers. By following the instructions and understanding the code provided, you can successfully set up and operate an automated hydroponics system that ensures optimal growing conditions for your plants. The system intelligently monitors environmental conditions and automates key aspects of plant care to ensure healthy growth.

If you have any questions or need further assistance, feel free to reach out.

---

