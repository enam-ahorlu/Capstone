
// ### Slave ESP32 Code (ESP-WROOM-32)
// ```cpp
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

#define BUZZER1_PIN 27 // GPIO27
#define BUZZER2_PIN 14 // GPIO14

int lcdColumns = 20;
int lcdRows = 4;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

#define CMD_SERVO_LOW_PH  'A'
#define CMD_SERVO_HIGH_PH 'B'
#define CMD_RELAY_FLOW_DIFF 'C'
#define CMD_BUZZER_FLOW_DIFF 'D'
#define CMD_SERVO_LOW_TDS 'E'
#define CMD_BUZZER_HIGH_TEMP 'F'
#define CMD_SOL_VALVE 'G'
#define CMD_SOL_VALVE_2 'H'
#define CMD_RELAY_FLOW_DIFF_2 'I'

const int SPrelayIN1 = 16; // GPIO16
const int SPrelayIN2 = 17; // GPIO17
const int SPrelayIN3 = 18; // GPIO18
const int SPrelayIN4 = 19; // GPIO19
const int DCrelayIN1 = 25; // GPIO25
const int DCrelayIN2 = 26; // GPIO26
const int SVrelayIN1_1 = 32; // GPIO32
const int SVrelayIN1_2 = 33; // GPIO33

Servo Servo1;
Servo Servo2;
Servo Servo3;

unsigned long servoTimer1 = 0;
bool servoActive1 = false;

unsigned long servoTimer2 = 0;
bool servoActive2 = false;

unsigned long servoTimer3 = 0;
bool servoActive3 = false;

unsigned long buzzerTimer1 = 0;
int buzzerCount1 = 0;
bool buzzerState1 = LOW;

unsigned long buzzerTimer2 = 0;
bool buzzerActive2 = false;

typedef struct struct_message {
  char command;
} struct_message;

struct_message myData;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);


  pinMode(SPrelayIN1, OUTPUT);
  pinMode(SPrelayIN2, OUTPUT);
  pinMode(SPrelayIN3, OUTPUT);
  pinMode(SPrelayIN4, OUTPUT);
  pinMode(DCrelayIN1, OUTPUT);
  pinMode(DCrelayIN2, OUTPUT);
  pinMode(SVrelayIN1_1, OUTPUT);
  pinMode(SVrelayIN1_2, OUTPUT);

  pinMode(BUZZER1_PIN, OUTPUT);
  pinMode(BUZZER2_PIN, OUTPUT);

  servoTimer1 = servoTimer2 = servoTimer3 = buzzerTimer1 = buzzerTimer2 = millis();
  servoActive1 = servoActive2 = servoActive3 = buzzerActive2 = false;
  buzzerState1 = LOW;
  buzzerCount1 = 0;

  Servo1.attach(13); // GPIO13
  Servo2.attach(12); // GPIO12
  Servo3.attach(14); // GPIO14

  turnOffPump1();
  turnOffPump2();
  turnOffDCPump();
  turnOffValve();

  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  // Main loop
}

void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *data, int len) {
  // Copy data to struct
  memcpy(&myData, data, sizeof(myData));
  performAction(myData.command);
}

void performAction(char command) {
  switch (command) {
    case CMD_SERVO_LOW_PH:
      activateservo1();
      break;
    case CMD_SERVO_HIGH_PH:
      activateservo2();
      break;
    case CMD_RELAY_FLOW_DIFF:
      turnOnDCPump();
      break;
    case CMD_BUZZER_FLOW_DIFF:
      activatebuzzer1();
      break;
    case CMD_SERVO_LOW_TDS:
      activateservo3();
      break;
    case CMD_BUZZER_HIGH_TEMP:
      activatebuzzer2();
      break;
    case CMD_SOL_VALVE:
      turnOnValve();
      break;
    case CMD_SOL_VALVE_2:
      turnOffValve();
      break;
    case CMD_RELAY_FLOW_DIFF_2:
      turnOffDCPump();
      break;
    default:
      Serial.println("Unknown command received");
      break;
  }
  Serial.print("Action performed for command: ");
  Serial.println(command);
}

void turnOnPump1() {
  digitalWrite(SPrelayIN1, LOW);
  digitalWrite(SPrelayIN2, HIGH);
  Serial.print("Pump1 turned on");
}

void turnOffPump1() {
  digitalWrite(SPrelayIN1, LOW);
  digitalWrite(SPrelayIN2, LOW);
  Serial.print("Pump1 turned off");
}

void turnOnPump2() {
  digitalWrite(SPrelayIN3, LOW);
  digitalWrite(SPrelayIN4, HIGH);
  Serial.print("Pump2 turned on");
}

void turnOffPump2() {
  digitalWrite(SPrelayIN3, LOW);
  digitalWrite(SPrelayIN4, LOW);
  Serial.print("Pump2 turned off");
}

void turnOnDCPump() {
  digitalWrite(DCrelayIN1, LOW);
  digitalWrite(DCrelayIN2, HIGH);
  Serial.print("DCPump turned on");
  lcd.setCursor(0, 0);
  lcd.print("Aux pump on");
}

void turnOffDCPump() {
  digitalWrite(DCrelayIN1, HIGH);
  digitalWrite(DCrelayIN2, HIGH);
  Serial.print("DCPump turned off");
  clearLine(0);
}

void turnOnValve() {
  digitalWrite(SVrelayIN1_1, HIGH);
  digitalWrite(SVrelayIN1_2, LOW);
  Serial.print("Solenoid Valve turned on");
  lcd.setCursor(0, 1);
  lcd.print("Solenoid on");
}

void turnOffValve() {
  digitalWrite(SVrelayIN1_1, LOW);
  digitalWrite(SVrelayIN1_2, LOW);
  Serial.print("Solenoid Valve turned off");
  clearLine(1);
}

void activateservo1() {
  unsigned long currentMillis = millis();

  if (!servoActive1) {
    Servo1.write(90);
    servoTimer1 = currentMillis;
    servoActive1 = true;
    Serial.print("Servo 1 active");
    lcd.setCursor(0, 3);
    lcd.print("pH- open");    
  } else if (servoActive1 && (currentMillis - servoTimer1 >= 1800)) {
    Servo1.write(0);
    servoActive1 = false;
    Serial.print("Servo 1 inactive");
    clearLine(3);
  }
}

void activateservo2() {
  unsigned long currentMillis = millis();

  if (!servoActive2) {
    Servo2.write(90);
    servoTimer2 = currentMillis;
    servoActive2 = true;
    Serial.print("Servo 2 active");
    lcd.setCursor(0, 3);
    lcd.print("pH+ open");
  } else if (servoActive2 && (currentMillis - servoTimer2 >= 1800)) {
    Servo2.write(0);
    servoActive2 = false;
    Serial.print("Servo 2 inactive");
    clearLine(3);
  }
}

void activateservo3() {
  unsigned long currentMillis = millis();

  if (!servoActive3) {
    Servo3.write(90);
    servoTimer3 = currentMillis;
    servoActive3 = true;
    Serial.print("Servo 3 active");
    lcd.setCursor(0, 2);
    lcd.print("NPK open");
  } else if (servoActive3 && (currentMillis - servoTimer3 >= 1800)) {
    Servo3.write(0);
    servoActive3 = false;
    Serial.print("Servo 3 inactive");
    clearLine(2);
  }
}

void activatebuzzer1() {
  unsigned long currentMillis = millis();

  if (buzzerCount1 < 10) {
    if (buzzerState1 == LOW && (currentMillis - buzzerTimer1 >= 100)) {
      digitalWrite(BUZZER1_PIN, HIGH);
      buzzerState1 = HIGH;
      buzzerTimer1 = currentMillis;
      Serial.print("buzzer 1 active");
    } else if (buzzerState1 == HIGH && (currentMillis - buzzerTimer1 >= 100)) {
      digitalWrite(BUZZER1_PIN, LOW);
      buzzerState1 = LOW;
      buzzerTimer1 = currentMillis;
      Serial.print("buzzer 1 active");
      buzzerCount1++;
    }
  }
}

void activatebuzzer2() {
  unsigned long currentMillis = millis();

  if (!buzzerActive2) {
    digitalWrite(BUZZER2_PIN, HIGH);
    buzzerTimer2 = currentMillis;
    buzzerActive2 = true;
  } else if (buzzerActive2 && (currentMillis - buzzerTimer2 >= 200)) {
    digitalWrite(BUZZER2_PIN, LOW);
    buzzerActive2 = false;
    buzzerTimer2 = currentMillis;
  }
}

void clearLine(int line) {
  lcd.setCursor(0, line);
  for (int i = 0; i < 20; i++) {
    lcd.print(" ");
  }
  lcd.setCursor(0, line);
}
