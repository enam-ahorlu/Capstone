#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHTesp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "GravityTDS.h"
#include <ph4502c_sensor.h>

// MAC address of the slave ESP32
uint8_t slaveMac[] = {0x10, 0x06, 0x1C, 0x87, 0x13, 0xB0}; // Replace with your slave ESP32 MAC address

// Global variable declarations
long globalDistance1;
long globalDistance2;
long globalDistance3;
float globalTemperatureDHT;
float globalHumidityDHT;
float globalWaterTemp;
float globalTDSValue;
float globalPHLevel;
float globalpHTemperature;

byte sensorInterrupt1 = 0;
byte sensorPin1 = 25; // Digital pin 25 for Sensor 1

byte sensorInterrupt2 = 1;
byte sensorPin2 = 26; // Digital pin 26 for Sensor 2

const float calibrationFactor = 4.5;
volatile byte pulseCounterSensor1 = 0;
volatile byte pulseCounterSensor2 = 0;
unsigned long oldTimeSensor1 = 0;
unsigned long oldTimeSensor2 = 0;
float globalFlowRateSensor1;
float globalFlowRateSensor2;

int lcdColumns = 20;
int lcdRows = 4;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 

#define DHTPIN 4 // GPIO4
//#define DHTTYPE DHT11
DHTesp dht;

#define ONE_WIRE_BUS 5 // GPIO5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define TdsSensorPin 35 // GPIO32 (ADC1_4)
GravityTDS gravityTds;

float temperature = 25, tdsValue = 0;

#define PH4502C_PH_LEVEL_PIN 33 // GPIO33 (ADC1_5)
#define PH4502C_TEMP_PIN 34 // GPIO34 (ADC1_6)

PH4502C_Sensor ph4502(PH4502C_PH_LEVEL_PIN, PH4502C_TEMP_PIN);

const int trigPin1 = 19; // GPIO19
const int echoPin1 = 18; // GPIO18
const int trigPin2 = 14; // GPIO21
const int echoPin2 = 27; // GPIO22
const int trigPin3 = 23; // GPIO23
const int echoPin3 = 13; // GPIO13

#define CMD_SERVO_LOW_PH  'A'
#define CMD_SERVO_HIGH_PH 'B'
#define CMD_RELAY_FLOW_DIFF 'C'
#define CMD_BUZZER_FLOW_DIFF 'D'
#define CMD_SERVO_LOW_TDS 'E'
#define CMD_BUZZER_HIGH_TEMP 'F'
#define CMD_SOL_VALVE 'G'
#define CMD_SOL_VALVE_2 'H'
#define CMD_RELAY_FLOW_DIFF_2 'I'

typedef struct struct_message {
  char command;
} struct_message;

struct_message myData;

unsigned long lastUpdate = 0; // Track the last update time for the loop
const unsigned long updateInterval = 2000; // Interval for updates (2 seconds)
unsigned long lastWaterTempUpdate = 0; // Track the last update time for water temperature

unsigned long lastPHUpdate = 0; // Track the last update time for pH
unsigned long lastTDSUpdate = 0; // Track the last update time for TDS

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  //esp_now_add_peer(slaveMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, slaveMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  dht.setup(DHTPIN, DHTesp::DHT11); // Initialize DHT sensor
  sensors.begin();
  ph4502.init();

  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(sensorPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin1), pulseCountSensor1, FALLING);

  pinMode(sensorPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin2), pulseCountSensor2, FALLING);
  oldTimeSensor1 = millis();
  oldTimeSensor2 = millis();

  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis; // Update the last update time

    // Update the DHT sensor values
    updateDHT11();

    // Update the water temperature from DS18B20
    readWaterTemperature();

    // Update flow rates
    updateFlowRates();

    // Update TDS value
    updateTDS();

    // Update pH sensor value
    updatePHSensor();

    // Update distances
    updateDistances();

    // Update the LCD
    if (globalDistance1 > 20) {
      lcd.setCursor(0, 0);
      lcd.print("Refill pH+    ");
    }

    if (globalDistance2 > 20) {
      lcd.setCursor(0, 1);
      lcd.print("Refill pH-    ");
    }

    if (globalDistance3 > 20) {
      lcd.setCursor(0, 2);
      lcd.print("Refill NPK    ");
    }

    if (globalWaterTemp > 30) {
      lcd.setCursor(0, 3);
      lcd.print("Check water temp");
    }

    checkAndSendCommands();
    lcd.clear();
  }
}

void checkAndSendCommands() {
  float flowDifference = abs(globalFlowRateSensor1 - globalFlowRateSensor2);

  if (globalPHLevel < 5.5) {
    sendCommand(CMD_SERVO_LOW_PH);
  } else if (globalPHLevel > 8.5) {
    sendCommand(CMD_SERVO_HIGH_PH);
  }

  if (flowDifference > 1.5) {
    sendCommand(CMD_RELAY_FLOW_DIFF);
    sendCommand(CMD_BUZZER_FLOW_DIFF);
  } else if (flowDifference < 1.5) {
    sendCommand(CMD_RELAY_FLOW_DIFF_2);
  }

  if (globalTDSValue < 800) {
    sendCommand(CMD_SERVO_LOW_TDS);
  }

  if (globalTemperatureDHT > 30) {
    sendCommand(CMD_BUZZER_HIGH_TEMP);
  }

  if (globalFlowRateSensor1 > 7) {
    sendCommand(CMD_SOL_VALVE);
  } else if (globalFlowRateSensor1 < 7) {
    sendCommand(CMD_SOL_VALVE_2);
  }
}

void sendCommand(char command) {
  myData.command = command;
  esp_now_send(slaveMac, (uint8_t *) &myData, sizeof(myData));
  Serial.print("Command sent: ");
  Serial.println(command);
}

void updateDHT11() {
  // globalTemperatureDHT = dht.readTemperature();
  // globalHumidityDHT = dht.readHumidity();

  TempAndHumidity newValues = dht.getTempAndHumidity();
  globalTemperatureDHT = newValues.temperature;
  globalHumidityDHT = newValues.humidity;

  if (!isnan(globalTemperatureDHT) && !isnan(globalHumidityDHT)) {
    Serial.print("Temperature: ");
    Serial.print(globalTemperatureDHT);
    Serial.println(" C");
    Serial.print("Humidity: ");
    Serial.print(globalHumidityDHT);
    Serial.println(" %");
  } else {
    Serial.println("DHT read error");
  }
}

void updateDistances() {
  globalDistance1 = getDistance(trigPin1, echoPin1);
  globalDistance2 = getDistance(trigPin2, echoPin2);
  globalDistance3 = getDistance(trigPin3, echoPin3);

  Serial.print("Distance 1: ");
  Serial.print(globalDistance1);
  Serial.println(" cm");
  Serial.print("Distance 2: ");
  Serial.print(globalDistance2);
  Serial.println(" cm");
  Serial.print("Distance 3: ");
  Serial.print(globalDistance3);
  Serial.println(" cm");
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void readWaterTemperature() {
  if (millis() - lastWaterTempUpdate >= 20000) { // Check if 10 seconds have passed
    globalWaterTemp = 22 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (35.0 - 22.0)));
    lastWaterTempUpdate = millis(); // Update the last update time
    Serial.println("Water Temperature: " + String(globalWaterTemp));
  }
}

void pulseCountSensor1() {
  pulseCounterSensor1++;
}

void pulseCountSensor2() {
  pulseCounterSensor2++;
}

void updateFlowRates() {
  if ((millis() - oldTimeSensor1) > 1000) {
    noInterrupts();
    globalFlowRateSensor1 = ((1000.0 / (millis() - oldTimeSensor1)) * pulseCounterSensor1) / calibrationFactor;
    oldTimeSensor1 = millis();
    pulseCounterSensor1 = 0;
    interrupts();
    Serial.print("Flow_rate_1: ");
    Serial.print(globalFlowRateSensor1);
    Serial.println(" L/m");
  }

  if ((millis() - oldTimeSensor2) > 1000) {
    noInterrupts();
    globalFlowRateSensor2 = ((1000.0 / (millis() - oldTimeSensor2)) * pulseCounterSensor2) / calibrationFactor;
    oldTimeSensor2 = millis();
    pulseCounterSensor2 = 0;
    interrupts();
    Serial.print("Flow_rate_2: ");
    Serial.print(globalFlowRateSensor2);
    Serial.println(" L/m");
}
}

void updateTDS() {
  if (millis() - lastTDSUpdate >= 20000) { // Check if 10 seconds have passed
    globalTDSValue = 750 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (850.0 - 750.0)));
    lastTDSUpdate = millis(); // Update the last update time
    Serial.println("TDS Value: " + String(globalTDSValue));
  }
}

void updatePHSensor() {
  if (millis() - lastPHUpdate >= 5000) { // Check if 5 seconds have passed
    globalPHLevel = 4.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (9.0 - 4.5)));
    lastPHUpdate = millis(); // Update the last update time
    Serial.println("pH Level: " + String(globalPHLevel));
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
Serial.print("\r\nLast Packet Send Status:\t");
Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
