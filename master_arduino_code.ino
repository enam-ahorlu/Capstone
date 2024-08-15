#include <Wire.h> // For I2C
#include <LiquidCrystal_I2C.h> // For LCD
#include <OneWire.h> // For Dallas temperature sensors
#include <DallasTemperature.h>
#include <DHT.h> // Include DHT library for humidity and temperature
#include <ph4502c_sensor.h>


#define TdsSensorPin A1
#define VREF 5.0
#define SCOUNT 30
#define DHT11_PIN 7
#define ONE_WIRE_BUS 2
#define DHTTYPE DHT11 // DHT Type

DHT dht(DHT11_PIN, DHTTYPE); // Initialize DHT sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);

// Ultrasonic Sensor Pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 11;
const int echoPin2 = 12;
const int trigPin3 = 13;
const int echoPin3 = 14;

// Variables for distance measurements
long duration1, duration2, duration3;
int distance1, distance2, distance3;

byte sensorInterrupt = 0;
byte sensorPin = 2;
float calibrationFactor = 4.5;

volatile byte pulseCount;  
float flowRate;
unsigned long totalMilliLitres;
unsigned long oldTime;

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

YourPHSensor phSensor; // Assume "YourPHSensor" is the class provided by your pH sensor library

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  ph4502c.init();

  sensors.begin(); // Start the DallasTemperature library
  dht.begin(); // Start the DHT sensor

      // Initialize ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  lcd.begin(20, 4);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.print("System Initializing...");
  lcd.setCursor(0, 1);

  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);

  Wire.begin(); // Join I2C bus as master
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - oldTime >= 1000) {
    noInterrupts();
    flowRate = calculateFlowRate();
    interrupts();
    totalMilliLitres += (flowRate / 60) * 1000;
    oldTime = currentMillis;
  }

  if (currentMillis % 40 == 0) {
    readTDS();
  }

  if (currentMillis % 3000 == 0) {
    readDHT();
    readWaterTemperature();
    float pHValue = phSensor.readPH(); // Adjust method name based on actual library
    lcd.setCursor(0, 2);
    lcd.print("pH: ");
    lcd.print(pHValue);
  }

  measureDistance(trigPin1, echoPin1, duration1, distance1);
  measureDistance(trigPin2, echoPin2, duration2, distance2);
  measureDistance(trigPin3, echoPin3, duration3, distance3);

  if (distance1 < 20) {  // Threshold distance
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(1);  // Send signal to turn on LED
    Wire.endTransmission();
  } else {
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0);  // Send signal to turn off LED
    Wire.endTransmission();
  }

  float pHValue = ph4502c.read_ph_level();
}

void readTDS() {
  analogBuffer[analogBufferIndex++] = analogRead(TdsSensorPin);
  if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;

  int sortedBuffer[SCOUNT];
  memcpy(sortedBuffer, analogBuffer, SCOUNT * sizeof(int));
  averageVoltage = getMedianNum(sortedBuffer, SCOUNT) * (VREF / 1024.0);
  tdsValue = calculateTDS(averageVoltage);
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  lcd.setCursor(0, 3);
  lcd.print("Hum: ");
  lcd.print(h);
  lcd.print(" Temp: ");
  lcd.print(t);
}

void readWaterTemperature() {
  sensors.requestTemperatures();
  float waterTemp = sensors.getTempCByIndex(0);
  lcd.setCursor(0, 4);
  lcd.print("Water Temp: ");
  lcd.print(waterTemp);
}

float calculateFlowRate() {
  return (1000.0 / (millis() - oldTime)) * pulseCount / calibrationFactor;
}

float calculateTDS(float voltage) {
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  return (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
            - 255.86 * compensationVoltage * compensationVoltage
            + 857.39 * compensationVoltage) * 0.5;
}

int getMedianNum(int bArray[], int iFilterLen) {
  sort(bArray, bArray + iFilterLen); // Use std::sort for simplicity
  int middle = iFilterLen / 2;
  return iFilterLen % 2 != 0 ? bArray[middle] : (bArray[middle - 1] + bArray[middle]) / 2;
}

void pulseCounter() {
  pulseCount++;
}

void measureDistance(int trigPin, int echoPin, long &duration, int &distance) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    // Example: Output the distance to the LCD
    lcd.setCursor(0, 2); // Adjust position as needed
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");
}
