/*
  Smart Fire Detection System
  ESP32 + DHT11 + MQ gas sensor + 16x2 I2C LCD + BLE

  Pins:
    DHT11 DATA -> GPIO 27
    MQ A0      -> GPIO 34 (D0 not used)
    LED        -> GPIO 26 (through resistor)
    Buzzer     -> GPIO 18
    LCD SDA    -> GPIO 21
    LCD SCL    -> GPIO 22 (I2C address 0x27)

  States:
    normal temp + no gas   -> LED off,  buzzer off,  SYSTEM NORMAL
    high temp   + no gas   -> LED blink, buzzer on,  HIGH TEMP
    normal temp + gas      -> LED on,   buzzer off,  GAS DETECTED
    high temp   + gas      -> LED blink, buzzer on,  FIRE

  If the DHT11 read fails we don't invent a temperature, and we
  never declare FIRE off a bad reading - gas detection still works
  on its own though.

  BLE just broadcasts the current state over GATT for the web app
  (Web Bluetooth) to read - it has no way to affect the LED, buzzer,
  LCD or the state machine. If nothing is connected, or it drops,
  the alarm logic keeps running exactly the same.

  Device name : Smart-Fire-ESP32
  Service UUID:        12345678-1234-1234-1234-1234567890ab
  Characteristic UUID: 12345678-1234-1234-1234-1234567890ac
  Characteristic: READ + NOTIFY (BLE2902 descriptor so a browser
  can subscribe to notifications)

  JSON sent ~once a second while connected:
    {
      "temperature": 29.0,
      "humidity": 82.3,
      "gas": 1420,
      "gasDetected": false,
      "temperatureHigh": false,
      "dhtValid": true,
      "state": "SYSTEM NORMAL",
      "alarm": false
    }
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---- pins ----
const uint8_t DHT_PIN    = 27;
const uint8_t MQ_PIN     = 34;
const uint8_t LED_PIN    = 26;
const uint8_t BUZZER_PIN = 18;
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

// ---- sensor settings ----
#define DHT_TYPE DHT11

const float TEMP_THRESHOLD = 30.0;

// raw ADC value, not a calibrated ppm reading
const int GAS_THRESHOLD = 1100;

// ---- timing (all non-blocking, millis based) ----
const unsigned long DHT_INTERVAL = 2000;
const unsigned long GAS_INTERVAL = 200;
const unsigned long LED_BLINK_INTERVAL = 500;
const unsigned long SERIAL_INTERVAL = 2000;
const unsigned long LCD_INTERVAL = 500;
const unsigned long BLE_INTERVAL = 1000;

// ---- BLE ----
#define BLE_DEVICE_NAME     "Smart-Fire-ESP32"
#define BLE_SERVICE_UUID    "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHAR_UUID       "12345678-1234-1234-1234-1234567890ac"

BLEServer* bleServer = nullptr;
BLECharacteristic* bleCharacteristic = nullptr;

volatile bool bleClientConnected = false;
unsigned long lastBLEUpdate = 0;

// ---- objects ----
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

enum SystemState {
  STATE_NORMAL,
  STATE_HIGH_TEMP,
  STATE_GAS_DETECTED,
  STATE_FIRE
};

// ---- globals ----
float temperature = NAN;
float humidity = NAN;
int gasValue = 0;

bool dhtValid = false;
bool gasDetected = false;

SystemState currentState = STATE_NORMAL;

unsigned long lastDHTRead = 0;
unsigned long lastGasRead = 0;
unsigned long lastLEDUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long lastLCDUpdate = 0;

bool ledState = false;

void readDHT();
void readGasSensor();
void determineSystemState();
void updateLED();
void updateBuzzer();
void updateLCD();
void printSerialStatus();
void setupBLE();
void updateBLE();
const char* getStateName(SystemState state);

class FireDetectionServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    delay(200); // give the stack a moment before we start advertising again
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  analogReadResolution(12);

  dht.begin();

  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FIRE SAFETY");
  lcd.setCursor(0, 1);
  lcd.print("SYSTEM READY");

  setupBLE();

  Serial.println();
  Serial.println("Smart Fire Detection System");
  Serial.println("ESP32 initialized");
  Serial.println("DHT11: GPIO 27");
  Serial.println("MQ: GPIO 34");
  Serial.println("LED: GPIO 26");
  Serial.println("Buzzer: GPIO 18");
  Serial.println("LCD: 0x27");
  Serial.print("Temp threshold: ");
  Serial.print(TEMP_THRESHOLD, 1);
  Serial.println(" C");
  Serial.print("Gas threshold (raw ADC): ");
  Serial.println(GAS_THRESHOLD);
  Serial.println("BLE: " BLE_DEVICE_NAME);

  // force an immediate first read/update instead of waiting out the intervals
  lastDHTRead = millis() - DHT_INTERVAL;
  lastGasRead = millis() - GAS_INTERVAL;
  lastLCDUpdate = millis() - LCD_INTERVAL;
  lastSerialUpdate = millis() - SERIAL_INTERVAL;
  lastBLEUpdate = millis() - BLE_INTERVAL;
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastDHTRead >= DHT_INTERVAL) {
    lastDHTRead = currentMillis;
    readDHT();
  }

  if (currentMillis - lastGasRead >= GAS_INTERVAL) {
    lastGasRead = currentMillis;
    readGasSensor();
  }

  // state decision doesn't care whether BLE is connected
  determineSystemState();

  updateLED();
  updateBuzzer();

  if (currentMillis - lastLCDUpdate >= LCD_INTERVAL) {
    lastLCDUpdate = currentMillis;
    updateLCD();
  }

  if (currentMillis - lastSerialUpdate >= SERIAL_INTERVAL) {
    lastSerialUpdate = currentMillis;
    printSerialStatus();
  }

  if (currentMillis - lastBLEUpdate >= BLE_INTERVAL) {
    lastBLEUpdate = currentMillis;
    updateBLE();
  }
}

void readDHT() {
  float newTemperature = dht.readTemperature();
  float newHumidity = dht.readHumidity();

  if (isnan(newTemperature) || isnan(newHumidity)) {
    dhtValid = false;
    Serial.println();
    Serial.println("DHT11 ERROR! Check wiring.");
  } else {
    temperature = newTemperature;
    humidity = newHumidity;
    dhtValid = true;
  }
}

void readGasSensor() {
  gasValue = analogRead(MQ_PIN);
  gasDetected = (gasValue >= GAS_THRESHOLD);
}

void determineSystemState() {
  // no valid temperature reading, so we can't call FIRE - gas alone still counts
  if (!dhtValid) {
    currentState = gasDetected ? STATE_GAS_DETECTED : STATE_NORMAL;
    return;
  }

  bool highTemperature = (temperature >= TEMP_THRESHOLD);

  if (highTemperature && gasDetected) {
    currentState = STATE_FIRE;
  } else if (highTemperature) {
    currentState = STATE_HIGH_TEMP;
  } else if (gasDetected) {
    currentState = STATE_GAS_DETECTED;
  } else {
    currentState = STATE_NORMAL;
  }
}

void updateLED() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case STATE_NORMAL:
      ledState = false;
      digitalWrite(LED_PIN, LOW);
      break;

    case STATE_GAS_DETECTED:
      ledState = true;
      digitalWrite(LED_PIN, HIGH);
      break;

    case STATE_HIGH_TEMP:
    case STATE_FIRE:
      if (currentMillis - lastLEDUpdate >= LED_BLINK_INTERVAL) {
        lastLEDUpdate = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;
  }
}

void updateBuzzer() {
  if (currentState == STATE_HIGH_TEMP || currentState == STATE_FIRE) {
    tone(BUZZER_PIN, 1000);
  } else {
    noTone(BUZZER_PIN);
  }
}

void updateLCD() {
  char line1[17];
  char line2[17];
  memset(line1, ' ', 16);
  memset(line2, ' ', 16);
  line1[16] = '\0';
  line2[16] = '\0';

  if (!dhtValid) {
    strcpy(line1, "DHT11 ERROR");
    strcpy(line2, "CHECK SENSOR");
  } else {
    snprintf(line1, sizeof(line1), "Temp:%4.1f C", temperature);

    switch (currentState) {
      case STATE_NORMAL:
        strcpy(line2, "SYSTEM NORMAL");
        break;
      case STATE_HIGH_TEMP:
        strcpy(line2, "HIGH TEMP!");
        break;
      case STATE_GAS_DETECTED:
        strcpy(line2, "GAS DETECTED");
        break;
      case STATE_FIRE:
        strcpy(line2, "!!! FIRE !!!");
        break;
    }
  }

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void printSerialStatus() {
  Serial.println();
  Serial.println("----------------------------------------");

  if (dhtValid) {
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" C");
    Serial.print("Humidity: ");
    Serial.print(humidity, 1);
    Serial.println(" %");
  } else {
    Serial.println("Temperature: ERROR");
    Serial.println("Humidity: ERROR");
  }

  Serial.print("Gas ADC: ");
  Serial.println(gasValue);
  Serial.print("Gas detected: ");
  Serial.println(gasDetected ? "YES" : "NO");
  Serial.print("System state: ");
  Serial.println(getStateName(currentState));

  Serial.print("LED: ");
  if (currentState == STATE_NORMAL) {
    Serial.println("OFF");
  } else if (currentState == STATE_GAS_DETECTED) {
    Serial.println("ON");
  } else {
    Serial.println("BLINKING");
  }

  Serial.print("Buzzer: ");
  Serial.println((currentState == STATE_HIGH_TEMP || currentState == STATE_FIRE) ? "ON" : "OFF");

  Serial.print("BLE: ");
  Serial.println(bleClientConnected ? "CONNECTED" : "DISCONNECTED");
  Serial.println("----------------------------------------");
}

const char* getStateName(SystemState state) {
  switch (state) {
    case STATE_NORMAL:       return "SYSTEM NORMAL";
    case STATE_HIGH_TEMP:    return "HIGH TEMP";
    case STATE_GAS_DETECTED: return "GAS DETECTED";
    case STATE_FIRE:          return "FIRE";
    default:                  return "UNKNOWN";
  }
}

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new FireDetectionServerCallbacks());

  BLEService* bleService = bleServer->createService(BLE_SERVICE_UUID);

  bleCharacteristic = bleService->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  // needed for a browser to subscribe to notifications
  bleCharacteristic->addDescriptor(new BLE2902());

  bleService->start();

  BLEAdvertising* bleAdvertising = BLEDevice::getAdvertising();
  bleAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  bleAdvertising->setScanResponse(true);
  bleAdvertising->setMinPreferred(0x06);
  bleAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
}

// builds the JSON payload from whatever the current state is - never
// touches currentState/LED/buzzer/LCD itself, just reports them.
// plain char buffer + snprintf instead of a JSON lib to avoid heap churn.
void updateBLE() {
  if (!bleClientConnected) {
    return;
  }

  char payload[192];

  if (dhtValid) {
    snprintf(
      payload, sizeof(payload),
      "{\"temperature\":%.1f,\"humidity\":%.1f,\"gas\":%d,"
      "\"gasDetected\":%s,\"temperatureHigh\":%s,\"dhtValid\":true,"
      "\"state\":\"%s\",\"alarm\":%s}",
      temperature,
      humidity,
      gasValue,
      gasDetected ? "true" : "false",
      (temperature >= TEMP_THRESHOLD) ? "true" : "false",
      getStateName(currentState),
      (currentState == STATE_FIRE || currentState == STATE_HIGH_TEMP) ? "true" : "false"
    );
  } else {
    // DHT invalid - report null temp/humidity rather than faking a value
    snprintf(
      payload, sizeof(payload),
      "{\"temperature\":null,\"humidity\":null,\"gas\":%d,"
      "\"gasDetected\":%s,\"temperatureHigh\":false,\"dhtValid\":false,"
      "\"state\":\"%s\",\"alarm\":%s}",
      gasValue,
      gasDetected ? "true" : "false",
      getStateName(currentState),
      (currentState == STATE_FIRE || currentState == STATE_HIGH_TEMP) ? "true" : "false"
    );
  }

  bleCharacteristic->setValue((uint8_t*)payload, strlen(payload));
  bleCharacteristic->notify();
}
