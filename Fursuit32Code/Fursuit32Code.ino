#include <dummy.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Configurable pins ----------
const int PIN_1 = 25;
const int PIN_2 = 26;
const int PIN_3 = 27;  // was 35 — that pin is input-only, can't drive PWM
const int PIN_4 = 22;  // second motor pin, driven together with PIN_3 by the Fan slider
const int PINS[4] = { PIN_1, PIN_2, PIN_3, PIN_4 };

// ---------- Battery monitoring ----------
// Voltage divider (2x 100k) halves the LiPo voltage before it
// reaches the ADC pin, so real battery voltage = 2x what we read.
const int BATTERY_PIN = 35;
const float DIVIDER_RATIO = 2.0;      // R1=R2=100k -> divide by 2
const float BATTERY_EMPTY_V = 3.3;    // rough single-cell LiPo empty cutoff
const float BATTERY_FULL_V = 4.2;     // single-cell LiPo full charge
const unsigned long BATTERY_READ_INTERVAL_MS = 5000;  // how often to sample
unsigned long lastBatteryReadTime = 0;

// ---------- Duty ramping ----------
// BLE writes only set the *target* duty. The main loop steps the
// actual PWM output toward that target a little at a time instead
// of jumping instantly — this softens motor startup current so a
// slider flick to full speed doesn't yank a big current spike from
// the battery all at once.
volatile uint8_t targetDuty[4] = { 0, 0, 0, 0 };
uint8_t currentDuty[4] = { 0, 0, 0, 0 };

const unsigned long RAMP_INTERVAL_MS = 15;  // how often we take a step
const uint8_t RAMP_STEP = 3;                // duty change per step (lower = gentler/slower ramp)
unsigned long lastRampTime = 0;

// ---------- PWM (LEDC) config ----------
// Newer ESP32 Arduino core (3.x) addresses LEDC by pin number
// directly instead of by channel — no more manual channel index.
const int PWM_FREQ = 5000;       // 5 kHz, fine for LEDs/motors
const int PWM_RESOLUTION = 8;    // 8-bit -> duty range 0-255

// ---------- BLE UUIDs ----------
// Randomly generated — unique enough for a personal project.
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID_1 "12345678-1234-1234-1234-123456789ab1"
#define CHARACTERISTIC_UUID_2 "12345678-1234-1234-1234-123456789ab2"
#define CHARACTERISTIC_UUID_3 "12345678-1234-1234-1234-123456789ab3"
#define CHARACTERISTIC_UUID_4 "12345678-1234-1234-1234-123456789ab4"
#define CHARACTERISTIC_UUID_BATTERY "12345678-1234-1234-1234-123456789ab5"

BLEServer *pServer = nullptr;
BLECharacteristic *pChar1 = nullptr;
BLECharacteristic *pChar2 = nullptr;
BLECharacteristic *pChar3 = nullptr;
BLECharacteristic *pChar4 = nullptr;
BLECharacteristic *pCharBattery = nullptr;

bool deviceConnected = false;

// ---------- Server connect/disconnect callbacks ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    Serial.println("Client connected");
  }
  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    Serial.println("Client disconnected — restarting advertising");
    BLEDevice::startAdvertising();
  }
};

// ---------- Shared write handler ----------
// Reads a single byte from the characteristic and stores it as
// the *target* duty for the given pin index (0, 1, or 2) — the
// ramping loop in loop() is what actually moves the PWM output.
//
// NOTE: we read raw bytes via getData()/getLength() instead of
// getValue() as a String. Arduino's String treats a 0x00 byte
// as a terminator, so a written value of 0 would come back as
// a zero-length String and silently get skipped — that was the
// bug behind "setting the slider to 0 doesn't work".
void applyPwmFromCharacteristic(BLECharacteristic *characteristic, int index, const char *label) {
  uint8_t *data = characteristic->getData();
  size_t len = characteristic->getLength();
  if (len > 0 && data != nullptr) {
    targetDuty[index] = data[0];   // 0-255
    Serial.printf("%s -> target %d\n", label, targetDuty[index]);
  }
}

class Pin1Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, 0, "Pin 1 (GPIO25)");
  }
};
class Pin2Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, 1, "Pin 2 (GPIO26)");
  }
};
class Pin3Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, 2, "Pin 3 (GPIO27)");
  }
};
class Pin4Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, 3, "Pin 4 (GPIO22)");
  }
};

// ---------- Battery reading ----------
// analogReadMilliVolts() gives calibrated millivolts at the pin
// (accounting for the ESP32's ADC attenuation/calibration), which
// is more accurate than raw analogRead() counts. We then undo the
// voltage divider and map the result to a rough 0-100% estimate.
// Note: LiPo discharge isn't linear, so this is an approximation,
// not a precise fuel gauge — good enough for "roughly how full".
uint8_t readBatteryPercent() {
  uint32_t pinMilliVolts = analogReadMilliVolts(BATTERY_PIN);
  float batteryVolts = (pinMilliVolts / 1000.0) * DIVIDER_RATIO;

  float percent = (batteryVolts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100.0;
  percent = constrain(percent, 0.0, 100.0);

  Serial.printf("Battery debug -> pin: %u mV, calculated battery: %.2f V, percent: %.0f%%\n",
                pinMilliVolts, batteryVolts, percent);

  return (uint8_t)percent;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Fursuit32...");

  // ---- Set up PWM (LEDC) on the 4 pins ----
  ledcAttach(PIN_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_4, PWM_FREQ, PWM_RESOLUTION);

  // Start all pins off
  ledcWrite(PIN_1, 0);
  ledcWrite(PIN_2, 0);
  ledcWrite(PIN_3, 0);
  ledcWrite(PIN_4, 0);

  // ---- Set up BLE ----
  BLEDevice::init("Fursuit Kaasijs");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pChar1 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_1,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pChar1->setCallbacks(new Pin1Callback());
  pChar1->setValue("0");

  pChar2 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_2,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pChar2->setCallbacks(new Pin2Callback());
  pChar2->setValue("0");

  pChar3 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_3,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pChar3->setCallbacks(new Pin3Callback());
  pChar3->setValue("0");

  pChar4 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_4,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pChar4->setCallbacks(new Pin4Callback());
  pChar4->setValue("0");

  // Battery is read-only from the client's side — the page polls
  // it periodically rather than the ESP32 pushing updates.
  pCharBattery = pService->createCharacteristic(
      CHARACTERISTIC_UUID_BATTERY,
      BLECharacteristic::PROPERTY_READ);
  uint8_t initialBattery = readBatteryPercent();
  pCharBattery->setValue(&initialBattery, 1);

  pService->start();

  // Start advertising so phones can find it
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Fursuit32 BLE advertising started. Look for 'Fursuit Kaasijs' in your BLE scanner app.");
}

void loop() {
  unsigned long now = millis();

  if (now - lastRampTime >= RAMP_INTERVAL_MS) {
    lastRampTime = now;

    for (int i = 0; i < 4; i++) {
      uint8_t target = targetDuty[i];  // snapshot — BLE task could change it mid-loop
      if (currentDuty[i] == target) continue;

      if (currentDuty[i] < target) {
        currentDuty[i] = min((int)target, currentDuty[i] + RAMP_STEP);
      } else {
        currentDuty[i] = max((int)target, currentDuty[i] - RAMP_STEP);
      }
      ledcWrite(PINS[i], currentDuty[i]);
    }
  }

  if (now - lastBatteryReadTime >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadTime = now;
    uint8_t batteryPercent = readBatteryPercent();
    pCharBattery->setValue(&batteryPercent, 1);
  }
}
