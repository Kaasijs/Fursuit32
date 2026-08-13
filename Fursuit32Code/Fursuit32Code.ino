#include <dummy.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Configurable pins ----------
const int PIN_1 = 25;
const int PIN_2 = 26;
const int PIN_3 = 35;

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

BLEServer *pServer = nullptr;
BLECharacteristic *pChar1 = nullptr;
BLECharacteristic *pChar2 = nullptr;
BLECharacteristic *pChar3 = nullptr;

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
// Reads a single byte from the characteristic and applies it
// as the PWM duty cycle on the given pin.
void applyPwmFromCharacteristic(BLECharacteristic *characteristic, int pin, const char *label) {
  String value = characteristic->getValue();
  if (value.length() > 0) {
    uint8_t duty = (uint8_t)value[0];   // 0-255
    ledcWrite(pin, duty);
    Serial.printf("%s -> duty %d\n", label, duty);
  }
}

class Pin1Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, PIN_1, "Pin 1 (GPIO25)");
  }
};
class Pin2Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, PIN_2, "Pin 2 (GPIO26)");
  }
};
class Pin3Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    applyPwmFromCharacteristic(characteristic, PIN_3, "Pin 3 (GPIO27)");
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Fursuit32...");

  // ---- Set up PWM (LEDC) on the 3 pins ----
  ledcAttach(PIN_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_3, PWM_FREQ, PWM_RESOLUTION);

  // Start all pins off
  ledcWrite(PIN_1, 0);
  ledcWrite(PIN_2, 0);
  ledcWrite(PIN_3, 0);

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

  pService->start();

  // Start advertising so phones can find it
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Fursuit32 BLE advertising started. Look for 'Fursuit Kaasijs' in your BLE scanner app.");
}

void loop() {
  delay(1000);
}
