#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>  // built into the ESP32 core — no new library install needed

// ---------- Assignable pins ----------
// These are the GPIOs the web page's setup wizard lets you pick from
// and group into labeled sliders. This is a broad, generally-safe set
// of PWM-capable OUTPUT pins common to ESP32 dev boards (D1 mini
// clones included): it excludes GPIO0/1/2/3 (boot mode + UART),
// GPIO6-11 (wired internally to flash — do not use), and GPIO34-39
// (input-only, can't drive PWM — this is exactly the mistake that
// bit us with the old GPIO35 wiring).
//
// If your specific board doesn't physically break out one of these
// pins, that's fine — just don't select it in the wizard. Nothing
// bad happens; an unassigned pin just never receives a duty write.
const int ALLOWED_PINS[] = { 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 };
const int NUM_PINS = sizeof(ALLOWED_PINS) / sizeof(ALLOWED_PINS[0]);  // 18

// ---------- Battery monitoring ----------
// Fixed hardware sense pin — not part of the configurable pin pool,
// since it's not an output and shouldn't be reassignable.
// Voltage divider (2x 100k) halves the LiPo voltage before it
// reaches the ADC pin, so real battery voltage = 2x what we read.
const int BATTERY_PIN = 35;
const float DIVIDER_RATIO = 2.0;      // R1=R2=100k -> divide by 2
const float BATTERY_EMPTY_V = 3.3;    // rough single-cell LiPo empty cutoff
const float BATTERY_FULL_V = 4.2;     // single-cell LiPo full charge
const unsigned long BATTERY_READ_INTERVAL_MS = 5000;  // how often to sample
unsigned long lastBatteryReadTime = 0;

float smoothedBatteryPercent = -1;  // -1 = not yet initialized
const float BATTERY_SMOOTHING_ALPHA = 0.15;  // lower = smoother/slower to react

// ---------- Duty ramping ----------
// BLE writes only set the *target* duty. The main loop steps the
// actual PWM output toward that target a little at a time instead
// of jumping instantly — this softens motor startup current so a
// slider flick to full speed doesn't yank a big current spike from
// the battery all at once. Applied uniformly to all pins — harmless
// for LEDs, meaningfully protective for anything with a motor.
volatile uint8_t targetDuty[18] = { 0 };
uint8_t currentDuty[18] = { 0 };

const unsigned long RAMP_INTERVAL_MS = 15;  // how often we take a step
const uint8_t RAMP_STEP = 3;                // duty change per step (lower = gentler/slower ramp)
unsigned long lastRampTime = 0;

// ---------- PWM (LEDC) config ----------
const int PWM_FREQ = 5000;       // 5 kHz, fine for LEDs/motors
const int PWM_RESOLUTION = 8;    // 8-bit -> duty range 0-255

// ---------- Persistent storage ----------
// The 18 pins are always physically available — what's configurable
// is which of them get grouped into which labeled slider. That
// grouping is just an opaque JSON string as far as the ESP32 is
// concerned; we don't parse it, just store/return it, so no JSON
// library is needed here. Format (decided by the web page):
//   [{"label":"Ears","pins":[0,1]}, {"label":"Tail","pins":[9]}]
// where each pin index maps to ALLOWED_PINS[index]. An empty "[]"
// means nothing configured yet — that's what tells the page to show
// the first-time setup wizard.
Preferences prefs;
String savedChannelConfig = "[]";

// BLE advertised name doubles as the page's displayed title. Default
// the very first boot (nothing saved yet) is "Fursuit32".
String savedDeviceName = "Fursuit32";

// Icon/background image URLs — pure passthrough, same idea as the
// channel config. Format: {"icon":"<url or empty>","background":"<url or empty>"}
String savedSiteSettings = "{}";

// ---------- BLE UUIDs ----------
// One characteristic per assignable pin, generated as
// ...789a01 through ...789a12 (hex 1-18) to keep this file short
// instead of hand-writing 18 #defines. Fixed-purpose characteristics
// use a separate ...789bNN range so they can never collide.
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define BATTERY_UUID "12345678-1234-1234-1234-123456789b01"
#define CONFIG_UUID  "12345678-1234-1234-1234-123456789b02"
#define NAME_UUID    "12345678-1234-1234-1234-123456789b03"
#define SITE_UUID    "12345678-1234-1234-1234-123456789b04"

String pinCharUuid(int index) {
  char buf[6];
  snprintf(buf, sizeof(buf), "%02x", index + 1);  // 01..12 (hex)
  return "12345678-1234-1234-1234-123456789a" + String(buf);
}

BLEServer *pServer = nullptr;
BLECharacteristic *pPinChars[18];
BLECharacteristic *pCharBattery = nullptr;
BLECharacteristic *pCharConfig = nullptr;
BLECharacteristic *pCharName = nullptr;
BLECharacteristic *pCharSite = nullptr;

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

// ---------- Per-pin write handler ----------
// One instance per pin, each remembering which index it owns. Reads
// raw bytes via getData()/getLength() instead of getValue() as a
// String — Arduino's String treats a 0x00 byte as a terminator, so a
// written value of 0 would come back as a zero-length String and
// silently get skipped. Sets only the *target* duty; loop() ramps
// the actual PWM output toward it.
class PwmPinCallback : public BLECharacteristicCallbacks {
  public:
    PwmPinCallback(int idx) : index(idx) {}
    void onWrite(BLECharacteristic *characteristic) override {
      uint8_t *data = characteristic->getData();
      size_t len = characteristic->getLength();
      if (len > 0 && data != nullptr) {
        targetDuty[index] = data[0];
        Serial.printf("Pin idx %d (GPIO%d) -> target %d\n", index, ALLOWED_PINS[index], targetDuty[index]);
      }
    }
  private:
    int index;
};

// ---------- Battery reading ----------
// analogReadMilliVolts() gives calibrated millivolts at the pin
// (accounting for the ESP32's ADC attenuation/calibration), which
// is more accurate than raw analogRead() counts. We then undo the
// voltage divider and map the result to a rough 0-100% estimate.
// Note: LiPo discharge isn't linear, so this is an approximation,
// not a precise fuel gauge — good enough for "roughly how full".
//
// The raw reading also sags under load: pulling current for a
// motor/LEDs briefly drops the battery's terminal voltage due to
// its internal resistance, even though true charge hasn't changed.
// We smooth readings with an exponential moving average so a motor
// spin-up doesn't make the percentage visibly jump around.
uint8_t readBatteryPercent() {
  uint32_t pinMilliVolts = analogReadMilliVolts(BATTERY_PIN);
  float batteryVolts = (pinMilliVolts / 1000.0) * DIVIDER_RATIO;

  float rawPercent = (batteryVolts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100.0;
  rawPercent = constrain(rawPercent, 0.0, 100.0);

  if (smoothedBatteryPercent < 0) {
    smoothedBatteryPercent = rawPercent;
  } else {
    smoothedBatteryPercent += (rawPercent - smoothedBatteryPercent) * BATTERY_SMOOTHING_ALPHA;
  }

  Serial.printf("Battery debug -> pin: %u mV, raw: %.0f%%, smoothed: %.0f%%\n",
                pinMilliVolts, rawPercent, smoothedBatteryPercent);

  return (uint8_t)smoothedBatteryPercent;
}

// Saves whatever JSON the page writes here straight to flash, no
// parsing needed on this side. Also zeroes every pin's target duty
// as a safety measure — if the page is reassigning which pins mean
// what, we don't want a pin left spinning at a stale value under
// its old label.
class ConfigCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    uint8_t *data = characteristic->getData();
    size_t len = characteristic->getLength();
    if (data == nullptr) return;

    String newConfig = "";
    for (size_t i = 0; i < len; i++) newConfig += (char)data[i];

    savedChannelConfig = newConfig;
    prefs.putString("cfg", savedChannelConfig);

    for (int i = 0; i < NUM_PINS; i++) targetDuty[i] = 0;

    Serial.println("Config saved: " + savedChannelConfig);
  }
};

// Changing the BLE advertised name isn't something this BLE stack can
// do live — it's set once at BLEDevice::init() and stays fixed after
// that. The reliable way to actually re-advertise under a new name is
// to save it and reboot. The page knows this happens and tells the
// user to reconnect after a few seconds.
class NameCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    uint8_t *data = characteristic->getData();
    size_t len = characteristic->getLength();
    if (data == nullptr || len == 0) return;

    String newName = "";
    for (size_t i = 0; i < len; i++) newName += (char)data[i];

    prefs.putString("name", newName);
    Serial.println("Name changed to '" + newName + "' — restarting to re-advertise...");
    delay(300);  // give the BLE write response a moment to actually go out before we drop the connection
    ESP.restart();
  }
};

// Icon/background URLs — pure passthrough like ConfigCallback, no
// restart needed since these only affect how the page renders itself.
class SiteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    uint8_t *data = characteristic->getData();
    size_t len = characteristic->getLength();
    if (data == nullptr) return;

    String newSettings = "";
    for (size_t i = 0; i < len; i++) newSettings += (char)data[i];

    savedSiteSettings = newSettings;
    prefs.putString("site", savedSiteSettings);

    Serial.println("Site settings saved: " + savedSiteSettings);
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Fursuit32...");

  // ---- Load saved channel config, name, and site settings from flash ----
  prefs.begin("fursuit32", false);
  savedChannelConfig = prefs.getString("cfg", "[]");
  savedDeviceName = prefs.getString("name", "Fursuit32");
  savedSiteSettings = prefs.getString("site", "{}");
  Serial.println("Loaded config: " + savedChannelConfig);
  Serial.println("Loaded name: " + savedDeviceName);
  Serial.println("Loaded site settings: " + savedSiteSettings);

  // ---- Set up PWM (LEDC) on every assignable pin ----
  // All 18 get attached and zeroed regardless of whether they're
  // currently part of a configured slider — harmless, since nothing
  // draws current until its target duty is set above 0, which only
  // happens for pins the page actually writes to.
  for (int i = 0; i < NUM_PINS; i++) {
    ledcAttach(ALLOWED_PINS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(ALLOWED_PINS[i], 0);
  }

  // ---- Set up BLE ----
  BLEDevice::init(savedDeviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // With 22 characteristics on this service (18 pins + battery + config
  // + name + site), the default handle count from createService(uuid)
  // isn't nearly enough — each characteristic needs its own GATT
  // handles, and running out mid-setup breaks the service silently
  // (which looks like "can't connect" from the phone's side, since the
  // service never finishes registering correctly). 80 handles gives
  // comfortable headroom above the ~45 actually needed.
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 80);

  for (int i = 0; i < NUM_PINS; i++) {
    pPinChars[i] = pService->createCharacteristic(
        pinCharUuid(i),
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
    pPinChars[i]->setCallbacks(new PwmPinCallback(i));
    pPinChars[i]->setValue("0");
  }

  // Battery is read-only from the client's side — the page polls
  // it periodically rather than the ESP32 pushing updates.
  pCharBattery = pService->createCharacteristic(
      BATTERY_UUID,
      BLECharacteristic::PROPERTY_READ);
  uint8_t initialBattery = readBatteryPercent();
  pCharBattery->setValue(&initialBattery, 1);

  // Config is read (page fetches current setup on connect) and write
  // (page saves new/edited setup).
  pCharConfig = pService->createCharacteristic(
      CONFIG_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharConfig->setCallbacks(new ConfigCallback());
  pCharConfig->setValue((uint8_t *)savedChannelConfig.c_str(), savedChannelConfig.length());

  // Device name — read (page shows it as the title) and write (page
  // saves a new one). A write triggers a restart, see NameCallback.
  pCharName = pService->createCharacteristic(
      NAME_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharName->setCallbacks(new NameCallback());
  pCharName->setValue((uint8_t *)savedDeviceName.c_str(), savedDeviceName.length());

  // Site settings (icon/background URLs) — same opaque passthrough
  // pattern as channel config, no restart needed to apply.
  pCharSite = pService->createCharacteristic(
      SITE_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharSite->setCallbacks(new SiteCallback());
  pCharSite->setValue((uint8_t *)savedSiteSettings.c_str(), savedSiteSettings.length());

  pService->start();

  // Start advertising so phones can find it
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Fursuit32 BLE advertising started. Look for '" + savedDeviceName + "' in your BLE scanner app.");
}

void loop() {
  unsigned long now = millis();

  if (now - lastRampTime >= RAMP_INTERVAL_MS) {
    lastRampTime = now;

    for (int i = 0; i < NUM_PINS; i++) {
      uint8_t target = targetDuty[i];  // snapshot — BLE task could change it mid-loop
      if (currentDuty[i] == target) continue;

      if (currentDuty[i] < target) {
        currentDuty[i] = min((int)target, currentDuty[i] + RAMP_STEP);
      } else {
        currentDuty[i] = max((int)target, currentDuty[i] - RAMP_STEP);
      }
      ledcWrite(ALLOWED_PINS[i], currentDuty[i]);
    }
  }

  if (now - lastBatteryReadTime >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadTime = now;
    uint8_t batteryPercent = readBatteryPercent();
    pCharBattery->setValue(&batteryPercent, 1);
  }
}
