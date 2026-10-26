/*
  DrowningWearable.ino -- Arduino C++ framework, not a rescue controller.
  Board: ESP32-S3-DevKitC-1-N8R8; Arduino-ESP32 3.x; NimBLE-Arduino 2.x.

  Install: SparkFun BMI270 Arduino Library, BlueRobotics MS5837 Library,
           ESP32Servo, NimBLE-Arduino. Other headers ship with the ESP32 core.

  DEFAULTS: pin mapping UNCONFIRMED, flash writes OFF, alert outputs OFF.
  The servo is NEVER attached or moved. Detection and ECG interpretation are
  TODOs; SOS records a dry-run request only. No universal drowning thresholds.

  Example pin assignments below are proposals except for your servo GPIO16.
  Confirm wiring, set PINMAP_CONFIRMED, then inspect Serial at 115200 baud.
  Send 'z' while the pressure sensor is in AIR to capture a session baseline.
  No automatic filesystem formatting. BLE exposes status only; file download
  is an explicit TODO. See README.txt for build settings and limitations.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <MS5837.h>
#include <ESP32Servo.h>
#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>
#include <atomic>
#include <cmath>
#include <cstddef>

namespace Config {
constexpr bool PINMAP_CONFIRMED = false;
constexpr bool ENABLE_FLASH_LOGGING = false;
constexpr bool ENABLE_ALERT_OUTPUTS = false;
constexpr int IMU_SDA = 8, IMU_SCL = 9;
constexpr int DEPTH_SDA = 10, DEPTH_SCL = 11; // separate bus: blocking driver
constexpr int ECG_OUT = 1;                 // ADC1 on ESP32-S3
constexpr int ECG_LO_PLUS = 4, ECG_LO_MINUS = 5;
constexpr int VIBRATION_GATE = 12, STROBE_GATE = 13;
constexpr int CANCEL_BUTTON = 14, SOS_BUTTON = 15;
constexpr int SERVO_SIGNAL = 16;
constexpr uint8_t IMU_ADDRESS = 0x68;
constexpr uint8_t PRESSURE_ADDRESS = 0x76;
constexpr uint32_t ECG_HZ = 250, IMU_HZ = 100, PRESSURE_HZ = 20;
constexpr size_t HISTORY_SECONDS = 60;
constexpr size_t HISTORY_SAMPLES = HISTORY_SECONDS * (ECG_HZ + IMU_HZ + PRESSURE_HZ);
constexpr size_t FLASH_RESERVE_BYTES = 128 * 1024;
constexpr char DEVICE_NAME[] = "Wearable-Framework";
constexpr char SERVICE_UUID[] = "e81b6200-7e57-4d8e-b6f4-889c1d4e8270";
constexpr char STATUS_UUID[] = "e81b6201-7e57-4d8e-b6f4-889c1d4e8270";
constexpr char LOG_INFO_UUID[] = "e81b6202-7e57-4d8e-b6f4-889c1d4e8270";
}

static_assert(configTICK_RATE_HZ >= 250, "ECG task requires <=4 ms RTOS ticks");
static_assert(configTICK_RATE_HZ % 250 == 0, "ECG period must fit RTOS ticks");

enum class State : uint8_t { BOOT, MONITOR, SUSPECT, DEPLOYMENT_REQUESTED };
enum class Kind : uint8_t { ECG = 1, IMU = 2, PRESSURE = 3 };
enum SampleFlags : uint8_t {
  VALUE_PRESENT = 1, LEADS_CONNECTED = 2, DRIVER_UNVERIFIED = 4,
  CLIPPED = 8
};

union Payload {
  uint16_t ecgAdc;
  int16_t imu[6]; // acceleration: mg; angular velocity: 0.1 degrees/s
  struct { int32_t pressurePa; int16_t temperatureCentiC; } pressure;
};
struct RawSample {
  uint32_t timeMs; // millis since boot; wraps after ~49.7 days
  Kind kind;
  uint8_t flags;
  uint16_t reserved;
  Payload values;
};
static_assert(sizeof(RawSample) == 20, "Review raw format if layout changes");

// This EASY-TO-READ framework format differs from the earlier packed estimate:
// 370 records/s * 20 bytes = 7,400 B/s; 60 seconds = 444,000 bytes of PSRAM.
// TODO: pack stream blocks to approach the previously budgeted 1,844 B/s.
// Raw history stays in RAM here; event-file persistence is not implemented.

struct Snapshot {
  uint32_t ecgMs = 0, imuMs = 0, pressureMs = 0;
  uint16_t ecgAdc = 0;
  uint8_t ecgFlags = 0, imuFlags = 0, pressureFlags = 0;
  float ax = NAN, ay = NAN, az = NAN;
  float gx = NAN, gy = NAN, gz = NAN;
  float pressurePa = NAN, temperatureC = NAN;
  float surfacePa = NAN, depthM = NAN;
};
struct SummaryRecord { // little-endian, IEEE754 floats; 32 bytes per second
  uint32_t timeMs;
  float depthM, pressurePa, heartRateBpm;
  uint32_t droppedSamples, qualityMask;
  uint16_t ecgAdc;
  uint8_t state, formatVersion;
  uint32_t crc32;
};
static_assert(sizeof(SummaryRecord) == 32, "Summary record must remain 32 bytes");
struct BleStatus { // <=20 bytes: works with default ATT payload size
  uint32_t timeMs;
  uint8_t state, flags;
  uint16_t heartRateBpm; // 0xffff = unknown, never a measured zero
  int32_t depthMm;       // INT32_MIN = unknown
  uint32_t droppedSamples, historySamples;
};
static_assert(sizeof(BleStatus) == 20, "BLE status requires 20 bytes");

BMI270 imu;
MS5837 pressureSensor;
Servo valveServo; // deliberately never attach(): no servo pulses in this sample
Preferences preferences;
NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* statusCharacteristic = nullptr;
QueueHandle_t sampleQueue = nullptr;
RawSample* history = nullptr;
size_t historyWriteIndex = 0; // storageTask owns all history writes
std::atomic<uint32_t> historyCount{0}, droppedSamples{0};
std::atomic<State> state{State::BOOT};
std::atomic<bool> calibrationRequested{false}, storageFault{false}, taskFault{false};
Snapshot latest;
portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
bool imuReady = false, pressureReady = false, filesystemReady = false;
float fluidDensity = 997.0f;
char logPath[48] = {};

// Explicit declarations avoid Arduino-generated prototype/custom-type issues.
Snapshot getSnapshot();
void enqueueSample(const RawSample& sample);
uint32_t qualityMask(const Snapshot& sample, uint32_t now);
bool evaluateDistressPlaceholder(const Snapshot& sample);
void requestDeploymentDryRun(const char* reason);
SummaryRecord makeSummary(uint32_t now);

Snapshot getSnapshot() {
  portENTER_CRITICAL(&snapshotMux);
  Snapshot copy = latest;
  portEXIT_CRITICAL(&snapshotMux);
  return copy;
}

void enqueueSample(const RawSample& sample) {
  // Acquisition never waits for a filesystem or BLE operation.
  if (xQueueSend(sampleQueue, &sample, 0) != pdPASS) ++droppedSamples;
}

uint32_t qualityMask(const Snapshot& sample, uint32_t now) {
  uint32_t flags = 0;
  bool freshEcg = (sample.ecgFlags & VALUE_PRESENT) && now - sample.ecgMs < 100;
  bool freshPressure = (sample.pressureFlags & VALUE_PRESENT) && now - sample.pressureMs < 250;
  if (freshEcg) flags |= 1;
  if (freshEcg && (sample.ecgFlags & LEADS_CONNECTED)) flags |= 2;
  if ((sample.imuFlags & VALUE_PRESENT) && now - sample.imuMs < 100) flags |= 4;
  if (freshPressure) flags |= 8;
  if (freshPressure && std::isfinite(sample.depthM)) flags |= 16;
  if (storageFault.load()) flags |= 32;
  if (taskFault.load()) flags |= 64;
  // Bit7 expressly means transport integrity is NOT established by this driver.
  if (sample.pressureFlags & DRIVER_UNVERIFIED) flags |= 128;
  return flags;
}

int16_t quantize(float value, float scale, uint8_t& flags) {
  float scaled = value * scale;
  if (scaled < -32768.0f) { scaled = -32768.0f; flags |= CLIPPED; }
  if (scaled > 32767.0f) { scaled = 32767.0f; flags |= CLIPPED; }
  return static_cast<int16_t>(lroundf(scaled));
}

void ecgTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    RawSample frame{};
    frame.kind = Kind::ECG;
    frame.timeMs = millis();
    if (Config::PINMAP_CONFIRMED) {
      frame.values.ecgAdc = analogRead(Config::ECG_OUT);
      frame.flags = VALUE_PRESENT;
      if (!digitalRead(Config::ECG_LO_PLUS) && !digitalRead(Config::ECG_LO_MINUS))
        frame.flags |= LEADS_CONNECTED;
      // Lead contact does not establish ECG waveform quality.
    }
    portENTER_CRITICAL(&snapshotMux);
    latest.ecgMs = frame.timeMs;
    latest.ecgAdc = frame.values.ecgAdc;
    latest.ecgFlags = frame.flags;
    portEXIT_CRITICAL(&snapshotMux);
    // TODO: sample-by-sample ECG filter, QRS detection, RR intervals and SQI.
    // For dependable timing, replace task polling with verified ADC/DMA timing.
    enqueueSample(frame);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / Config::ECG_HZ));
  }
}

void imuTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    RawSample frame{};
    frame.kind = Kind::IMU;
    frame.timeMs = millis();
    bool ok = imuReady && imu.getSensorData() == BMI2_OK;
    if (ok) {
      const float axes[] = {imu.data.accelX, imu.data.accelY, imu.data.accelZ,
                            imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ};
      for (float value : axes) ok = ok && std::isfinite(value);
      if (ok) {
        frame.flags = VALUE_PRESENT;
        for (size_t i = 0; i < 6; ++i)
          frame.values.imu[i] = quantize(axes[i], i < 3 ? 1000.0f : 10.0f, frame.flags);
      }
    }
    portENTER_CRITICAL(&snapshotMux);
    latest.imuMs = frame.timeMs;
    latest.imuFlags = frame.flags;
    latest.ax = ok ? imu.data.accelX : NAN;
    latest.ay = ok ? imu.data.accelY : NAN;
    latest.az = ok ? imu.data.accelZ : NAN;
    latest.gx = ok ? imu.data.gyroX : NAN;
    latest.gy = ok ? imu.data.gyroY : NAN;
    latest.gz = ok ? imu.data.gyroZ : NAN;
    portEXIT_CRITICAL(&snapshotMux);
    // TODO: use FIFO/data-ready for guaranteed fresh samples; orientation fusion,
    // gravity removal and windowed motion features do not exist in this sample.
    enqueueSample(frame);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / Config::IMU_HZ));
  }
}

bool pressureResponds() {
  Wire.beginTransmission(Config::PRESSURE_ADDRESS);
  return Wire.endTransmission() == 0;
}

void pressureTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  float surfacePa = NAN; // never silently reuse a prior session's calibration
  for (;;) {
    float p = NAN, temperature = NAN;
    bool present = false;
    if (pressureReady && pressureResponds()) {
      pressureSensor.read(); // may block ~40ms; this task owns separate Wire
      p = pressureSensor.pressure(MS5837::Pa);
      temperature = pressureSensor.temperature();
      present = pressureResponds() && std::isfinite(p) && p > 0 && p < 3500000 &&
                std::isfinite(temperature) && temperature > -50 && temperature < 100;
    }
    // Probe + range check are NOT proof of a valid conversion. The library read()
    // returns void; TODO propagate conversion/I2C errors through a checked driver.
    if (calibrationRequested.exchange(false)) {
      if (present) {
        surfacePa = p;
        Serial.println("Air baseline captured (bench approximation, one sample).");
      } else Serial.println("Baseline rejected: no pressure measurement.");
    }
    RawSample frame{};
    frame.timeMs = millis();
    frame.kind = Kind::PRESSURE;
    if (present) {
      frame.flags = VALUE_PRESENT | DRIVER_UNVERIFIED;
      frame.values.pressure.pressurePa = static_cast<int32_t>(lroundf(p));
      frame.values.pressure.temperatureCentiC = static_cast<int16_t>(lroundf(temperature * 100));
    }
    float depth = present && std::isfinite(surfacePa)
                    ? (p - surfacePa) / (fluidDensity * 9.80665f) : NAN;
    portENTER_CRITICAL(&snapshotMux);
    latest.pressureMs = frame.timeMs;
    latest.pressureFlags = frame.flags;
    latest.pressurePa = present ? p : NAN;
    latest.temperatureC = present ? temperature : NAN;
    latest.surfacePa = surfacePa;
    latest.depthM = depth;
    portEXIT_CRITICAL(&snapshotMux);
    enqueueSample(frame);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / Config::PRESSURE_HZ));
  }
}

bool evaluateDistressPlaceholder(const Snapshot& sample) {
  (void)sample;
  // TODO: quality/freshness gating; ~2s motion features; recent valid ECG beats;
  // depth trend/context; calibrated persistence and recovery hysteresis.
  // Missing data is UNKNOWN, never zero movement or confirmed recovery.
  // A chest pressure sensor does not measure airway immersion.
  return false; // intentionally no invented thresholds or automatic actuation
}

void requestDeploymentDryRun(const char* reason) {
  State previous = state.exchange(State::DEPLOYMENT_REQUESTED);
  if (previous != State::DEPLOYMENT_REQUESTED)
    Serial.printf("DRY RUN deployment request: %s. SERVO NOT MOVED.\n", reason);
  // TODO: event marker + snapshot 60s pre / 120s post, deployment latch across
  // reset, mechanical readiness and independent feedback. A request is not a
  // successful inflation. Only after mechanical validation would code attach
  // valveServo to GPIO16 and write a calibrated pulse width.
}

struct DebouncedButton {
  bool previousRaw = false, stable = false;
  uint32_t changedMs = 0;
  bool pressed(int pin, uint32_t now) {
    bool raw = digitalRead(pin) == LOW;
    if (raw != previousRaw) { previousRaw = raw; changedMs = now; }
    if (now - changedMs >= 30 && raw != stable) { stable = raw; return stable; }
    return false;
  }
};

void controlTask(void*) {
  DebouncedButton sos, cancel;
  uint32_t lastEvaluation = 0;
  for (;;) {
    uint32_t now = millis();
    if (Config::PINMAP_CONFIRMED) {
      if (sos.pressed(Config::SOS_BUTTON, now)) requestDeploymentDryRun("SOS button");
      if (cancel.pressed(Config::CANCEL_BUTTON, now) && state.load() == State::SUSPECT)
        state.store(State::MONITOR); // cannot undo a deployment request
    }
    if (now - lastEvaluation >= 250) {
      lastEvaluation = now;
      if (evaluateDistressPlaceholder(getSnapshot())) requestDeploymentDryRun("algorithm");
    }
    if (Config::PINMAP_CONFIRMED && Config::ENABLE_ALERT_OUTPUTS) {
      bool alert = state.load() == State::SUSPECT || state.load() == State::DEPLOYMENT_REQUESTED;
      digitalWrite(Config::VIBRATION_GATE, alert ? HIGH : LOW);
      digitalWrite(Config::STROBE_GATE, alert ? HIGH : LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

uint32_t crc32Bytes(const uint8_t* data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

SummaryRecord makeSummary(uint32_t now) {
  Snapshot sample = getSnapshot();
  uint32_t quality = qualityMask(sample, now);
  SummaryRecord record{};
  record.timeMs = now;
  record.depthM = (quality & 8) ? sample.depthM : NAN;
  record.pressurePa = (quality & 8) ? sample.pressurePa : NAN;
  record.heartRateBpm = NAN; // TODO: only populate from validated ECG processing
  record.droppedSamples = droppedSamples.load();
  record.qualityMask = quality;
  record.ecgAdc = sample.ecgAdc; // meaningful only if quality bit0 is set
  record.state = static_cast<uint8_t>(state.load());
  record.formatVersion = 1;
  record.crc32 = crc32Bytes(reinterpret_cast<const uint8_t*>(&record), offsetof(SummaryRecord, crc32));
  return record;
}

void storageTask(void*) {
  SummaryRecord batch[10];
  size_t batchCount = 0;
  uint32_t lastSummary = millis();
  for (;;) {
    RawSample frame{};
    if (xQueueReceive(sampleQueue, &frame, pdMS_TO_TICKS(10)) == pdPASS && history) {
      history[historyWriteIndex] = frame;
      historyWriteIndex = (historyWriteIndex + 1) % Config::HISTORY_SAMPLES;
      if (historyCount.load() < Config::HISTORY_SAMPLES) ++historyCount;
    }
    uint32_t now = millis();
    if (now - lastSummary < 1000) continue;
    lastSummary = now; // actual timestamp records late execution; no fake catch-up samples
    SummaryRecord record = makeSummary(now);
    if (!filesystemReady || storageFault.load()) continue;
    batch[batchCount++] = record;
    if (batchCount != 10) continue;
    size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
    size_t bytes = batchCount * sizeof(SummaryRecord);
    if (used > total || total - used < Config::FLASH_RESERVE_BYTES + bytes) {
      storageFault.store(true); // stop writes, continue acquisition and processing
      Serial.println("Log partition full: existing recordings retained.");
    } else {
      File file = LittleFS.open(logPath, FILE_APPEND);
      if (!file || file.write(reinterpret_cast<const uint8_t*>(batch), bytes) != bytes) {
        storageFault.store(true);
        Serial.println("Log write failed.");
      }
      if (file) { file.flush(); file.close(); }
    }
    batchCount = 0;
    // TODO: event-file writer, session header (calibration/configuration),
    // interrupted-write recovery, retention policy, and BLE download requests.
    // Last <=10s of summaries and all RAM history can be lost on power failure.
  }
}

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    NimBLEDevice::startAdvertising();
  }
};
BleServerCallbacks bleCallbacks;

void initializeBle() {
  NimBLEDevice::init(Config::DEVICE_NAME);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(&bleCallbacks, false); // static object: server must not delete it
  NimBLEService* service = bleServer->createService(Config::SERVICE_UUID);
  statusCharacteristic = service->createCharacteristic(Config::STATUS_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* info = service->createCharacteristic(Config::LOG_INFO_UUID, NIMBLE_PROPERTY::READ);
  info->setValue("Download TODO");
  BleStatus initial{};
  initial.heartRateBpm = 0xffff;
  initial.depthMm = INT32_MIN;
  statusCharacteristic->setValue(reinterpret_cast<uint8_t*>(&initial), sizeof(initial));
  service->start();
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->enableScanResponse(true);
  advertising->addServiceUUID(Config::SERVICE_UUID);
  advertising->setName(Config::DEVICE_NAME);
  advertising->start();
  // TODO: pairing/bonding and a session/chunk/CRC/ACK/resume transfer protocol.
  // No BLE command can operate the actuator in this sample.
}

void initializeStorage() {
  history = static_cast<RawSample*>(heap_caps_calloc(Config::HISTORY_SAMPLES,
      sizeof(RawSample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!history) {
    storageFault.store(true);
    Serial.println("PSRAM allocation failed: enable OPI PSRAM; raw history unavailable.");
  }
  if (!Config::ENABLE_FLASH_LOGGING) return;
  if (!preferences.begin("wearable", false)) { storageFault.store(true); return; }
  fluidDensity = preferences.getFloat("rho", 997.0f);
  if (!std::isfinite(fluidDensity) || fluidDensity < 900 || fluidDensity > 1200) fluidDensity = 997.0f;
  uint32_t session = preferences.getUInt("session", 0) + 1;
  bool saved = session != 0 && preferences.putUInt("session", session) == sizeof(session);
  preferences.end();
  if (!saved) { storageFault.store(true); return; }
  snprintf(logPath, sizeof(logPath), "/session-%lu-v1.bin", static_cast<unsigned long>(session));
  filesystemReady = LittleFS.begin(false); // never format existing data automatically
  if (!filesystemReady || LittleFS.exists(logPath)) {
    filesystemReady = false;
    storageFault.store(true);
    Serial.println("Filesystem unavailable or session collision; logging disabled.");
  }
}

void initializeHardware() {
  if (!Config::PINMAP_CONFIRMED) return;
  // Keep MOSFETs low. Servo pin is held low and NEVER attached to PWM.
  for (int pin : {Config::VIBRATION_GATE, Config::STROBE_GATE, Config::SERVO_SIGNAL}) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  pinMode(Config::CANCEL_BUTTON, INPUT_PULLUP);
  pinMode(Config::SOS_BUTTON, INPUT_PULLUP);
  pinMode(Config::ECG_LO_PLUS, INPUT_PULLUP);
  pinMode(Config::ECG_LO_MINUS, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(Config::ECG_OUT, ADC_11db);
  bool imuBus = Wire1.begin(Config::IMU_SDA, Config::IMU_SCL, 400000);
  bool depthBus = Wire.begin(Config::DEPTH_SDA, Config::DEPTH_SCL, 400000);
  imuReady = imuBus && imu.beginI2C(Config::IMU_ADDRESS, Wire1) == BMI2_OK;
  if (imuReady) {
    imuReady = imu.setAccelODR(BMI2_ACC_ODR_100HZ) == BMI2_OK &&
               imu.setGyroODR(BMI2_GYR_ODR_100HZ) == BMI2_OK;
  }
  pressureReady = depthBus && pressureSensor.init(); // released 1.1.1 uses Wire
  if (pressureReady) {
    pressureSensor.setModel(MS5837::MS5837_30BA);
    pressureSensor.setFluidDensity(fluidDensity);
  }
  Serial.printf("IMU ready: %d; pressure initialized: %d\n", imuReady, pressureReady);
}

void startTask(TaskFunction_t task, const char* name, uint32_t stackBytes, UBaseType_t priority) {
  if (xTaskCreate(task, name, stackBytes, nullptr, priority, nullptr) != pdPASS) {
    taskFault.store(true);
    Serial.printf("TASK CREATION FAILED: %s\n", name);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Wearable framework: no automatic detection or physical deployment.");
  Serial.printf("Flash: %lu bytes; PSRAM: %lu bytes\n",
      static_cast<unsigned long>(ESP.getFlashChipSize()), static_cast<unsigned long>(ESP.getPsramSize()));
  if (!Config::PINMAP_CONFIRMED) Serial.println("Hardware disabled: review pins and set PINMAP_CONFIRMED.");
  sampleQueue = xQueueCreate(256, sizeof(RawSample));
  if (!sampleQueue) { taskFault.store(true); Serial.println("Queue allocation failed."); return; }
  initializeStorage();
  initializeHardware();
  initializeBle();
  state.store(State::MONITOR);
  // Example priorities. No pinning or hard-real-time guarantee is assumed.
  startTask(ecgTask, "ecg", 4096, 4);
  startTask(imuTask, "imu", 4096, 3);
  startTask(pressureTask, "pressure", 4096, 2);
  startTask(controlTask, "control", 4096, 3);
  startTask(storageTask, "storage", 6144, 1);
}

void loop() {
  if (Serial.available() && Serial.read() == 'z') {
    if (Config::PINMAP_CONFIRMED) calibrationRequested.store(true);
    else Serial.println("Calibration unavailable: hardware disabled.");
  }
  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  if (now - lastStatus >= 1000) {
    lastStatus = now;
    Snapshot sample = getSnapshot();
    BleStatus status{};
    status.timeMs = now;
    status.state = static_cast<uint8_t>(state.load());
    status.flags = static_cast<uint8_t>(qualityMask(sample, now));
    status.heartRateBpm = 0xffff;
    bool freshDepth = (status.flags & 8) && std::isfinite(sample.depthM);
    status.depthMm = freshDepth ? static_cast<int32_t>(lroundf(sample.depthM * 1000)) : INT32_MIN;
    status.droppedSamples = droppedSamples.load();
    status.historySamples = historyCount.load();
    if (statusCharacteristic) {
      statusCharacteristic->setValue(reinterpret_cast<uint8_t*>(&status), sizeof(status));
      if (bleServer->getConnectedCount()) statusCharacteristic->notify();
    }
    Serial.printf("state=%u flags=0x%02x depth=%0.3f m raw=%lu dropped=%lu\n",
        status.state, status.flags, freshDepth ? sample.depthM : NAN,
        static_cast<unsigned long>(status.historySamples), static_cast<unsigned long>(status.droppedSamples));
  }
  delay(10);
}
