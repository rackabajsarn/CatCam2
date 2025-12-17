#define MQTT_MAX_PACKET_SIZE 65535
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_camera.h>
#include <ArduinoOTA.h>
#include <base64.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "esp_system.h" // This is needed to use esp_reset_reason()
#include "secrets.h"
//#include "model_handler.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "img_converters.h" // For JPEG decoding
#include "esp_jpg_decode.h" // For JPG_SCALE_4X
#include <ESP32_JPEG_Library.h>
//#include <ArduTFLite.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
// TensorFlow Lite Micro headers
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
//#include "tensorflow/lite/version.h"

// Wi-Fi credentials from secrets.h
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT topics
#define COMMAND_TOPIC "catflap/command"
#define DEBUG_TOPIC "catflap/debug"
#define ALERT_TOPIC "catflap/alert"
#define WIFI_SIGNAL_TOPIC "catflap/wifi_signal"
#define FREE_HEAP_TOPIC "catflap/free_heap"
#define IR_PULSE_QUALITY_TOPIC "catflap/ir_pulse_quality"
#define ROUNDTRIP_TOPIC "catflap/roundtrip"
#define LOOPTIME_TOPIC "catflap/looptime"
#define IP_TOPIC "catflap/ip"
#define IMAGE_TOPIC "catflap/image"
#define IR_BARRIER_TOPIC "catflap/ir_barrier"
#define FLAP_STATE_TOPIC "catflap/flap_state"
#define DETECTION_MODE_TOPIC "catflap/detection_mode"
#define CAT_LOCATION_TOPIC "catflap/cat_location"
#define DEBUG_TOGGLE_TOPIC "catflap/debug_toggle"
#define COOLDOWN_STATE_TOPIC "catflap/cooldown"
#define CAMERA_PRESET_TOPIC "catflap/camera_preset"
#define SET_DETECTION_MODE_TOPIC "catflap/detection_mode/set"
#define SET_DEBUG_TOGGLE_TOPIC "catflap/debug_toggle/set"
#define SET_COOLDOWN_TOPIC "catflap/cooldown/set"
#define SET_CAMERA_PRESET_TOPIC "catflap/camera_preset/set"
#define SET_CAMERA_PRESET_SAVE_TOPIC "catflap/camera_preset/save"
#define SET_AE_LEVEL_TOPIC "catflap/ae_level/set"
#define SET_AEC_VALUE_TOPIC "catflap/aec_value/set"
#define SET_EXPOSURE_CTRL_TOPIC "catflap/exposure_ctrl/set"
#define SET_AEC2_TOPIC "catflap/aec2/set"
#define SET_GAIN_CTRL_TOPIC "catflap/gain_ctrl/set"
#define SET_AGC_GAIN_TOPIC "catflap/agc_gain/set"
#define SET_CAT_LOCATION_TOPIC "catflap/cat_location/set"
#define SET_FLAP_STATE_TOPIC "catflap/flap_state/set"
#define INFERENCE_TOPIC "catflap/inference"
#define SET_EXPOSURE_CTRL_TOPIC "catflap/exposure_ctrl/set"
#define SET_AEC_VALUE_TOPIC "catflap/aec_value/set"
#define EXPOSURE_CTRL_TOPIC "catflap/exposure_ctrl"
#define AEC_VALUE_TOPIC "catflap/aec_value"
#define SET_JPEG_QUALITY_TOPIC "catflap/jpeg_quality/set"
#define SET_BRIGHTNESS_TOPIC "catflap/brightness/set"
#define SET_CONTRAST_TOPIC "catflap/contrast/set"
#define SET_AWB_TOPIC "catflap/awb/set"
#define INFERENCE_MODE_TOPIC "catflap/inference_mode"
#define SET_INFERENCE_MODE_TOPIC "catflap/inference_mode/set"
#define ESP32_INFERENCE_TOPIC "catflap/esp32_inference"
#define SERVER_INFERENCE_TOPIC "catflap/server_inference"
#define INFERENCE_COMPARISON_TOPIC "catflap/inference_comparison"

// Device information
#define DEVICE_NAME "catflap"
#define DEVICE_UNIQUE_ID "catflap_esp32"
#define MQTT_DISCOVERY_PREFIX "homeassistant"
#define DEVICE_SW_VERSION "1.1.0" //Increment together with git commits

// EEPROM Addresses
#define EEPROM_SIZE 160
#define EEPROM_VERSION 7  // Increment this number whenever you change the EEPROM layout

// Base Addresses
#define EEPROM_ADDR_VERSION         0
#define EEPROM_ADDR_DETECTION_MODE  1
#define EEPROM_ADDR_COOLDOWN        2
#define EEPROM_ADDR_CAT_LOCATION    3
#define EEPROM_ADDR_INFERENCE_MODE  4
#define EEPROM_ADDR_ACTIVE_PRESET   5

// Camera Settings Blocks (Base Addresses)
#define EEPROM_BASE_LIVE_SETTINGS   10
#define EEPROM_BASE_PRESET_DAY      30
#define EEPROM_BASE_PRESET_NIGHT    50

// Camera Settings Offsets (relative to Base)
#define CAM_OFFSET_QUALITY          0
#define CAM_OFFSET_BRIGHTNESS       1
#define CAM_OFFSET_CONTRAST         2
#define CAM_OFFSET_AWB              3
#define CAM_OFFSET_AE_LEVEL         4
#define CAM_OFFSET_AEC_VALUE_L      5
#define CAM_OFFSET_AEC_VALUE_H      6
#define CAM_OFFSET_AEC              7
#define CAM_OFFSET_AEC2             8
#define CAM_OFFSET_AGC              9
#define CAM_OFFSET_AGC_GAIN         10

// Inference Modes
#define INFERENCE_MODE_LOCAL  0
#define INFERENCE_MODE_SERVER 1
#define INFERENCE_MODE_BOTH   2

// GPIO Definitions
#define IR_PIN 32
//#define OPEN_SENSOR_PIN 32
#define ENABLE_FLAP_PIN 33

#define BPP 1  // Grayscale: 1 byte per pixel
#define IR_PWM_PIN 13
#define IR_PWM_CH  2
#define IR_FREQ    38000
#define IR_DUTY    128
#define BURST_INTERVAL       600       // µs: period of on/off for IR bursts
#define IR_DEBOUNCE_MS       10        // time input must stay stable
#define IR_MIN_HOLD_MS       120       // rate-limit chatter (optional)
#define PULSE_CHECK_INTERVAL 20        // 50 ms between pulse count checks
#define PULSE_THRESHOLD      8        // 20 pulses in interval means beam is OK
//#define IR_SEEN_WINDOW_US    1500   // Beam is 'OK' if a LOW edge was seen in last 3 ms

// Camera pin configuration
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      5
#define Y2_GPIO_NUM      4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22


struct ModelMetadata {
	String modelName;
	uint8_t numberOfLabels;
	float threshold;
};

struct ExitMetrics {
	uint16_t mean;
	uint16_t gradMean;        // 0..255-ish
	uint16_t whitePermille;   // 0..1000
	uint8_t  minv;
	uint8_t  maxv;
};

// Forward declarations of functions
void IRAM_ATTR ir_falling_isr();
void checkResetReason();
enum IrState : uint8_t { IR_CLEAR=0, IR_BROKEN=1 };  // CLEAR = beam received
inline IrState readIrRaw();
void ir_init();
void ir_on();
void ir_off();
void ir_burst();
void irDisableForCritical();
void irEnableAfterCritical();
void setup_wifi();
void mqttConnect();
void mqttReconnect();
void mqttSubscribe();
void mqttInitialPublish();
esp_err_t initCamera();
void captureAndSendImage();
uint8_t* jpegToGrayscale(const uint8_t* jpg_buf, size_t jpg_len, int* out_width, int* out_height);
camera_fb_t* cropImage(camera_fb_t *fb);
camera_fb_t* resizeImage(camera_fb_t *cropped, int originalSize, int targetSize);
camera_fb_t* cropFrame(camera_fb_t *fb);
camera_fb_t* resizeFrame(camera_fb_t *cropped);
bool setupInference();
bool loadModelFromSD(const char* path, uint8_t** model_data, size_t* model_size);
bool loadModelMetadata(const char *path, ModelMetadata &meta);
void enableFlap();
void disableFlap();
void handleMqttMessages(char* topic, byte* payload, unsigned int length);
void setupOTA();
void setupWebServer();
void publishDiscoveryConfigs();
void publishCooldownState();
void publishDiagnostics();
void publishIPState();
void monitorHeap();
void recordLoopTime();
float getAndResetMeanLoopTime();
void mqttDebugPrint(const char* message);
void mqttDebugPrintln(const char* message);
void mqttDebugPrintf(const char* format, ...);
void handleInferenceTopic(String inferenceStr);
void handleServerInference(String serverInferenceStr);
void handleJpegQualityCommand(String qualityStr);
void publishCameraSettings();
void handleBrightnessCommand(String brightnessStr);
void handleContrastCommand(String contrastStr);
void handleSaturationCommand(String saturationStr);
void handleAWBCommand(String awbStateStr);
void handleAECValueCommand(String aecStr);
void handleCooldownCommand(String cooldownStr);
void handleAeLevelCommand(String levelStr);
void handleAecValueCommand(String valueStr);
void handleExposureCtrlCommand(String stateStr);
void handleAec2Command(String stateStr);
void handleGainCtrlCommand(String stateStr);
void handleAgcGainCommand(String valueStr);
void handleCameraPresetSelect(String presetStr);
void handleCameraPresetSave();
void handleIRBarrierStateChange(bool barrierBroken);
void saveSettingsToEEPROM();
void loadSettingsFromEEPROM();
void handleDetectionModeCommand(String stateStr);
void handleDebugToggleCommand(String stateStr);
void handleCatLocationCommand(String locationStr);
void handleFlapStateCommand(String stateStr);
void updateIRBarrierState();
bool isEEPROMInitialized();
void initializeEEPROM();
void savePresetToEEPROM(uint8_t presetIndex);
void applyPresetFromEEPROM(uint8_t presetIndex);
void handleInferenceModeCommand(String modeStr);
bool saveGrayscaleToSD(uint8_t* buf, int width, int height, const char* filename);
uint32_t fnv1a_32(const uint8_t* data, size_t len);
String makeImageName(const uint8_t* data, size_t len);
bool isExitLike96(const uint8_t* img, int w, int h, ExitMetrics* out);
// Add more handler functions as needed

WiFiClient espClient;
PubSubClient client(espClient);
AsyncWebServer server(80);  // HTTP server for model uploads

// Global variables
bool flapEnabled = true;  // Initialize flap state
bool detectionModeEnabled = true;
bool mqttDebugEnabled = true;
bool catLocation = true; // true = home
bool barrierTriggered = false;  // Flag to indicate barrier has been triggered
bool barrierStableState = false; // Last confirmed stable state (false = not broken, true = broken)
esp_err_t cameraInitStatus = ESP_FAIL;
uint8_t inferenceMode = INFERENCE_MODE_BOTH; // Default to both

// Model comparison tracking
String lastESP32Inference = "";
String lastServerInference = "";
unsigned long totalInferences = 0;
unsigned long inferenceMatches = 0;
unsigned long inferenceMismatches = 0;

// Timer for saving settings
unsigned long lastSettingsChangeTime = 0;
const unsigned long SETTINGS_SAVE_DELAY = 15000; // 15 seconds

// Timer for wifi signal update
unsigned long lastDiagnosticUpdateTime = 0;
const unsigned long DIAGNOSTIC_UPDATE_INTERVAL = 15000; 

// Variables for loop time measurement
unsigned long loopStartTime = 0;
unsigned long loopDurationSum = 0;
unsigned long loopCount = 0;

// Timing for full trigger -> capture -> send -> inference chain
unsigned long triggerStartTime = 0;      // IR barrier becomes stably broken
unsigned long captureStartTime = 0;      // captureAndSendImage() entry
unsigned long captureEndTime = 0;        // after successful capture
unsigned long sendEndTime = 0;           // after MQTT image publish
// Local inference timing breakdown
unsigned long decodeStartTime = 0;       // JPEG decode start
unsigned long decodeEndTime = 0;         // JPEG decode complete
unsigned long cropResizeEndTime = 0;     // crop + resize complete
unsigned long inferenceEndTime = 0;      // TFLite inference complete

// Debounce and Cooldown Settings
float cooldownDuration = 0.0;    // Cooldown duration in seconds (default to 0)
unsigned long lastBarrierClearedTime = 0;     // Timestamp of the last valid trigger
unsigned long lastIrDebug = 0;

uint8_t activeCameraPreset = 0; // 0 = day, 1 = night

static IrState g_stable = IR_CLEAR;
static IrState g_candidate = IR_CLEAR;
static uint32_t g_lastChangeMs = 0;      // when candidate changed
static uint32_t g_lastStableMs = 0;      // last time we committed a stable change
static bool g_irSensingEnabled = true;   // mask while IR is intentionally off
static volatile uint32_t g_pulseCount = 0;    // count IR pulses for robust detection
static uint32_t g_lastPulseCheckMs = 0;       // last time we checked pulse count
static uint32_t g_pulseQualitySum = 0;
static uint32_t g_pulseQualitySamples = 0;


// The Tensor Arena memory area is used by TensorFlow Lite to store input, output and intermediate tensors
// Try internal RAM first (much faster), fall back to PSRAM if needed.
// Keep internal target modest to improve allocation success; arena usage is ~48 KB with the slim model
constexpr int kTensorArenaSizeInternal = 64 * 1024;   // Prefer a small block to leave headroom for servers
constexpr int kTensorArenaSizePSRAM = 600 * 1024;     // Fallback size in PSRAM if internal fails
int kTensorArenaSize = 0;  // Actual allocated size
uint8_t* tensor_arena = nullptr;
bool tensor_arena_in_psram = false;

// Global pointers for the model and interpreter.
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
bool g_model_loaded = false; // True if model loaded successfully
bool g_sd_ready = false;     // True if SD card initialized and mounted

// Global pointer to the active model data (owned heap buffer from SD).
const uint8_t* g_model_data = nullptr;
size_t g_model_size = 0;
// If non-null, this holds a heap/PSRAM-allocated model we own and must free on reload.
static uint8_t* g_owned_heap_model = nullptr;
static ModelMetadata currentModelMeta;

// -------- Model ownership helpers (SD-only model) --------
static void freeOwnedModel()
{
  if (g_owned_heap_model) {
    heap_caps_free(g_owned_heap_model);
    g_owned_heap_model = nullptr;
  }
}

static bool adoptFileModel(const char* path)
{
  uint8_t* buf = nullptr;
  size_t sz = 0;
  if (!loadModelFromSD(path, &buf, &sz)) {
    return false;
  }
  // Minimal FlatBuffer sanity: "TFL3" at bytes 4..7
  if (!(sz >= 8 && buf[4]=='T' && buf[5]=='F' && buf[6]=='L' && buf[7]=='3')) {
    heap_caps_free(buf);
    return false;
  }

  freeOwnedModel();
  g_owned_heap_model = buf;
  g_model_data = buf;   // const pointer to owned heap
  g_model_size = sz;
  return true;
}

// Try primary metadata path, then legacy path if present.
static bool loadActiveModelMetadata()
{
  if (loadModelMetadata("/model_meta.json", currentModelMeta)) {
    return true;
  }
  // Legacy location used by older builds
  return loadModelMetadata("/models/model_metadata.json", currentModelMeta);
}
// -------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for Serial

  // Try internal RAM first (faster). Use modest target size and log largest free block.
  tensor_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSizeInternal, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (tensor_arena) {
    kTensorArenaSize = kTensorArenaSizeInternal;
    tensor_arena_in_psram = false;
    Serial.printf("Tensor arena in INTERNAL RAM: %d KB\n", kTensorArenaSize / 1024);
  } else {
    size_t largestInternal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    Serial.printf("Internal RAM arena alloc failed. Largest free internal block: %u bytes\n", (unsigned)largestInternal);
    // Try a right-sized internal arena if there is a reasonably large contiguous block
    size_t retryInternalSize = (largestInternal > (12 * 1024)) ? (largestInternal - 12 * 1024) : 0; // keep headroom
    if (retryInternalSize >= 64 * 1024) {
      tensor_arena = (uint8_t*)heap_caps_malloc(retryInternalSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (tensor_arena) {
        kTensorArenaSize = retryInternalSize;
        tensor_arena_in_psram = false;
        Serial.printf("Tensor arena in INTERNAL RAM (downsized): %d KB\n", kTensorArenaSize / 1024);
      }
    }
  }

  if (!tensor_arena) {
    // Fall back to PSRAM
    tensor_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSizePSRAM, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (tensor_arena) {
      kTensorArenaSize = kTensorArenaSizePSRAM;
      tensor_arena_in_psram = true;
      Serial.printf("Tensor arena in PSRAM: %d KB (slower)\n", kTensorArenaSize / 1024);
    } else {
      Serial.println("Failed to allocate tensor arena!");
    }
  }

  Serial.printf("Heap after arena alloc: total=%u, internal=%u, psram=%u\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  bool sdFailed = false;
  bool modelFailed = false;
  
  // Initialize SD card and load model from SD
  if (!SD_MMC.begin("/sdcard", true)) {
    g_model_loaded = false;
    sdFailed = true;
    g_sd_ready = false;
  } else {
    g_sd_ready = true;
    uint8_t cardType = SD_MMC.cardType();
    uint64_t cardSizeMB = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD card type: %d, size: %llu MB\n", cardType, (unsigned long long)cardSizeMB);
    mqttDebugPrintf("SD card mounted: type %d, %llu MB\n", cardType, (unsigned long long)cardSizeMB);
    // Ensure inference folder exists for saved frames/web listing
    if (!SD_MMC.exists("/inference")) {
      if (!SD_MMC.mkdir("/inference")) {
        mqttDebugPrintln("Failed to create /inference directory on SD");
        Serial.println("Failed to create /inference directory on SD");
      }
    }
    if (!adoptFileModel("/model.tflite")) {
      modelFailed = true;
      g_model_loaded = false;
    } else {
      mqttDebugPrintln("Using SD card model");
      loadActiveModelMetadata();
      g_model_loaded = true;
    }
  }
  cameraInitStatus = initCamera();
  
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize GPIOs
  pinMode(IR_PIN, INPUT);
  pinMode(ENABLE_FLAP_PIN, OUTPUT);
  
  // Start IR transmitter FIRST
  ir_init();
  ir_on();
  
  // Give the IR carrier time to stabilize and TSOP to settle
  delay(100);
  
  // NOW attach the interrupt and reset counters
  g_pulseCount = 0;
  g_lastPulseCheckMs = millis();
  attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_falling_isr, FALLING);
  
  // Wait for first valid reading
  delay(PULSE_CHECK_INTERVAL + 5);  // Wait one pulse check interval
  IrState initialReading = readIrRaw();
  g_stable = initialReading;
  g_candidate = initialReading;
  g_lastStableMs = millis();  // ← ADD THIS LINE

  setup_wifi();
    // Disable Wi-Fi power save mode
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Initialize MQTT
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(handleMqttMessages);

  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
    
  mqttConnect();

  setupOTA();  // Initialize OTA

  // Initialize EEPROM
  if (!isEEPROMInitialized()) {
    initializeEEPROM();
  } else {
    loadSettingsFromEEPROM();
  }

  setupWebServer();  // Initialize HTTP server for model uploads
  checkResetReason();
  publishIPState();
  mqttInitialPublish();

  setupInference();
  if (sdFailed) mqttDebugPrintln("SD card initialization failed - local inference disabled");
  if (modelFailed) mqttDebugPrintln("Model loading from SD failed - local inference disabled");
  mqttDebugPrintln("ESP Setup finished");
  mqttDebugPrintf("Camera init: 0x%x (%s)\n",
                cameraInitStatus,
                esp_err_to_name(cameraInitStatus));
}

void loop() {
  recordLoopTime();
  // Reconnect to Wi-Fi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Handle IR barrier state
  updateIRBarrierState();

  // Save settings to EEPROM if needed
  if (millis() - lastSettingsChangeTime > SETTINGS_SAVE_DELAY && lastSettingsChangeTime != 0) {
    saveSettingsToEEPROM();
    lastSettingsChangeTime = 0;  // Reset timer
  }

  publishDiagnostics();
  //delay(10);  // Small delay to prevent watchdog resets
  
  // IR transmitter burst handling
  ir_burst();



// In loop(), near the end:
// if (millis() - lastIrDebug > 1000) {
//   lastIrDebug = millis();
//   uint32_t nowUs = micros();
//   bool beam_ok = (nowUs - g_lastIrLowUs) < IR_SEEN_WINDOW_US;
//   int raw = digitalRead(IR_PIN);
//   mqttDebugPrintf("IR debug: raw=%d, beam_ok=%s, dt_us=%lu\n",
//                   raw,
//                   beam_ok ? "true" : "false",
//                   (unsigned long)(nowUs - g_lastIrLowUs));
// }


  yield();
}


void IRAM_ATTR ir_falling_isr() {
  // TSOP output is active LOW when it detects IR carrier
  // Just count pulses - much more robust than timing individual edges
  g_pulseCount++;
}

void ir_init()
{
    ledcSetup(IR_PWM_CH, IR_FREQ, 8);   // 38 kHz, 8-bit
    ledcAttachPin(IR_PWM_PIN, IR_PWM_CH);
    ledcWrite(IR_PWM_CH, 0);            // start off
}

void ir_on()  { ledcWrite(IR_PWM_CH, IR_DUTY); }
void ir_off() { ledcWrite(IR_PWM_CH, 0); }

void ir_burst() {
    // Call from loop() or a 1 kHz timer
    static uint32_t t=0; static bool on=false;
    if (micros() - t > BURST_INTERVAL) {  // ~600 µs
      t = micros();
      on = !on;
      on ? ir_on() : ir_off();
    }
}

// Map raw digital read to logical state
inline IrState readIrRaw()
{
  // Check pulse count over a time window for robust detection
  static IrState lastReading = IR_CLEAR;
  uint32_t nowMs = millis();
  
  // Every 20ms, check if we got enough pulses
  if (nowMs - g_lastPulseCheckMs >= PULSE_CHECK_INTERVAL) {
    uint32_t pulses = g_pulseCount;
    g_pulseCount = 0;  // Reset counter
    g_lastPulseCheckMs = nowMs;
    const uint32_t expectedPulses = (PULSE_CHECK_INTERVAL * 1000UL) / (BURST_INTERVAL * 2UL);  // ms -> µs, two edges per burst
    const uint32_t clampExpected = expectedPulses > 0 ? expectedPulses : 1;
    uint32_t pulsePercent = (pulses * 100U) / clampExpected;
    if (pulsePercent > 100U) {
      pulsePercent = 100U;
    }
    g_pulseQualitySum += pulsePercent;
    g_pulseQualitySamples++;
    
    // We burst at ~830 Hz (600µs on + 600µs off = 1200µs period)
    // In 50ms we should see ~41 pulses if beam is clear
    // Allow some margin: if we see >20 pulses, beam is clear
    lastReading = (pulses > PULSE_THRESHOLD) ? IR_CLEAR : IR_BROKEN;
  }
  
  // Return the last calculated reading
  return lastReading;
}

void irDisableForCritical() {
  ir_off();  // Physically turn off IR transmitter to prevent spurious signals
  detachInterrupt(digitalPinToInterrupt(IR_PIN));
  g_irSensingEnabled = false;
}

void irEnableAfterCritical() {
  g_pulseCount = 0;
  uint32_t now = millis();
  g_lastPulseCheckMs = now;
  g_candidate = g_stable;
  g_lastChangeMs = now;
  g_lastStableMs = now;
  g_irSensingEnabled = true;
  attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_falling_isr, FALLING);
  delay(5);
  ir_on();  // Turn IR transmitter back on
}

void checkResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();

  switch (reason) {
    case ESP_RST_POWERON:
      mqttDebugPrintln("Reset due to power on");
      break;
    case ESP_RST_SW:
      mqttDebugPrintln("Reset due to software restart");
      break;
    case ESP_RST_PANIC:
      mqttDebugPrintln("Reset due to exception/panic");
      break;
    case ESP_RST_INT_WDT:
      mqttDebugPrintln("Reset due to interrupt watchdog");
      break;
    case ESP_RST_TASK_WDT:
      mqttDebugPrintln("Reset due to task watchdog");
      break;
    default:
      mqttDebugPrintln("Unknown reset reason");
      break;
    }
}

bool isEEPROMInitialized() {
  uint8_t version = EEPROM.read(EEPROM_ADDR_VERSION);
  return version == EEPROM_VERSION;
}

void initializeEEPROM() {
  mqttDebugPrintln("Initializing EEPROM with default settings...");
  
  // Write the current version to EEPROM
  // This write is buffered and will be committed by saveSettingsToEEPROM()
  EEPROM.write(EEPROM_ADDR_VERSION, EEPROM_VERSION);
  
  // Initialize settings to default values
  detectionModeEnabled = true;
  cooldownDuration = 0.0;
  catLocation = true; // default: cat is home
  inferenceMode = INFERENCE_MODE_BOTH; // Default to both
  
  // Initialize camera settings to defaults
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 5);
    s->set_brightness(s, -2);
    s->set_contrast(s, 1);
    s->set_whitebal(s, true);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 0);
    s->set_exposure_ctrl(s, 1); // auto exposure on
    s->set_aec2(s, 1);          // extra AEC off
    s->set_gain_ctrl(s, 1);     // auto gain on
    s->set_agc_gain(s, 0);
  }
  savePresetToEEPROM(0); // day

  if (s != NULL) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 5);
    s->set_brightness(s, -2);
    s->set_contrast(s, 1);
    s->set_whitebal(s, false);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 100);
    s->set_exposure_ctrl(s, 0); // auto exposure on
    s->set_aec2(s, 0);          // extra AEC off
    s->set_gain_ctrl(s, 0);     // auto gain on
    s->set_agc_gain(s, 5);
  }
  // Save default settings to EEPROM
  // This handles its own critical section and commits the version write above too
  saveSettingsToEEPROM();

  // Initialize presets: copy current camera settings into both day and night 
  // TODO: individualize these defaults
  
  savePresetToEEPROM(1); // night

  // Default active preset: day
  irDisableForCritical();
  activeCameraPreset = 0;
  EEPROM.write(EEPROM_ADDR_ACTIVE_PRESET, activeCameraPreset);
  EEPROM.commit();
  irEnableAfterCritical();
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttConnect() {
  while (!client.connected()) {
    // Generate a unique client ID using the MAC address
    String clientId = "ESP32Client-";
    clientId += String(WiFi.macAddress());

    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttSubscribe();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttReconnect() {
  while (!client.connected()) {
    // Generate a unique client ID using the MAC address
    String clientId = "ESP32Client-";
    clientId += String(WiFi.macAddress());

    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe to command topics
      mqttSubscribe();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttSubscribe() {
    // System commands
    client.subscribe(COMMAND_TOPIC);
    client.subscribe(INFERENCE_TOPIC);
    client.subscribe(SERVER_INFERENCE_TOPIC);  // Subscribe to server inference results

    // Camera settings
    client.subscribe(SET_JPEG_QUALITY_TOPIC);
    client.subscribe(SET_BRIGHTNESS_TOPIC);
    client.subscribe(SET_CONTRAST_TOPIC);
    client.subscribe(SET_AWB_TOPIC);
    client.subscribe(SET_EXPOSURE_CTRL_TOPIC);
    client.subscribe(SET_AEC_VALUE_TOPIC);

    // Operational settings
    client.subscribe(SET_DETECTION_MODE_TOPIC);
    client.subscribe(SET_DEBUG_TOGGLE_TOPIC);
    client.subscribe(SET_COOLDOWN_TOPIC);
    client.subscribe(SET_FLAP_STATE_TOPIC);
    client.subscribe(SET_CAT_LOCATION_TOPIC);
    client.subscribe(SET_AE_LEVEL_TOPIC);
    client.subscribe(SET_AEC_VALUE_TOPIC);
    client.subscribe(SET_EXPOSURE_CTRL_TOPIC);
    client.subscribe(SET_AEC2_TOPIC);
    client.subscribe(SET_GAIN_CTRL_TOPIC);
    client.subscribe(SET_AGC_GAIN_TOPIC);
    client.subscribe(SET_CAMERA_PRESET_TOPIC);
    client.subscribe(SET_CAMERA_PRESET_SAVE_TOPIC);
    client.subscribe(SET_INFERENCE_MODE_TOPIC);
}

void mqttInitialPublish() {
  publishDiscoveryConfigs();
  publishCameraSettings();
  publishCooldownState();
  client.publish(DETECTION_MODE_TOPIC, detectionModeEnabled ? "ON" : "OFF", true);
  client.publish(DEBUG_TOGGLE_TOPIC, mqttDebugEnabled ? "ON" : "OFF", true);
  client.publish(CAT_LOCATION_TOPIC, catLocation ? "ON" : "OFF", true);
  client.publish(CAMERA_PRESET_TOPIC, activeCameraPreset == 0 ? "day" : "night", true);
  
  String modeStr;
  switch(inferenceMode) {
    case INFERENCE_MODE_LOCAL: modeStr = "local"; break;
    case INFERENCE_MODE_SERVER: modeStr = "server"; break;
    default: modeStr = "both"; break;
  }
  client.publish(INFERENCE_MODE_TOPIC, modeStr.c_str(), true);
}

esp_err_t initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // Use JPEG format
  config.fb_location = CAMERA_FB_IN_PSRAM;

  // Frame parameters (QVGA for faster decode)
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 10; // higher number = lower size, faster decode
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    //ESP.restart();
  }
  
  // Apply grayscale effect (special_effect 2 = grayscale)
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_special_effect(s, 2);  // 0=No Effect, 1=Negative, 2=Grayscale, 3=Red Tint, etc.
  }
  
  return err;
}

bool loadModelMetadata(const char *path, ModelMetadata &meta)
{
	File file = SD_MMC.open(path, FILE_READ);
	if (!file) {
		Serial.print("Failed to open metadata file: ");
		Serial.println(path);
		return false;
	}

	StaticJsonDocument<256> doc;  // plenty for this tiny JSON

	DeserializationError err = deserializeJson(doc, file);
	file.close();

	if (err) {
		Serial.print("Failed to parse metadata JSON: ");
		Serial.println(err.c_str());
		return false;
	}

	meta.modelName       = doc["model_name"]       | String("unknown_model");
	meta.numberOfLabels  = doc["number_of_labels"] | 0;
	meta.threshold       = doc["threshold_value"]  | 0.5f;

	return true;
}

// Function to load the model file from the SD card into memory.
bool loadModelFromSD(const char* modelPath, uint8_t** modelBuffer, size_t* modelSize) {
  File modelFile = SD_MMC.open(modelPath, FILE_READ);
  if (!modelFile) {
    Serial.println("Failed to open model file on SD card");
    return false;
  }
  
  *modelSize = modelFile.size();
  // Allocate buffer in PSRAM if available.
  *modelBuffer = (uint8_t*) heap_caps_malloc(*modelSize, MALLOC_CAP_SPIRAM);
  if (!*modelBuffer) {
    Serial.println("Failed to allocate memory for model");
    modelFile.close();
    return false;
  }
  
  // Read the file into the buffer.
  size_t bytesRead = modelFile.read(*modelBuffer, *modelSize);
  modelFile.close();
  
  if (bytesRead != *modelSize) {
    Serial.println("Error reading the complete model file");
    heap_caps_free(*modelBuffer);
    return false;
  }
  
  Serial.printf("Model loaded: %d bytes\n", *modelSize);
  return true;
}

bool updateModelFromSD() {
  if (!g_sd_ready) return false;
  // Check if a new model file exists, e.g. "model_new.tflite"
  if (SD_MMC.exists("/model_new.tflite")) {
    // Optional: Validate the new file (size, checksum, etc.)
    File newModel = SD_MMC.open("/model_new.tflite", FILE_READ);
    if (!newModel) {
      Serial.println("Failed to open new model file.");
      return false;
    }
    size_t newSize = newModel.size();
    newModel.close();
    
    // Remove the old model file.
    if (SD_MMC.exists("/model.tflite")) {
      SD_MMC.remove("/model.tflite");
    }

    // If validation passes, rename the file.
    if (SD_MMC.rename("/model_new.tflite", "/model.tflite")) {
      Serial.println("Model updated successfully.");
      return true;
    } else {
      Serial.println("Failed to rename the model file.");
    }
  }
  return false;
}

bool reloadModel() {
  // Reload from SD and reinitialize interpreter
  g_model_loaded = false;
  if (!adoptFileModel("/model.tflite")) {
    mqttDebugPrintf("Failed to load the new model from SD");
    return false;
  }
  loadActiveModelMetadata();
  g_model_loaded = true;
  if (!setupInference()) {
    g_model_loaded = false;
    mqttDebugPrintf("Failed to reinitialize inference with the new model");
    return false;
  } else {
    mqttDebugPrintln("New model loaded and interpreter reinitialized.");
  }
  return true;
}

// Setup function to initialize the model and interpreter.
bool setupInference() {
  if (!g_model_loaded || g_model_data == nullptr) {
    mqttDebugPrintln("Inference setup skipped: model not loaded.");
    return false;
  }
  mqttDebugPrintln("Setting up inference...");
  // Load the model from flash.
  model = tflite::GetModel(g_model_data);
  if (model == nullptr) {
    mqttDebugPrintln("GetModel returned nullptr. Model data invalid.");
    return false;
  }
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    mqttDebugPrintf("Model schema version (%d) does not match TFLite Micro schema version (%d).\n",
                  model->version(), TFLITE_SCHEMA_VERSION);
    return false;
  }
  
  // Create an op resolver with only the ops our model needs.
  static tflite::MicroMutableOpResolver<12> resolver;
  resolver.AddCast();
  resolver.AddQuantize();
  resolver.AddDepthwiseConv2D();
  resolver.AddConv2D();
  resolver.AddMul();
  resolver.AddAdd();
  resolver.AddSub();
  resolver.AddMaxPool2D();
  resolver.AddMean();
  resolver.AddStridedSlice();
  resolver.AddFullyConnected();
  resolver.AddSoftmax();
  
  // Create a persistent interpreter.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;
  
  // Allocate tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    mqttDebugPrintf("AllocateTensors() failed! Arena %dKB in %s may be too small\n",
                    kTensorArenaSize / 1024,
                    tensor_arena_in_psram ? "PSRAM" : "internal RAM");
    return false;
  }
  size_t used = interpreter->arena_used_bytes();
  mqttDebugPrintf("Tensor arena: %d/%d KB used (%s)\n", 
                  used / 1024, kTensorArenaSize / 1024,
                  tensor_arena_in_psram ? "PSRAM - SLOW" : "internal RAM - FAST");
  mqttDebugPrintln("Inference setup complete.");
  return true;
}

// Run inference using the persistent interpreter.
// 'input_data' must match the model's expected input size.
TfLiteTensor* run_inference(uint8_t* input_data, size_t input_data_size) {
  if (!g_model_loaded || interpreter == nullptr) {
    mqttDebugPrintln("run_inference: Model not loaded or interpreter not initialized.");
    return nullptr;
  }
  // Get the model's input tensor.
  TfLiteTensor* input = interpreter->input(0);
  if (input == nullptr) {
    mqttDebugPrintln("run_inference: input tensor is nullptr.");
    return nullptr;
  }
  if (input_data_size != input->bytes) {
    mqttDebugPrintf("Input data size (%d) does not match model input size (%d)\n",
                  input_data_size, input->bytes);
    return nullptr;  // or handle the error as needed
  }
  // Copy the input data into the input tensor.
  memcpy(input->data.uint8, input_data, input_data_size);
  // Run inference.
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    mqttDebugPrintf("Invoke failed");
    return nullptr;
  }
  // Get the output tensor.
  TfLiteTensor* output = interpreter->output(0);
  if (output == nullptr) {
    mqttDebugPrintln("run_inference: output tensor is nullptr.");
    return nullptr;
  }
  // For a two-element output:
  // output->data.uint8[0] is the score for "prey"
  // output->data.uint8[1] is the score for "not_prey"
  // output->data.uint8[2] is the score for "not_cat" if used

  return output;
}

static inline float dequantize_u8_prob(const TfLiteTensor* tensor, uint8_t v) {
  // For quantized softmax outputs this typically yields ~[0..1].
  const float scale = tensor->params.scale;
  const int zp = tensor->params.zero_point;
  return (static_cast<int>(v) - zp) * scale;
}

static inline float dequantize_i8_value(const TfLiteTensor* tensor, int8_t v) {
  const float scale = tensor->params.scale;
  const int zp = tensor->params.zero_point;
  return (static_cast<int>(v) - zp) * scale;
}

static inline float sigmoidf_safe(float x) {
  // Guard against extreme values to avoid overflow.
  if (x > 16.0f) return 0.9999999f;
  if (x < -16.0f) return 0.0000001f;
  return 1.0f / (1.0f + expf(-x));
}

// -------- JPEG Decode Infrastructure --------
// Allocate for FULL VGA size to be safe (esp_jpg_decode scaling may not reduce output)
// Once working, we can optimize buffer sizes
static uint8_t* g_rgb_buf = nullptr;      // RGB888 buffer for VGA (640*480*3 = 921600 bytes)
static uint8_t* g_decoded_gray_buf = nullptr;  // Grayscale buffer (640*480 = 307200 bytes)
static uint8_t* g_rgb_buf_espjpeg = nullptr;   // Aligned RGB buffer for ESP32_JPEG decoder
static uint8_t* g_rgb_buf_espjpeg_base = nullptr; // Base pointer for aligned alloc
static const int DECODE_WIDTH = 320;   // QVGA width to speed decode
static const int DECODE_HEIGHT = 240;  // QVGA height
static const int SENSOR_WIDTH = 320;
static const int SENSOR_HEIGHT = 240;
static const size_t DECODE_RGB_SIZE = DECODE_WIDTH * DECODE_HEIGHT * 3;   // 230400 bytes
static const size_t DECODE_GRAY_SIZE = DECODE_WIDTH * DECODE_HEIGHT;      // 76800 bytes

// Context for esp_jpg_decode callbacks
typedef struct {
  const uint8_t* jpg_data;
  size_t jpg_len;
  uint8_t* rgb_out;
  int width;
  int height;
  int rgb_index;
  int blocks_seen;
  int max_x;
  int max_y;
  bool first_block_logged;
  bool skipped_block;
} decode_ctx_t;

// Reader callback - feeds JPEG data to decoder
static size_t decode_reader(void* arg, size_t index, uint8_t* buf, size_t len) {
  decode_ctx_t* ctx = (decode_ctx_t*)arg;

  // If we've already read everything, signal EOF
  if (index >= ctx->jpg_len) {
    return 0;
  }

  size_t remaining = ctx->jpg_len - index;
  if (len > remaining) {
    len = remaining;
  }

  if (len > 0) {
    memcpy(buf, ctx->jpg_data + index, len);
  }

  return len;
}

// Writer callback - receives decoded RGB blocks
static bool decode_writer(void* arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t* data) {
  decode_ctx_t* ctx = (decode_ctx_t*)arg;

  ctx->blocks_seen++;
  if (!ctx->first_block_logged) {
    mqttDebugPrintf("decode_writer first block: x=%d y=%d w=%d h=%d\n", x, y, w, h);
    ctx->first_block_logged = true;
  }
  
  // Safety: if block extends beyond our expected buffer, skip entirely
  // This prevents heap corruption if decoder outputs larger than expected
  if (x + w > ctx->width || y + h > ctx->height) {
    ctx->skipped_block = true;
    return true;  // Skip this block but continue decoding
  }
  
  // Copy RGB888 block to output buffer at correct position
  for (uint16_t row = 0; row < h; row++) {
    uint16_t dst_y = y + row;
    for (uint16_t col = 0; col < w; col++) {
      uint16_t dst_x = x + col;
      
      int src_idx = (row * w + col) * 3;
      int dst_idx = (dst_y * ctx->width + dst_x) * 3;
      
      ctx->rgb_out[dst_idx]     = data[src_idx];     // R
      ctx->rgb_out[dst_idx + 1] = data[src_idx + 1]; // G
      ctx->rgb_out[dst_idx + 2] = data[src_idx + 2]; // B
    }
  }

  int block_max_x = x + w;
  int block_max_y = y + h;
  if (block_max_x > ctx->max_x) ctx->max_x = block_max_x;
  if (block_max_y > ctx->max_y) ctx->max_y = block_max_y;
  return true;
}

// Decode JPEG to grayscale using esp_jpg_decode
// Returns pointer to decoded grayscale data or NULL
uint8_t* jpegToGrayscale(const uint8_t* jpg_buf, size_t jpg_len, int* out_width, int* out_height) {
  // Allocate RGB buffer if needed (full VGA size to be safe)
  if (!g_rgb_buf) {
    g_rgb_buf = (uint8_t*)heap_caps_malloc(DECODE_RGB_SIZE, MALLOC_CAP_SPIRAM);
    if (!g_rgb_buf) {
      mqttDebugPrintln("Failed to allocate RGB decode buffer");
      return nullptr;
    }
    mqttDebugPrintf("Allocated RGB buffer: %d bytes\n", DECODE_RGB_SIZE);
  }
  
  // Allocate grayscale buffer if needed
  if (!g_decoded_gray_buf) {
    g_decoded_gray_buf = (uint8_t*)heap_caps_malloc(DECODE_GRAY_SIZE, MALLOC_CAP_SPIRAM);
    if (!g_decoded_gray_buf) {
      mqttDebugPrintln("Failed to allocate grayscale buffer");
      return nullptr;
    }
    mqttDebugPrintf("Allocated grayscale buffer: %d bytes\n", DECODE_GRAY_SIZE);
  }
  
  // Setup decode context for full VGA output
  decode_ctx_t ctx;
  ctx.jpg_data = jpg_buf;
  ctx.jpg_len = jpg_len;
  ctx.rgb_out = g_rgb_buf;
  ctx.width = DECODE_WIDTH;
  ctx.height = DECODE_HEIGHT;
  ctx.rgb_index = 0;
  ctx.blocks_seen = 0;
  ctx.max_x = 0;
  ctx.max_y = 0;
  ctx.first_block_logged = false;
  ctx.skipped_block = false;

  mqttDebugPrintf("jpegToGrayscale: jpg_len=%d\n", (int)jpg_len);
  Serial.printf("jpegToGrayscale: jpg_len=%d\n", (int)jpg_len);
  
  // Decode JPEG (no scaling to keep dimensions predictable)
  esp_err_t err = esp_jpg_decode(jpg_len, JPG_SCALE_NONE, decode_reader, decode_writer, &ctx);
  if (err != ESP_OK) {
    mqttDebugPrintf("esp_jpg_decode failed: %d\n", err);
    Serial.printf("esp_jpg_decode failed: %d\n", err);
    return nullptr;
  }

  mqttDebugPrintf("jpegToGrayscale: decode OK, blocks=%d, max_x=%d, max_y=%d, skipped=%d\n",
                  ctx.blocks_seen, ctx.max_x, ctx.max_y, ctx.skipped_block ? 1 : 0);
  Serial.printf("jpegToGrayscale: decode OK, blocks=%d, max_x=%d, max_y=%d, skipped=%d\n",
                ctx.blocks_seen, ctx.max_x, ctx.max_y, ctx.skipped_block ? 1 : 0);
  
  // Output is full VGA
  *out_width = DECODE_WIDTH;
  *out_height = DECODE_HEIGHT;

  // Warn if decoder delivered anything other than 0..width/height coverage
  if (ctx.max_x > DECODE_WIDTH || ctx.max_y > DECODE_HEIGHT || ctx.skipped_block) {
    mqttDebugPrintf("jpegToGrayscale WARN: max_x=%d max_y=%d skipped=%d expected<=%d,%d\n",
                    ctx.max_x, ctx.max_y, ctx.skipped_block ? 1 : 0, DECODE_WIDTH, DECODE_HEIGHT);
  }
  
  // Convert RGB888 to grayscale - just take R channel (all channels same with grayscale effect)
  for (int i = 0; i < DECODE_GRAY_SIZE; i++) {
    g_decoded_gray_buf[i] = g_rgb_buf[i * 3];
  }
  
  return g_decoded_gray_buf;
}

// Decode using ESP32_JPEG library (aligned, typically faster)
uint8_t* jpegToGrayscaleEsp32Jpeg(const uint8_t* jpg_buf, size_t jpg_len, int* out_width, int* out_height) {
  // Allocate aligned RGB buffer if needed (manual align to 16 bytes)
  if (!g_rgb_buf_espjpeg) {
    size_t alloc_size = DECODE_RGB_SIZE + 16; // extra for alignment
    g_rgb_buf_espjpeg_base = (uint8_t*)heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
    if (!g_rgb_buf_espjpeg_base) {
      mqttDebugPrintln("ESP32_JPEG: Failed to allocate RGB base buffer");
      return nullptr;
    }
    uintptr_t addr = (uintptr_t)g_rgb_buf_espjpeg_base;
    uintptr_t aligned = (addr + 15) & ~((uintptr_t)0x0F);
    g_rgb_buf_espjpeg = (uint8_t*)aligned;
    mqttDebugPrintf("ESP32_JPEG: Allocated aligned RGB buffer: %d bytes (base=%p aligned=%p)\n", DECODE_RGB_SIZE, g_rgb_buf_espjpeg_base, g_rgb_buf_espjpeg);
  }

  // Allocate grayscale buffer if needed
  if (!g_decoded_gray_buf) {
    g_decoded_gray_buf = (uint8_t*)heap_caps_malloc(DECODE_GRAY_SIZE, MALLOC_CAP_SPIRAM);
    if (!g_decoded_gray_buf) {
      mqttDebugPrintln("ESP32_JPEG: Failed to allocate grayscale buffer");
      return nullptr;
    }
    mqttDebugPrintf("ESP32_JPEG: Allocated grayscale buffer: %d bytes\n", DECODE_GRAY_SIZE);
  }

  mqttDebugPrintf("ESP32_JPEG: decode start, jpg_len=%d\n", (int)jpg_len);

  // Configure decoder
  jpeg_dec_config_t config = {
    .output_type = JPEG_RAW_TYPE_RGB888,
    .rotate = JPEG_ROTATE_0D,
  };

  jpeg_dec_handle_t *dec = jpeg_dec_open(&config);
  if (!dec) {
    mqttDebugPrintln("ESP32_JPEG: jpeg_dec_open failed");
    return nullptr;
  }

  jpeg_dec_io_t *io = (jpeg_dec_io_t*)calloc(1, sizeof(jpeg_dec_io_t));
  jpeg_dec_header_info_t *info = (jpeg_dec_header_info_t*)calloc(1, sizeof(jpeg_dec_header_info_t));
  if (!io || !info) {
    mqttDebugPrintln("ESP32_JPEG: alloc io/header failed");
    if (io) free(io);
    if (info) free(info);
    jpeg_dec_close(dec);
    return nullptr;
  }

  io->inbuf = (unsigned char*)jpg_buf;
  io->inbuf_len = jpg_len;

  jpeg_error_t ret = jpeg_dec_parse_header(dec, io, info);
  if (ret != JPEG_ERR_OK) {
    mqttDebugPrintf("ESP32_JPEG: parse header failed %d\n", (int)ret);
    free(io);
    free(info);
    jpeg_dec_close(dec);
    return nullptr;
  }

  int expected_len = info->width * info->height * 3;
  if (expected_len > (int)DECODE_RGB_SIZE) {
    mqttDebugPrintf("ESP32_JPEG: output too large %dx%d (bytes=%d)\n", info->width, info->height, expected_len);
    free(io);
    free(info);
    jpeg_dec_close(dec);
    return nullptr;
  }

  io->outbuf = g_rgb_buf_espjpeg;
  int consumed = io->inbuf_len - io->inbuf_remain;
  io->inbuf = (unsigned char*)jpg_buf + consumed;
  io->inbuf_len = io->inbuf_remain;

  ret = jpeg_dec_process(dec, io);
  if (ret != JPEG_ERR_OK) {
    mqttDebugPrintf("ESP32_JPEG: decode failed %d\n", (int)ret);
    free(io);
    free(info);
    jpeg_dec_close(dec);
    return nullptr;
  }

  *out_width = info->width;
  *out_height = info->height;

  int gray_pixels = info->width * info->height;
  if (gray_pixels > (int)DECODE_GRAY_SIZE) {
    mqttDebugPrintf("ESP32_JPEG: gray buffer too small %d px\n", gray_pixels);
    free(io);
    free(info);
    jpeg_dec_close(dec);
    return nullptr;
  }

  for (int i = 0; i < gray_pixels; i++) {
    g_decoded_gray_buf[i] = g_rgb_buf_espjpeg[i * 3];
  }

  mqttDebugPrintf("ESP32_JPEG: decode OK, w=%d h=%d\n", *out_width, *out_height);

  free(io);
  free(info);
  jpeg_dec_close(dec);
  return g_decoded_gray_buf;
}

// Fallback decode using esp32-camera's fmt2rgb888 (internal JPEG decoder)
uint8_t* jpegToGrayscaleFallback(const uint8_t* jpg_buf, size_t jpg_len, int* out_width, int* out_height) {
  // Allocate RGB buffer if needed (full VGA size to be safe)
  if (!g_rgb_buf) {
    g_rgb_buf = (uint8_t*)heap_caps_malloc(DECODE_RGB_SIZE, MALLOC_CAP_SPIRAM);
    if (!g_rgb_buf) {
      mqttDebugPrintln("Fallback: Failed to allocate RGB decode buffer");
      return nullptr;
    }
    mqttDebugPrintf("Fallback: Allocated RGB buffer: %d bytes\n", DECODE_RGB_SIZE);
  }

  // Allocate grayscale buffer if needed
  if (!g_decoded_gray_buf) {
    g_decoded_gray_buf = (uint8_t*)heap_caps_malloc(DECODE_GRAY_SIZE, MALLOC_CAP_SPIRAM);
    if (!g_decoded_gray_buf) {
      mqttDebugPrintln("Fallback: Failed to allocate grayscale buffer");
      return nullptr;
    }
    mqttDebugPrintf("Fallback: Allocated grayscale buffer: %d bytes\n", DECODE_GRAY_SIZE);
  }

  mqttDebugPrintf("Fallback: fmt2rgb888 decode start, jpg_len=%d\n", (int)jpg_len);
  bool ok = fmt2rgb888(jpg_buf, jpg_len, PIXFORMAT_JPEG, g_rgb_buf);
  if (!ok) {
    mqttDebugPrintln("Fallback: fmt2rgb888 failed");
    return nullptr;
  }

  // Assume full VGA output
  *out_width = DECODE_WIDTH;
  *out_height = DECODE_HEIGHT;

  // Convert RGB888 to grayscale (use R channel)
  for (int i = 0; i < DECODE_GRAY_SIZE; i++) {
    g_decoded_gray_buf[i] = g_rgb_buf[i * 3];
  }

  mqttDebugPrintf("Fallback: fmt2rgb888 decode OK, w=%d h=%d\n", *out_width, *out_height);
  return g_decoded_gray_buf;
}

void captureAndSendImage() {
  if (cameraInitStatus != ESP_OK) {
    mqttDebugPrintln("captureAndSendImage: camera not initialized");
    return;
  }
  captureStartTime = millis();
  irDisableForCritical();
  delay(5); // Short delay to allow camera to adjust to lighting change

  // First capture to flush the old frame from the buffer
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    esp_camera_fb_return(fb);
  }

  // Second capture to get the current frame (JPEG with grayscale effect)
  fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    mqttDebugPrintln("Camera capture failed");
    irEnableAfterCritical();
    return;
  }

  irEnableAfterCritical();  // Re-enable IR as soon as capture completes
  captureEndTime = millis();

  // --- SERVER PATH: Publish raw JPEG immediately (no cropping) ---
  if (inferenceMode == INFERENCE_MODE_SERVER || inferenceMode == INFERENCE_MODE_BOTH) {
    if (client.connected()) {
      if (!client.publish(IMAGE_TOPIC, fb->buf, fb->len, true)) {
        mqttDebugPrintln("Failed to send image to server");
      }
    }
    sendEndTime = millis();
    // Full timing printed in handleServerInference() when server responds
  }

  // --- LOCAL INFERENCE PATH: Decode JPEG at 1/4 scale → crop center 96x96 → inference ---
  if (inferenceMode == INFERENCE_MODE_LOCAL || inferenceMode == INFERENCE_MODE_BOTH) {
    mqttDebugPrintf("LOCAL: starting decode, fb->len=%d\n", fb->len);
    Serial.printf("LOCAL: starting decode, fb->len=%d\n", fb->len);
    decodeStartTime = millis();
    
    // Decode JPEG to grayscale (QVGA 320x240)
    int decoded_w = 0, decoded_h = 0;
    size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t heap_min_before = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    mqttDebugPrintf("heap before decode: %u free / %u min\n", (unsigned)heap_before, (unsigned)heap_min_before);

    // Try fast ESP32_JPEG decoder first, fallback to fmt2rgb888 if needed
    uint8_t *gray_buf = jpegToGrayscaleEsp32Jpeg(fb->buf, fb->len, &decoded_w, &decoded_h);
    if (!gray_buf || decoded_w != DECODE_WIDTH || decoded_h != DECODE_HEIGHT) {
      mqttDebugPrintln("Primary ESP32_JPEG decode failed or size mismatch, trying fmt2rgb888 fallback");
      gray_buf = jpegToGrayscaleFallback(fb->buf, fb->len, &decoded_w, &decoded_h);
    }
    size_t heap_after = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t heap_min_after = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    
    decodeEndTime = millis();
    mqttDebugPrintf("LOCAL: decode done, gray_buf=%p, w=%d, h=%d\n", gray_buf, decoded_w, decoded_h);
    
    if (gray_buf && decoded_w == DECODE_WIDTH && decoded_h == DECODE_HEIGHT) {
      // Crop centered 192x192 square from decoded QVGA (320x240), then resize to 96x96.
      const int targetSize = 96;
      const int cropSize = 192;
      int left = (decoded_w - cropSize) / 2;
      int top  = (decoded_h - cropSize) / 2;
      
      // Allocate inference buffer
      uint8_t *inference_buf = (uint8_t*)heap_caps_malloc(targetSize * targetSize, MALLOC_CAP_SPIRAM);
      if (inference_buf) {
        // Resize cropSize x cropSize to 96x96 using nearest neighbor.
        const float scale = (float)cropSize / (float)targetSize;
        for (int y = 0; y < targetSize; y++) {
          int srcY = top + (int)(y * scale);
          if (srcY < 0) srcY = 0;
          if (srcY >= decoded_h) srcY = decoded_h - 1;
          for (int x = 0; x < targetSize; x++) {
            int srcX = left + (int)(x * scale);
            if (srcX < 0) srcX = 0;
            if (srcX >= decoded_w) srcX = decoded_w - 1;
            inference_buf[y * targetSize + x] = gray_buf[srcY * decoded_w + srcX];
          }
        }
        cropResizeEndTime = millis();
        bool esp32PreyDetected = false;
        String esp32Result = "";
        uint8_t max_score = 0;
        float conf = NAN;
        ExitMetrics exitMetrics;
        if (!isExitLike96(inference_buf, targetSize, targetSize, &exitMetrics)) {
          TfLiteTensor* inference_result = run_inference(inference_buf, targetSize * targetSize);

          if (!inference_result) {
            esp32Result = currentModelMeta.numberOfLabels == 3 ? "not_cat" : "not_prey";
            esp32PreyDetected = false;
            max_score = 0;
            conf = 0.0f;
          } else {
            int num_scores = inference_result->dims->data[inference_result->dims->size - 1];

            // Two supported output modes:
            // - uint8 probs: N>=2 scores (prey, not_prey, [not_cat])
            // - int8 logits_margin: N==1 score (prey_logit - not_prey_logit)

            if (inference_result->type == kTfLiteInt8 && num_scores == 1) {
              const int8_t q_margin = inference_result->data.int8[0];
              const float margin = dequantize_i8_value(inference_result, q_margin);
              const float thr = currentModelMeta.threshold;  // margin threshold
              const bool preyDetected = (margin >= thr);
              const float prey_prob = sigmoidf_safe(margin);

              esp32PreyDetected = preyDetected;
              esp32Result = preyDetected ? "prey" : "not_prey";
              conf = prey_prob;

              mqttDebugPrintf("LOCAL: margin=%.3f thr=%.3f prey=%d prey_prob=%.3f\n",
                              margin, thr, preyDetected ? 1 : 0, prey_prob);
            } else {
              // Default: assume uint8 softmax outputs.
              auto *scores = inference_result->data.uint8;

              // Use metadata threshold for prey instead of argmax / out0>out1.
              const float prey_prob = dequantize_u8_prob(inference_result, scores[0]);
              const float thr = currentModelMeta.threshold;
              const bool preyDetected = (prey_prob >= thr);

              int chosen_index = 0;
              float chosen_prob = prey_prob;

              if (preyDetected) {
                chosen_index = 0;
                chosen_prob = prey_prob;
              } else {
                // Choose best non-prey label.
                chosen_index = 1;
                for (int i = 2; i < num_scores; i++) {
                  if (scores[i] > scores[chosen_index]) {
                    chosen_index = i;
                  }
                }
                chosen_prob = dequantize_u8_prob(inference_result, scores[chosen_index]);
              }

              esp32PreyDetected = preyDetected;
              max_score = scores[chosen_index];
              conf = chosen_prob;

              switch (chosen_index) {
                case 0: esp32Result = "prey"; break;
                case 1: esp32Result = "not_prey"; break;
                case 2: esp32Result = "not_cat"; break;
                default: esp32Result = currentModelMeta.numberOfLabels == 3 ? "not_cat" : "not_prey"; break;
              }

              // Publish useful debug info for tuning
              mqttDebugPrintf("LOCAL: prey_prob=%.3f thr=%.3f prey=%d chosen=%d conf=%.3f\n",
                              prey_prob, thr, preyDetected ? 1 : 0, chosen_index, chosen_prob);
            }
          }

        } else {
          esp32Result = currentModelMeta.numberOfLabels == 3 ? "not_cat" : "not_prey";
          esp32PreyDetected = false;
          max_score = 255;
          mqttDebugPrintf("Image classified as EXIT-like: mean=%d gradMean=%d whitePermille=%d min=%d max=%d\n",
                          exitMetrics.mean,
                          exitMetrics.gradMean,
                          exitMetrics.whitePermille,
                          exitMetrics.minv,
                          exitMetrics.maxv);
        }

        inferenceEndTime = millis();
        
        // Publish result
        String imgName = makeImageName((uint8_t*)fb->buf, fb->len);
        lastESP32Inference = esp32Result;
        StaticJsonDocument<256> doc;
        doc["hash"] = imgName;
        doc["label"] = esp32Result;
        // Confidence: prefer the already-dequantized value (probabilities) or sigmoid(margin).
        if (isnan(conf)) {
          conf = max_score / 255.0f;
          if (interpreter && interpreter->output(0)) {
            TfLiteTensor* out = interpreter->output(0);
            conf = dequantize_u8_prob(out, max_score);
          }
        }
        doc["confidence"] = conf;
        doc["model"] = currentModelMeta.modelName; // const char*

        String payload;
        serializeJson(doc, payload);
        client.publish(ESP32_INFERENCE_TOPIC, payload.c_str(), false);
        
        // Print LOCAL timing summary
        mqttDebugPrintf("[TIMER LOCAL] barrier_to_capture=%lu, capture=%lu, decode=%lu, crop=%lu, inference=%lu, total=%lu ms | result=%s\n",
                        captureStartTime - triggerStartTime,
                        captureEndTime - captureStartTime,
                        decodeEndTime - decodeStartTime,
                        cropResizeEndTime - decodeEndTime,
                        inferenceEndTime - cropResizeEndTime,
                        inferenceEndTime - triggerStartTime,
                        esp32Result.c_str());
        
        // Make flap decision
        if (esp32PreyDetected) {
          handleFlapStateCommand("OFF");
        } else {
          handleFlapStateCommand("ON");
        }
        
        heap_caps_free(inference_buf);
      } else {
        mqttDebugPrintln("Failed to allocate inference buffer");
      }
    } else {
      mqttDebugPrintf("JPEG decode failed or unexpected size: %dx%d (expected %dx%d)\n", 
                      decoded_w, decoded_h, DECODE_WIDTH, DECODE_HEIGHT);
    }
  }

  esp_camera_fb_return(fb);
}

// Compare variance without sqrt: var_num / (N*N) <= stdMax^2
static bool isLowVariance(uint64_t sum, uint64_t sumsq, int N, uint16_t stdMax) {
	uint64_t var_num = sumsq * (uint64_t)N - sum * sum;
	uint64_t var_den = (uint64_t)N * (uint64_t)N;
	uint64_t rhs = (uint64_t)stdMax * (uint64_t)stdMax * var_den;
	return var_num <= rhs;
}

bool isExitLike96(const uint8_t* img, int w, int h, ExitMetrics* out) {
	const int N = w * h;

	// Tunables (start here, then tune from logs)
	const uint8_t  WHITE_T        = 200;  // “very bright”
	const uint16_t WHITE_FRAC_PPM = 450;  // 45% (permille)
	const uint16_t GRAD_MAX       = 3;    // mean abs-neighbor-diff threshold (see below)
	const uint16_t STD_MAX        = 18;   // only used in the white+uniform gate

	uint64_t sum = 0;
	uint64_t sumsq = 0;
	uint32_t white = 0;
	uint8_t mn = 255, mx = 0;

	for (int i = 0; i < N; i++) {
		uint8_t p = img[i];
		sum += p;
		sumsq += (uint64_t)p * (uint64_t)p;
		if (p < mn) mn = p;
		if (p > mx) mx = p;
		if (p >= WHITE_T) white++;
	}

	uint16_t mean = (uint16_t)(sum / (uint64_t)N);
	uint16_t whitePermille = (uint16_t)((white * 1000UL) / (uint32_t)N);

	// Texture: mean absolute gradient (sample every 2px for speed)
	uint32_t gradSum = 0;
	uint32_t gradCnt = 0;
	const int step = 2;

	for (int y = 0; y < h - 1; y += step) {
		const int row = y * w;
		const int rowD = (y + 1) * w;

		for (int x = 0; x < w - 1; x += step) {
			uint8_t p  = img[row + x];
			uint8_t pr = img[row + x + 1];
			uint8_t pd = img[rowD + x];

			gradSum += (p > pr) ? (p - pr) : (pr - p);
			gradSum += (p > pd) ? (p - pd) : (pd - p);
			gradCnt += 2;
		}
	}

	uint16_t gradMean = (gradCnt > 0) ? (uint16_t)(gradSum / gradCnt) : 0;

	// Decision
	bool lowTexture = (gradMean <= GRAD_MAX);

	// Extra safety for “blown out” cases
	bool veryWhite = (whitePermille >= WHITE_FRAC_PPM);
	bool uniform   = isLowVariance(sum, sumsq, N, STD_MAX);

	bool isExit = lowTexture || (veryWhite && uniform);

	if (out) {
		out->mean = mean;
		out->gradMean = gradMean;
		out->whitePermille = whitePermille;
		out->minv = mn;
		out->maxv = mx;
	}

	return isExit;
}


// Function to crop a centered 384x384 square from the frame.
// Returns a pointer to the cropped image data (allocated in PSRAM) or NULL on failure.
camera_fb_t* cropFrame(camera_fb_t *fb) {
  const int cropSize = 384;
  // Allocate a new camera_fb_t structure in PSRAM.
  camera_fb_t *cropped = (camera_fb_t*) heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_SPIRAM);
  if (!cropped) {
      Serial.println("Failed to allocate memory for cropped frame struct");
      return NULL;
  }
  if (fb->width < cropSize || fb->height < cropSize) {
      Serial.println("Frame too small for 384x384 crop");
      heap_caps_free(cropped);
      return NULL;
  }
  cropped->width  = cropSize;
  cropped->height = cropSize;
  cropped->format = PIXFORMAT_GRAYSCALE;
  cropped->len    = cropSize * cropSize * BPP;
  // Allocate the image buffer in PSRAM.
  cropped->buf = (uint8_t*) heap_caps_malloc(cropped->len, MALLOC_CAP_SPIRAM);
  if (!cropped->buf) {
      Serial.println("Failed to allocate buffer for cropped frame");
      heap_caps_free(cropped);
      return NULL;
  }

  // Calculate top-left corner of the crop.
  int left = (fb->width - cropSize) / 2;
  int top  = (fb->height - cropSize) / 2;

  // Copy pixels from the original frame to the cropped buffer.
  for (int y = 0; y < cropSize; y++) {
      int srcY = top + y;
      for (int x = 0; x < cropSize; x++) {
          int srcX = left + x;
          int srcIndex = srcY * fb->width + srcX; // Since BPP=1
          int dstIndex = y * cropSize + x;
          cropped->buf[dstIndex] = fb->buf[srcIndex];
      }
  }
  return cropped;
}

// Function to resize the cropped image from originalSize x originalSize (384x384)
// to targetSize x targetSize (96x96) using nearest-neighbor scaling.
// Returns a pointer to the resized image data (allocated in PSRAM) or NULL on failure.
camera_fb_t* resizeFrame(camera_fb_t *cropped) {
  const int targetSize = 96;
  int originalSize = cropped->width;  // should be 384
  // Allocate a new camera_fb_t structure for the resized image.
  camera_fb_t *resized = (camera_fb_t*) heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_SPIRAM);
  if (!resized) {
      Serial.println("Failed to allocate memory for resized frame struct");
      return NULL;
  }
  resized->width  = targetSize;
  resized->height = targetSize;
  resized->format = PIXFORMAT_GRAYSCALE;
  resized->len    = targetSize * targetSize * BPP;
  resized->buf = (uint8_t*) heap_caps_malloc(resized->len, MALLOC_CAP_SPIRAM);
  if (!resized->buf) {
      Serial.println("Failed to allocate buffer for resized frame");
      heap_caps_free(resized);
      return NULL;
  }

  float scale = (float)originalSize / targetSize;  // In our case, 384/96 = 4.0
  // Nearest-neighbor resizing.
  for (int y = 0; y < targetSize; y++) {
      for (int x = 0; x < targetSize; x++) {
          int srcX = (int)(x * scale);
          int srcY = (int)(y * scale);
          int srcIndex = srcY * originalSize + srcX;
          int dstIndex = y * targetSize + x;
          resized->buf[dstIndex] = cropped->buf[srcIndex];
      }
  }
  return resized;
}

// Function to save a grayscale image to SD card as raw or PGM format
// filename should include extension (e.g., "/inference/img_001.pgm")
bool saveGrayscaleToSD(uint8_t* buf, int width, int height, const char* filename) {
  if (!g_sd_ready) {
    return false;
  }
  if (!SD_MMC.exists("/inference")) {
    SD_MMC.mkdir("/inference");
  }
  
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    mqttDebugPrintln("Failed to open file for writing");
    return false;
  }
  
  // Write PGM header (Portable GrayMap format - widely supported)
  file.printf("P5\n%d %d\n255\n", width, height);
  
  // Write raw pixel data
  file.write(buf, width * height);
  file.close();
  
  return true;
}

void enableFlap() {
  digitalWrite(ENABLE_FLAP_PIN, LOW);

  flapEnabled = true;
  client.publish(FLAP_STATE_TOPIC, flapEnabled ? "ON" : "OFF", true);
}

void disableFlap() {
  digitalWrite(ENABLE_FLAP_PIN, HIGH);

  flapEnabled = false;
  client.publish(FLAP_STATE_TOPIC, flapEnabled ? "ON" : "OFF", true);
}

void handleMqttMessages(char* topic, byte* payload, unsigned int length) {
  String incomingMessage;
  for (unsigned int i = 0; i < length; i++) {
    incomingMessage += (char)payload[i];
  }
  incomingMessage.trim();
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(incomingMessage);

  if (String(topic) == COMMAND_TOPIC) {
    if (incomingMessage.equalsIgnoreCase("snapshot")) {
      triggerStartTime = millis();
      captureAndSendImage();
    } else if (incomingMessage.equalsIgnoreCase("restart")) {
      ESP.restart();
    }
  } else if (String(topic) == SET_JPEG_QUALITY_TOPIC) {
    handleJpegQualityCommand(incomingMessage);
  } else if (String(topic) == SET_BRIGHTNESS_TOPIC) {
    handleBrightnessCommand(incomingMessage);
  } else if (String(topic) == SET_CONTRAST_TOPIC) {
    handleContrastCommand(incomingMessage);
  } else if (String(topic) == SET_AWB_TOPIC) {
    handleAWBCommand(incomingMessage);
  } else if (String(topic) == SET_DETECTION_MODE_TOPIC) {
    handleDetectionModeCommand(incomingMessage);
  } else if (String(topic) == SET_FLAP_STATE_TOPIC) {
    handleFlapStateCommand(incomingMessage);
  } else if (String(topic) == SET_DEBUG_TOGGLE_TOPIC) {
    handleDebugToggleCommand(incomingMessage);
  } else if (String(topic) == SET_COOLDOWN_TOPIC) {
    handleCooldownCommand(incomingMessage);
  } else if (String(topic) == SET_AE_LEVEL_TOPIC) {
    handleAeLevelCommand(incomingMessage);
  } else if (String(topic) == SET_AEC_VALUE_TOPIC) {
    handleAecValueCommand(incomingMessage);
  } else if (String(topic) == SET_EXPOSURE_CTRL_TOPIC) {
    handleExposureCtrlCommand(incomingMessage);
  } else if (String(topic) == SET_AEC2_TOPIC) {
    handleAec2Command(incomingMessage);
  } else if (String(topic) == SET_GAIN_CTRL_TOPIC) {
    handleGainCtrlCommand(incomingMessage);
  } else if (String(topic) == SET_AGC_GAIN_TOPIC) {
    handleAgcGainCommand(incomingMessage);
  } else if (String(topic) == SET_CAMERA_PRESET_TOPIC) {
    handleCameraPresetSelect(incomingMessage);
  } else if (String(topic) == SET_CAMERA_PRESET_SAVE_TOPIC) {
    handleCameraPresetSave();
  } else if (String(topic) == INFERENCE_TOPIC) {
    handleInferenceTopic(incomingMessage);
  } else if (String(topic) == SERVER_INFERENCE_TOPIC) {
    handleServerInference(incomingMessage);
  } else if (String(topic) == SET_CAT_LOCATION_TOPIC) {
    handleCatLocationCommand(incomingMessage);
  } else if (String(topic) == SET_INFERENCE_MODE_TOPIC) {
    handleInferenceModeCommand(incomingMessage);
  } else {
    mqttDebugPrintln("unknown topic");
  }
}

void mqttDebugPrint(const char* message) {
  Serial.print(message);
  if (mqttDebugEnabled && client.connected()) {
    client.publish(DEBUG_TOPIC, message, false);
  }
}

void mqttDebugPrintln(const char* message) {
  Serial.println(message);
  if (mqttDebugEnabled && client.connected()) {
    String msg = String(message) + "\n";
    client.publish(DEBUG_TOPIC, msg.c_str(), false);
  }
}

void mqttDebugPrintf(const char* format, ...) {
  // Define a buffer to hold the formatted message
  char buffer[256];  // Adjust the size as needed

  // Initialize the variable argument list
  va_list args;
  va_start(args, format);

  // Format the string and store it in buffer
  vsnprintf(buffer, sizeof(buffer), format, args);

  // Clean up the variable argument list
  va_end(args);

  // Output to Serial Monitor
  Serial.print(buffer);

  // Publish to MQTT if connected and enabled
  if (mqttDebugEnabled && client.connected()) {
    client.publish(DEBUG_TOPIC, buffer, false);
  }
}

void publishDiscoveryConfigs() {
  // Calculate capacity needed for the JSON documents
  const size_t capacity = JSON_OBJECT_SIZE(10) + 512;

  // IR Barrier Binary Sensor
  String irSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/" + DEVICE_NAME + "/ir_barrier/config";
  DynamicJsonDocument irSensorConfig(capacity);
  irSensorConfig["name"] = "IR Barrier";
  irSensorConfig["state_topic"] = IR_BARRIER_TOPIC;
  irSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_ir_barrier";
  irSensorConfig["device_class"] = "motion";
  irSensorConfig["payload_on"] = "broken";
  irSensorConfig["payload_off"] = "clear";
  JsonObject deviceInfo = irSensorConfig.createNestedObject("device");
  deviceInfo["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfo["name"] = "Cat Flap";
  deviceInfo["model"] = "AI Cat Flap";
  deviceInfo["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  deviceInfo["sw_version"] = DEVICE_SW_VERSION;
  deviceInfo["configuration_url"] = "http://192.168.1.163:5000/classify";
  String irSensorConfigPayload;
  serializeJson(irSensorConfig, irSensorConfigPayload);
  client.publish(irSensorConfigTopic.c_str(), irSensorConfigPayload.c_str(), true);

  // Flap State Sensor
  String flapStateConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/" + DEVICE_NAME + "/flap_state/config";
  DynamicJsonDocument flapStateConfig(capacity);
  flapStateConfig["name"] = "Cat Flap State";
  flapStateConfig["state_topic"] = FLAP_STATE_TOPIC;
  flapStateConfig["device_class"] = "door";
  flapStateConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_flap_state";
  JsonObject deviceInfo2 = flapStateConfig.createNestedObject("device");
  deviceInfo2["identifiers"] = DEVICE_UNIQUE_ID;
  String flapStateConfigPayload;
  serializeJson(flapStateConfig, flapStateConfigPayload);
  client.publish(flapStateConfigTopic.c_str(), flapStateConfigPayload.c_str(), true);

  // Flap Control Switch
  String flapSwitchConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/flap_state/config";
  DynamicJsonDocument flapSwitchConfig(capacity);
  flapSwitchConfig["name"] = "Allow Entry";
  flapSwitchConfig["command_topic"] = SET_FLAP_STATE_TOPIC;
  flapSwitchConfig["state_topic"] = FLAP_STATE_TOPIC;
  flapSwitchConfig["payload_on"] = "ON";
  flapSwitchConfig["payload_off"] = "OFF";
  flapSwitchConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_flap_control";
  JsonObject deviceInfo3 = flapSwitchConfig.createNestedObject("device");
  deviceInfo3["identifiers"] = DEVICE_UNIQUE_ID;
  String flapSwitchConfigPayload;
  serializeJson(flapSwitchConfig, flapSwitchConfigPayload);
  client.publish(flapSwitchConfigTopic.c_str(), flapSwitchConfigPayload.c_str(), true);

  // Detection Mode Switch
  String detectionModeConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/detection_mode/config";
  DynamicJsonDocument detectionModeConfig(capacity);
  detectionModeConfig["name"] = "Detection Mode";
  detectionModeConfig["command_topic"] = SET_DETECTION_MODE_TOPIC;
  detectionModeConfig["state_topic"] = DETECTION_MODE_TOPIC;
  detectionModeConfig["payload_on"] = "ON";
  detectionModeConfig["payload_off"] = "OFF";
  detectionModeConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_detection_mode";
  JsonObject deviceInfoDetection = detectionModeConfig.createNestedObject("device");
  deviceInfoDetection["identifiers"] = DEVICE_UNIQUE_ID;
  String detectionModeConfigPayload;
  serializeJson(detectionModeConfig, detectionModeConfigPayload);
  client.publish(detectionModeConfigTopic.c_str(), detectionModeConfigPayload.c_str(), true);

  // Cat Location Sensor
  String catLocationConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/" + DEVICE_NAME + "/cat_location/config";
  DynamicJsonDocument catLocationConfig(capacity);
  catLocationConfig["name"] = "Cat Location";
  catLocationConfig["state_topic"] = CAT_LOCATION_TOPIC;
  catLocationConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_cat_location";
  catLocationConfig["device_class"] = "presence";
  JsonObject deviceInfoCatLocation = catLocationConfig.createNestedObject("device");
  deviceInfoCatLocation["identifiers"] = DEVICE_UNIQUE_ID;
  String catLocationConfigPayload;
  serializeJson(catLocationConfig, catLocationConfigPayload);
  client.publish(catLocationConfigTopic.c_str(), catLocationConfigPayload.c_str(), true);

  // Set Location Switch
  String setLocationSwitchConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/cat_location/config";
  DynamicJsonDocument setLocationSwitchConfig(capacity);
  setLocationSwitchConfig["name"] = "Cat Home";
  setLocationSwitchConfig["command_topic"] = SET_CAT_LOCATION_TOPIC;
  setLocationSwitchConfig["state_topic"] = CAT_LOCATION_TOPIC;
  setLocationSwitchConfig["payload_on"] = "ON";
  setLocationSwitchConfig["payload_off"] = "OFF";
  setLocationSwitchConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_set_location";
  JsonObject deviceInfoSetLocation = setLocationSwitchConfig.createNestedObject("device");
  deviceInfoSetLocation["identifiers"] = DEVICE_UNIQUE_ID;
  String setLocationSwitchConfigPayload;
  serializeJson(setLocationSwitchConfig, setLocationSwitchConfigPayload);
  client.publish(setLocationSwitchConfigTopic.c_str(), setLocationSwitchConfigPayload.c_str(), true);

  // Debug Toggle Switch
  String debugToggleConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/debug_toggle/config";
  DynamicJsonDocument debugToggleConfig(capacity);
  debugToggleConfig["name"] = "Debug Output";
  debugToggleConfig["command_topic"] = SET_DEBUG_TOGGLE_TOPIC;
  debugToggleConfig["state_topic"] = DEBUG_TOGGLE_TOPIC;
  debugToggleConfig["payload_on"] = "ON";
  debugToggleConfig["payload_off"] = "OFF";
  debugToggleConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_debug_toggle";
  debugToggleConfig["entity_category"] = "diagnostic";
  JsonObject deviceInfoDebugToggle = debugToggleConfig.createNestedObject("device");
  deviceInfoDebugToggle["identifiers"] = DEVICE_UNIQUE_ID;
  String debugToggleConfigPayload;
  serializeJson(debugToggleConfig, debugToggleConfigPayload);
  client.publish(debugToggleConfigTopic.c_str(), debugToggleConfigPayload.c_str(), true);

   // Camera Entity
  String cameraConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/camera/" + DEVICE_NAME + "/camera/config";
  DynamicJsonDocument cameraConfig(capacity);
  cameraConfig["name"] = "Last Image";
  cameraConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_camera";
  cameraConfig["topic"] = IMAGE_TOPIC;
  JsonObject deviceInfo4 = cameraConfig.createNestedObject("device");
  deviceInfo4["identifiers"] = DEVICE_UNIQUE_ID;
  String cameraConfigPayload;
  serializeJson(cameraConfig, cameraConfigPayload);
  client.publish(cameraConfigTopic.c_str(), cameraConfigPayload.c_str(), true);

  // JPEG Quality Number
  String jpegQualityConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/jpeg_quality/config";
  DynamicJsonDocument jpegQualityConfig(capacity);
  jpegQualityConfig["name"] = "Camera JPEG Quality";
  jpegQualityConfig["command_topic"] = "catflap/jpeg_quality/set";
  //jpegQualityConfig["entity_category"] = "configuration";
  jpegQualityConfig["state_topic"] = "catflap/jpeg_quality";
  jpegQualityConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_jpeg_quality";
  jpegQualityConfig["min"] = 0;
  jpegQualityConfig["max"] = 63;
  jpegQualityConfig["step"] = 1;
  JsonObject deviceInfoJpegQuality = jpegQualityConfig.createNestedObject("device");
  deviceInfoJpegQuality["identifiers"] = DEVICE_UNIQUE_ID;
  String jpegQualityConfigPayload;
  serializeJson(jpegQualityConfig, jpegQualityConfigPayload);
  client.publish(jpegQualityConfigTopic.c_str(), jpegQualityConfigPayload.c_str(), true);

  // AE Level Number (-2..2)
  String aeLevelConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/ae_level/config";
  DynamicJsonDocument aeLevelConfig(capacity);
  aeLevelConfig["name"] = "AE Level";
  aeLevelConfig["command_topic"] = "catflap/ae_level/set";
  aeLevelConfig["state_topic"] = "catflap/ae_level";
  aeLevelConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_ae_level";
  aeLevelConfig["min"] = -2;
  aeLevelConfig["max"] = 2;
  aeLevelConfig["step"] = 1;
  JsonObject deviceInfoAeLevel = aeLevelConfig.createNestedObject("device");
  deviceInfoAeLevel["identifiers"] = DEVICE_UNIQUE_ID;
  String aeLevelConfigPayload;
  serializeJson(aeLevelConfig, aeLevelConfigPayload);
  client.publish(aeLevelConfigTopic.c_str(), aeLevelConfigPayload.c_str(), true);

  // AEC Value Number (0..1200 typical OV2640 range)
  String aecValueConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/aec_value/config";
  DynamicJsonDocument aecValueConfig(capacity);
  aecValueConfig["name"] = "AE Value";
  aecValueConfig["command_topic"] = "catflap/aec_value/set";
  aecValueConfig["state_topic"] = "catflap/aec_value";
  aecValueConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_aec_value";
  aecValueConfig["min"] = 0;
  aecValueConfig["max"] = 1200;
  aecValueConfig["step"] = 1;
  JsonObject deviceInfoAecValue = aecValueConfig.createNestedObject("device");
  deviceInfoAecValue["identifiers"] = DEVICE_UNIQUE_ID;
  String aecValueConfigPayload;
  serializeJson(aecValueConfig, aecValueConfigPayload);
  client.publish(aecValueConfigTopic.c_str(), aecValueConfigPayload.c_str(), true);

  // Exposure Ctrl Switch (AEC enable)
  String exposureCtrlConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/exposure_ctrl/config";
  DynamicJsonDocument exposureCtrlConfig(capacity);
  exposureCtrlConfig["name"] = "AE Auto Exposure";
  exposureCtrlConfig["command_topic"] = "catflap/exposure_ctrl/set";
  exposureCtrlConfig["state_topic"] = "catflap/exposure_ctrl";
  exposureCtrlConfig["payload_on"] = "ON";
  exposureCtrlConfig["payload_off"] = "OFF";
  exposureCtrlConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_exposure_ctrl";
  JsonObject deviceInfoExposureCtrl = exposureCtrlConfig.createNestedObject("device");
  deviceInfoExposureCtrl["identifiers"] = DEVICE_UNIQUE_ID;
  String exposureCtrlConfigPayload;
  serializeJson(exposureCtrlConfig, exposureCtrlConfigPayload);
  client.publish(exposureCtrlConfigTopic.c_str(), exposureCtrlConfigPayload.c_str(), true);

  // AEC2 Switch (advanced exposure)
  String aec2ConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/aec2/config";
  DynamicJsonDocument aec2Config(capacity);
  aec2Config["name"] = "AE Advanced";
  aec2Config["command_topic"] = "catflap/aec2/set";
  aec2Config["state_topic"] = "catflap/aec2";
  aec2Config["payload_on"] = "ON";
  aec2Config["payload_off"] = "OFF";
  aec2Config["unique_id"] = String(DEVICE_UNIQUE_ID) + "_aec2";
  JsonObject deviceInfoAec2 = aec2Config.createNestedObject("device");
  deviceInfoAec2["identifiers"] = DEVICE_UNIQUE_ID;
  String aec2ConfigPayload;
  serializeJson(aec2Config, aec2ConfigPayload);
  client.publish(aec2ConfigTopic.c_str(), aec2ConfigPayload.c_str(), true);

  // Gain Ctrl Switch (AGC enable)
  String gainCtrlConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/gain_ctrl/config";
  DynamicJsonDocument gainCtrlConfig(capacity);
  gainCtrlConfig["name"] = "AG Auto Gain";
  gainCtrlConfig["command_topic"] = "catflap/gain_ctrl/set";
  gainCtrlConfig["state_topic"] = "catflap/gain_ctrl";
  gainCtrlConfig["payload_on"] = "ON";
  gainCtrlConfig["payload_off"] = "OFF";
  gainCtrlConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_gain_ctrl";
  JsonObject deviceInfoGainCtrl = gainCtrlConfig.createNestedObject("device");
  deviceInfoGainCtrl["identifiers"] = DEVICE_UNIQUE_ID;
  String gainCtrlConfigPayload;
  serializeJson(gainCtrlConfig, gainCtrlConfigPayload);
  client.publish(gainCtrlConfigTopic.c_str(), gainCtrlConfigPayload.c_str(), true);

  // AGC Gain Number (0..30 typical)
  String agcGainConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/agc_gain/config";
  DynamicJsonDocument agcGainConfig(capacity);
  agcGainConfig["name"] = "AG Gain";
  agcGainConfig["command_topic"] = "catflap/agc_gain/set";
  agcGainConfig["state_topic"] = "catflap/agc_gain";
  agcGainConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_agc_gain";
  agcGainConfig["min"] = 0;
  agcGainConfig["max"] = 30;
  agcGainConfig["step"] = 1;
  JsonObject deviceInfoAgcGain = agcGainConfig.createNestedObject("device");
  deviceInfoAgcGain["identifiers"] = DEVICE_UNIQUE_ID;
  String agcGainConfigPayload;
  serializeJson(agcGainConfig, agcGainConfigPayload);
  client.publish(agcGainConfigTopic.c_str(), agcGainConfigPayload.c_str(), true);

  // Camera Preset Select (day/night)
  String cameraPresetConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/select/" + DEVICE_NAME + "/camera_preset/config";
  DynamicJsonDocument cameraPresetConfig(capacity);
  cameraPresetConfig["name"] = "Preset";
  cameraPresetConfig["command_topic"] = SET_CAMERA_PRESET_TOPIC;
  cameraPresetConfig["state_topic"] = CAMERA_PRESET_TOPIC;
  cameraPresetConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_camera_preset";
  cameraPresetConfig["icon"] = "mdi:theme-light-dark";
  JsonArray presetOptions = cameraPresetConfig.createNestedArray("options");
  presetOptions.add("day");
  presetOptions.add("night");
  JsonObject deviceInfoPreset = cameraPresetConfig.createNestedObject("device");
  deviceInfoPreset["identifiers"] = DEVICE_UNIQUE_ID;
  String cameraPresetConfigPayload;
  serializeJson(cameraPresetConfig, cameraPresetConfigPayload);
  client.publish(cameraPresetConfigTopic.c_str(), cameraPresetConfigPayload.c_str(), true);

  // Camera Preset Save Button (save current settings to selected preset)
  String cameraPresetSaveConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/camera_preset_save/config";
  DynamicJsonDocument cameraPresetSaveConfig(capacity);
  cameraPresetSaveConfig["name"] = "Preset Save";
  cameraPresetSaveConfig["command_topic"] = SET_CAMERA_PRESET_SAVE_TOPIC;
  cameraPresetSaveConfig["payload_press"] = "save";
  cameraPresetSaveConfig["icon"] = "mdi:content-save";
  cameraPresetSaveConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_camera_preset_save";
  JsonObject deviceInfoPresetSave = cameraPresetSaveConfig.createNestedObject("device");
  deviceInfoPresetSave["identifiers"] = DEVICE_UNIQUE_ID;
  String cameraPresetSaveConfigPayload;
  serializeJson(cameraPresetSaveConfig, cameraPresetSaveConfigPayload);
  client.publish(cameraPresetSaveConfigTopic.c_str(), cameraPresetSaveConfigPayload.c_str(), true);

  // Brightness Number
  String brightnessConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/brightness/config";
  DynamicJsonDocument brightnessConfig(capacity);
  brightnessConfig["name"] = "Camera Brightness";
  brightnessConfig["command_topic"] = "catflap/brightness/set";
  //brightnessConfig["entity_category"] = "configuration";
  brightnessConfig["state_topic"] = "catflap/brightness";
  brightnessConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_brightness";
  brightnessConfig["min"] = -2;
  brightnessConfig["max"] = 2;
  brightnessConfig["step"] = 1;
  JsonObject deviceInfoBrightness = brightnessConfig.createNestedObject("device");
  deviceInfoBrightness["identifiers"] = DEVICE_UNIQUE_ID;
  String brightnessConfigPayload;
  serializeJson(brightnessConfig, brightnessConfigPayload);
  client.publish(brightnessConfigTopic.c_str(), brightnessConfigPayload.c_str(), true);

  // Contrast Number
  String contrastConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/contrast/config";
  DynamicJsonDocument contrastConfig(capacity);
  contrastConfig["name"] = "Camera Contrast";
  contrastConfig["command_topic"] = "catflap/contrast/set";
  //contrastConfig["entity_category"] = "configuration";
  contrastConfig["state_topic"] = "catflap/contrast";
  contrastConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_contrast";
  contrastConfig["min"] = -2;
  contrastConfig["max"] = 2;
  contrastConfig["step"] = 1;
  JsonObject deviceInfoContrast = contrastConfig.createNestedObject("device");
  deviceInfoContrast["identifiers"] = DEVICE_UNIQUE_ID;
  String contrastConfigPayload;
  serializeJson(contrastConfig, contrastConfigPayload);
  client.publish(contrastConfigTopic.c_str(), contrastConfigPayload.c_str(), true);

  // AWB Switch
  String awbConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/awb/config";
  DynamicJsonDocument awbConfig(capacity);
  awbConfig["name"] = "Camera AWB";
  awbConfig["command_topic"] = "catflap/awb/set";
  //awbConfig["entity_category"] = "configuration";
  awbConfig["state_topic"] = "catflap/awb";
  awbConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_awb";
  awbConfig["payload_on"] = "ON";
  awbConfig["payload_off"] = "OFF";
  JsonObject deviceInfoAWB = awbConfig.createNestedObject("device");
  deviceInfoAWB["identifiers"] = DEVICE_UNIQUE_ID;
  String awbConfigPayload;
  serializeJson(awbConfig, awbConfigPayload);
  client.publish(awbConfigTopic.c_str(), awbConfigPayload.c_str(), true);

  // Snapshot Button
  String snapshotButtonConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/snapshot/config";
  DynamicJsonDocument snapshotButtonConfig(capacity);
  snapshotButtonConfig["name"] = "Snapshot";
  snapshotButtonConfig["command_topic"] = COMMAND_TOPIC;
  snapshotButtonConfig["payload_press"] = "snapshot";
  snapshotButtonConfig["icon"] = "mdi:camera-iris";
  snapshotButtonConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_snapshot";
  JsonObject deviceInfoSnapshot = snapshotButtonConfig.createNestedObject("device");
  deviceInfoSnapshot["identifiers"] = DEVICE_UNIQUE_ID;
  String snapshotButtonConfigPayload;
  serializeJson(snapshotButtonConfig, snapshotButtonConfigPayload);
  client.publish(snapshotButtonConfigTopic.c_str(), snapshotButtonConfigPayload.c_str(), true);

  // Restart Button
  String restartButtonConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/restart/config";
  DynamicJsonDocument restartButtonConfig(capacity);
  restartButtonConfig["name"] = "Cat Flap Restart";
  restartButtonConfig["command_topic"] = COMMAND_TOPIC;
  restartButtonConfig["payload_press"] = "restart";
  restartButtonConfig["icon"] = "mdi:restart";
  restartButtonConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_restart";
  JsonObject deviceInfoRestart = restartButtonConfig.createNestedObject("device");
  deviceInfoRestart["identifiers"] = DEVICE_UNIQUE_ID;
  String restartButtonConfigPayload;
  serializeJson(restartButtonConfig, restartButtonConfigPayload);
  client.publish(restartButtonConfigTopic.c_str(), restartButtonConfigPayload.c_str(), true);

  // Debug Sensor
  String debugSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/debug/config";
  DynamicJsonDocument debugSensorConfig(512);
  debugSensorConfig["name"] = "Debug Output";
  debugSensorConfig["state_topic"] = DEBUG_TOPIC;
  debugSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_debug";
  debugSensorConfig["icon"] = "mdi:message-text";
  debugSensorConfig["entity_category"] = "diagnostic";
  JsonObject deviceInfoDebug = debugSensorConfig.createNestedObject("device");
  deviceInfoDebug["identifiers"] = DEVICE_UNIQUE_ID;
  String debugSensorConfigPayload;
  serializeJson(debugSensorConfig, debugSensorConfigPayload);
  client.publish(debugSensorConfigTopic.c_str(), debugSensorConfigPayload.c_str(), true);

  // Alert Sensor
  String alertSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/alert/config";
  DynamicJsonDocument alertSensorConfig(512);
  alertSensorConfig["name"] = "Alert Output";
  alertSensorConfig["state_topic"] = ALERT_TOPIC;
  alertSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_alert";
  alertSensorConfig["icon"] = "mdi:message-text";
  JsonObject deviceInfoAlert = alertSensorConfig.createNestedObject("device");
  deviceInfoAlert["identifiers"] = DEVICE_UNIQUE_ID;
  String alertSensorConfigPayload;
  serializeJson(alertSensorConfig, alertSensorConfigPayload);
  client.publish(alertSensorConfigTopic.c_str(), alertSensorConfigPayload.c_str(), true);

  // Inference Sensor
  String inferenceSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/inference/config";
  DynamicJsonDocument inferenceSensorConfig(512);
  inferenceSensorConfig["name"] = "Inference Output";
  inferenceSensorConfig["state_topic"] = INFERENCE_TOPIC;
  inferenceSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_inference";
  inferenceSensorConfig["icon"] = "mdi:message-text";
  JsonObject deviceInfoInference = inferenceSensorConfig.createNestedObject("device");
  deviceInfoInference["identifiers"] = DEVICE_UNIQUE_ID;
  String inferenceSensorConfigPayload;
  serializeJson(inferenceSensorConfig, inferenceSensorConfigPayload);
  client.publish(inferenceSensorConfigTopic.c_str(), inferenceSensorConfigPayload.c_str(), true);

  // Cooldown Number Entity
  String cooldownConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/cooldown/config";
  DynamicJsonDocument cooldownConfig(512);
  cooldownConfig["name"] = "IR Barrier Cooldown";
  cooldownConfig["command_topic"] = SET_COOLDOWN_TOPIC;
  //cooldownConfig["entity_category"] = "configuration";
  cooldownConfig["state_topic"] = COOLDOWN_STATE_TOPIC;
  cooldownConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_cooldown";
  cooldownConfig["min"] = 0.0;
  cooldownConfig["max"] = 5.0;
  cooldownConfig["step"] = 0.1;
  cooldownConfig["unit_of_measurement"] = "s";
  JsonObject deviceInfoCooldown = cooldownConfig.createNestedObject("device");
  deviceInfoCooldown["identifiers"] = DEVICE_UNIQUE_ID;
  String cooldownConfigPayload;
  serializeJson(cooldownConfig, cooldownConfigPayload);
  client.publish(cooldownConfigTopic.c_str(), cooldownConfigPayload.c_str(), true);

  // Wifi signal Sensor
  String wifiSignalSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/wifi_signal/config";
  DynamicJsonDocument wifiSignalSensorConfig(512);
  wifiSignalSensorConfig["name"] = "WiFi Signal";
  wifiSignalSensorConfig["state_topic"] = WIFI_SIGNAL_TOPIC;
  wifiSignalSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_wifi_signal";
  wifiSignalSensorConfig["unit_of_measurement"] = "dBm";
  wifiSignalSensorConfig["entity_category"] = "diagnostic";
  wifiSignalSensorConfig["device_class"] = "signal_strength";
  wifiSignalSensorConfig["state_class"] = "measurement";
  JsonObject deviceInfoWifiSignal = wifiSignalSensorConfig.createNestedObject("device");
  deviceInfoWifiSignal["identifiers"] = DEVICE_UNIQUE_ID;
  String wifiSignalSensorConfigPayload;
  serializeJson(wifiSignalSensorConfig, wifiSignalSensorConfigPayload);
  client.publish(wifiSignalSensorConfigTopic.c_str(), wifiSignalSensorConfigPayload.c_str(), true);

  // IR Pulse Quality Sensor
  String pulseQualityConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/ir_pulse_quality/config";
  DynamicJsonDocument pulseQualityConfig(512);
  pulseQualityConfig["name"] = "IR Pulse Quality";
  pulseQualityConfig["state_topic"] = IR_PULSE_QUALITY_TOPIC;
  pulseQualityConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_ir_pulse_quality";
  pulseQualityConfig["unit_of_measurement"] = "%";
  pulseQualityConfig["entity_category"] = "diagnostic";
  pulseQualityConfig["icon"] = "mdi:percent";
  pulseQualityConfig["state_class"] = "measurement";
  JsonObject deviceInfoPulseQuality = pulseQualityConfig.createNestedObject("device");
  deviceInfoPulseQuality["identifiers"] = DEVICE_UNIQUE_ID;
  String pulseQualityConfigPayload;
  serializeJson(pulseQualityConfig, pulseQualityConfigPayload);
  client.publish(pulseQualityConfigTopic.c_str(), pulseQualityConfigPayload.c_str(), true);

  // IP Sensor
  String ipSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/ip/config";
  DynamicJsonDocument ipSensorConfig(512);
  ipSensorConfig["name"] = "IP Address";
  ipSensorConfig["state_topic"] = IP_TOPIC;
  ipSensorConfig["icon"] = "mdi:ip-network";
  ipSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_ip";
  ipSensorConfig["entity_category"] = "diagnostic";
  JsonObject deviceInfoIP = ipSensorConfig.createNestedObject("device");
  deviceInfoIP["identifiers"] = DEVICE_UNIQUE_ID;
  String ipSensorConfigPayload;
  serializeJson(ipSensorConfig, ipSensorConfigPayload);
  client.publish(ipSensorConfigTopic.c_str(), ipSensorConfigPayload.c_str(), true);

  // Free Heap Sensor
  String freeHeapSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/free_heap/config";
  DynamicJsonDocument freeHeapSensorConfig(512);
  freeHeapSensorConfig["name"] = "Free Heap";
  freeHeapSensorConfig["state_topic"] = FREE_HEAP_TOPIC;
  freeHeapSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_free_heap";
  freeHeapSensorConfig["unit_of_measurement"] = "B";
  freeHeapSensorConfig["entity_category"] = "diagnostic";
  freeHeapSensorConfig["device_class"] = "data_size";
  freeHeapSensorConfig["state_class"] = "measurement";
  JsonObject deviceInfoFreeHeap = freeHeapSensorConfig.createNestedObject("device");
  deviceInfoFreeHeap["identifiers"] = DEVICE_UNIQUE_ID;
  String freeHeapSensorConfigPayload;
  serializeJson(freeHeapSensorConfig, freeHeapSensorConfigPayload);
  client.publish(freeHeapSensorConfigTopic.c_str(), freeHeapSensorConfigPayload.c_str(), true);

    // Roundtrip Time Sensor
  String roundtripSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/roundtrip/config";
  DynamicJsonDocument roundtripSensorConfig(512);
  roundtripSensorConfig["name"] = "Roundtrip time";
  roundtripSensorConfig["state_topic"] = ROUNDTRIP_TOPIC;
  roundtripSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_roundtrip";
  roundtripSensorConfig["unit_of_measurement"] = "ms";
  roundtripSensorConfig["entity_category"] = "diagnostic";
  roundtripSensorConfig["state_class"] = "measurement";
  JsonObject deviceInfoRoundtrip = roundtripSensorConfig.createNestedObject("device");
  deviceInfoRoundtrip["identifiers"] = DEVICE_UNIQUE_ID;
  String roundtripSensorConfigPayload;
  serializeJson(roundtripSensorConfig, roundtripSensorConfigPayload);
  client.publish(roundtripSensorConfigTopic.c_str(), roundtripSensorConfigPayload.c_str(), true);

  // Looptime Sensor
  String looptimeSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/looptime/config";
  DynamicJsonDocument looptimeSensorConfig(512);
  looptimeSensorConfig["name"] = "Looptime";
  looptimeSensorConfig["state_topic"] = LOOPTIME_TOPIC;
  looptimeSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_looptime";
  looptimeSensorConfig["unit_of_measurement"] = "us";
  looptimeSensorConfig["entity_category"] = "diagnostic";
  looptimeSensorConfig["state_class"] = "measurement";
  JsonObject deviceInfoLooptime = looptimeSensorConfig.createNestedObject("device");
  deviceInfoLooptime["identifiers"] = DEVICE_UNIQUE_ID;
  String looptimeSensorConfigPayload;
  serializeJson(looptimeSensorConfig, looptimeSensorConfigPayload);
  client.publish(looptimeSensorConfigTopic.c_str(), looptimeSensorConfigPayload.c_str(), true);

  // Inference Mode Select
  String inferenceModeConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/select/" + DEVICE_NAME + "/inference_mode/config";
  DynamicJsonDocument inferenceModeConfig(capacity);
  inferenceModeConfig["name"] = "Inference Mode";
  inferenceModeConfig["command_topic"] = SET_INFERENCE_MODE_TOPIC;
  inferenceModeConfig["state_topic"] = INFERENCE_MODE_TOPIC;
  inferenceModeConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_inference_mode";
  inferenceModeConfig["icon"] = "mdi:brain";
  JsonArray modeOptions = inferenceModeConfig.createNestedArray("options");
  modeOptions.add("local");
  modeOptions.add("server");
  modeOptions.add("both");
  JsonObject deviceInfoInferenceMode = inferenceModeConfig.createNestedObject("device");
  deviceInfoInferenceMode["identifiers"] = DEVICE_UNIQUE_ID;
  String inferenceModeConfigPayload;
  serializeJson(inferenceModeConfig, inferenceModeConfigPayload);
  client.publish(inferenceModeConfigTopic.c_str(), inferenceModeConfigPayload.c_str(), true);
}

// TODO: Inference handled on device. compare inference from device and from mqtt
void handleInferenceTopic(String inferenceStr) {
  if (inferenceStr == "not_cat"){
    /* code */
  } else if (inferenceStr == "cat_morris_entering") {
    handleCatLocationCommand("ON");
  } else if (inferenceStr == "cat_morris_leaving") {
    handleCatLocationCommand("OFF");
  } else if (inferenceStr == "unknow_cat_entering") {
    /* code */
  } else if (inferenceStr == "prey") {
    handleFlapStateCommand("OFF");
  }
}

void handleServerInference(String serverInferenceStr) {
  unsigned long serverResponseTime = millis();

  // Server now sends JSON: {"hash","label","confidence","model"}
  String parsedLabel = serverInferenceStr;
  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, serverInferenceStr);
  if (!err) {
    parsedLabel = doc["label"] | parsedLabel;
    // Optional: capture hash/confidence/model for future use/debug
    String srvHash = doc["hash"] | "";
    double srvConf = doc["confidence"] | -1.0;
    String srvModel = doc["model"] | "";
  } else {
    mqttDebugPrintf("Server inference parse failed: %s\n", err.c_str());
  }

  // Use parsed label for comparisons/state
  lastServerInference = parsedLabel;
  totalInferences++;
  
  // Print SERVER full round-trip timing
  unsigned long serverInferenceLatency = serverResponseTime - sendEndTime;
  unsigned long serverTotalMs = serverResponseTime - triggerStartTime;
  mqttDebugPrintf("[TIMER SERVER COMPLETE] barrier_to_capture=%lu, capture=%lu, send=%lu, server_inference=%lu, total=%lu ms | result=%s\n",
                  captureStartTime - triggerStartTime,
                  captureEndTime - captureStartTime,
                  sendEndTime - captureEndTime,
                  serverInferenceLatency,
                  serverTotalMs,
                  parsedLabel.c_str());
  
  // Compare ESP32 and Server results (only if both modes active)
  if (inferenceMode == INFERENCE_MODE_BOTH && lastESP32Inference.length() > 0) {
    if (lastESP32Inference == lastServerInference) {
      inferenceMatches++;
      mqttDebugPrintf("Inference MATCH: %s\n", lastESP32Inference.c_str());
    } else {
      inferenceMismatches++;
      mqttDebugPrintf("Inference MISMATCH! ESP32=%s, Server=%s\n", 
                      lastESP32Inference.c_str(), lastServerInference.c_str());
    }
    
    // Publish comparison stats
    float accuracy = totalInferences > 0 ? (float)inferenceMatches / totalInferences * 100.0 : 0.0;
    String comparisonMsg = String("Matches: ") + inferenceMatches + 
                          " / " + totalInferences + 
                          " (" + String(accuracy, 1) + "%)";
    client.publish(INFERENCE_COMPARISON_TOPIC, comparisonMsg.c_str(), true);
  }
}

void handleDetectionModeCommand(String stateStr) {
  bool newState = stateStr.equalsIgnoreCase("ON");
  detectionModeEnabled = newState;
  client.publish(DETECTION_MODE_TOPIC, detectionModeEnabled ? "ON" : "OFF", true);
  mqttDebugPrintln("Detection mode updated");
  lastSettingsChangeTime = millis();
}

void handleFlapStateCommand(String stateStr) {
  bool setFlapEnabled = stateStr.equalsIgnoreCase("ON");
  if (setFlapEnabled){
    enableFlap();
  } else {
    disableFlap();
  }
}

void handleDebugToggleCommand(String stateStr) {
  mqttDebugEnabled = stateStr.equalsIgnoreCase("ON");
  client.publish(DEBUG_TOGGLE_TOPIC, mqttDebugEnabled ? "ON" : "OFF", true);
  mqttDebugPrintln("Debug output toggled");
}

void handleCatLocationCommand(String locationStr) {
  catLocation = locationStr.equalsIgnoreCase("ON");
  client.publish(CAT_LOCATION_TOPIC, catLocation ? "ON" : "OFF", true);
  mqttDebugPrintln("Cat location updated");
}

void handleJpegQualityCommand(String qualityStr) {
  int quality = qualityStr.toInt();
  if (quality < 0 || quality > 63) {
    mqttDebugPrintln("Invalid JPEG quality value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_quality(s, quality);
    mqttDebugPrintln("JPEG quality updated");
    client.publish("catflap/jpeg_quality", String(quality).c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleBrightnessCommand(String brightnessStr) {
  int brightness = brightnessStr.toInt();
  if (brightness < -2 || brightness > 2) {
    mqttDebugPrintln("Invalid brightness value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, brightness);
    mqttDebugPrintln("Brightness updated");
    client.publish("catflap/brightness", String(brightness).c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleContrastCommand(String contrastStr) {
  int contrast = contrastStr.toInt();
  if (contrast < -2 || contrast > 2) {
    mqttDebugPrintln("Invalid contrast value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_contrast(s, contrast);
    mqttDebugPrintln("Contrast updated");
    client.publish("catflap/contrast", String(contrast).c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleAWBCommand(String awbStateStr) {
  bool awbState = awbStateStr.equalsIgnoreCase("ON");
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_whitebal(s, awbState);
    mqttDebugPrintln("AWB updated");
    client.publish("catflap/awb", awbState ? "ON" : "OFF", true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleCooldownCommand(String cooldownStr) {
  float newCooldown = cooldownStr.toFloat();
  if (newCooldown < 0.0 || newCooldown > 5.0) {
    mqttDebugPrintln("Invalid cooldown value");
    return;
  }
  lastSettingsChangeTime = millis();  // Update settings change timer
  cooldownDuration = newCooldown;
  mqttDebugPrintf("Cooldown duration updated to %.1f seconds\n", cooldownDuration);
  publishCooldownState();
}

void handleAeLevelCommand(String levelStr) {
  int level = levelStr.toInt();
  if (level < -2 || level > 2) {
    mqttDebugPrintln("Invalid AE level value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_ae_level(s, level);
    mqttDebugPrintln("AE level updated");
    client.publish("catflap/ae_level", String(level).c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleAecValueCommand(String valueStr) {
  int value = valueStr.toInt();
  if (value < 0 || value > 1200) {
    mqttDebugPrintln("Invalid AEC value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_aec_value(s, value);
    mqttDebugPrintln("AEC value updated");
    client.publish("catflap/aec_value", String(value).c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleExposureCtrlCommand(String stateStr) {
  bool enabled = stateStr.equalsIgnoreCase("ON");
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_exposure_ctrl(s, enabled ? 1 : 0);
    mqttDebugPrintln("Exposure control updated");
    client.publish("catflap/exposure_ctrl", enabled ? "ON" : "OFF", true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleAec2Command(String stateStr) {
  bool enabled = stateStr.equalsIgnoreCase("ON");
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_aec2(s, enabled ? 1 : 0);
    mqttDebugPrintln("AEC2 updated");
    client.publish("catflap/aec2", enabled ? "ON" : "OFF", true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleGainCtrlCommand(String stateStr) {
  bool enabled = stateStr.equalsIgnoreCase("ON");
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_gain_ctrl(s, enabled ? 1 : 0);
    mqttDebugPrintln("Gain control updated");
    client.publish("catflap/gain_ctrl", enabled ? "ON" : "OFF", true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleAgcGainCommand(String valueStr) {
  int value = valueStr.toInt();
  if (value < 0 || value > 30) {
    mqttDebugPrintln("Invalid AGC gain value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_agc_gain(s, value);
    mqttDebugPrintln("AGC gain updated");
    client.publish("catflap/agc_gain", String(value).c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleIRBarrierStateChange(bool barrierBroken) {
  unsigned long currentTime = millis();
  if (barrierBroken) {
    if (currentTime - lastBarrierClearedTime >= (cooldownDuration * 1000)) {

      // Mark start of trigger chain (barrier -> inference)
      triggerStartTime = currentTime;

      // Publish IR barrier state
      client.publish(IR_BARRIER_TOPIC, "broken", true);
      
      captureAndSendImage();

    } else {
      mqttDebugPrintln("Trigger ignored due to cooldown");
    }
  } else {
    // Update on falling edge
    lastBarrierClearedTime = currentTime;

    // Publish IR barrier state
    client.publish(IR_BARRIER_TOPIC, "clear", true);
  }
}

void publishCameraSettings() {
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // Publish JPEG Quality
    int quality = s->status.quality;
    client.publish("catflap/jpeg_quality", String(quality).c_str(), true);

    // Publish Brightness
    int brightness = s->status.brightness;
    client.publish("catflap/brightness", String(brightness).c_str(), true);

    // Publish Contrast
    int contrast = s->status.contrast;
    client.publish("catflap/contrast", String(contrast).c_str(), true);

    // Publish AWB
    String awbState = s->status.awb ? "ON" : "OFF";
    client.publish("catflap/awb", awbState.c_str(), true);

    // Publish additional camera controls
    client.publish("catflap/ae_level", String(s->status.ae_level).c_str(), true);
    client.publish("catflap/aec_value", String(s->status.aec_value).c_str(), true);
    client.publish("catflap/exposure_ctrl", s->status.aec ? "ON" : "OFF", true);
    client.publish("catflap/aec2", s->status.aec2 ? "ON" : "OFF", true);
    client.publish("catflap/gain_ctrl", s->status.agc ? "ON" : "OFF", true);
    client.publish("catflap/agc_gain", String(s->status.agc_gain).c_str(), true);

    // Publish other settings as needed
  }
}

void publishDiagnostics() {
  if (millis() > (lastDiagnosticUpdateTime + DIAGNOSTIC_UPDATE_INTERVAL)){
    lastDiagnosticUpdateTime = millis();
    String wifiSignalStr = String(WiFi.RSSI());
    client.publish(WIFI_SIGNAL_TOPIC, wifiSignalStr.c_str(), true);
    String freeHeap = String(ESP.getFreeHeap());
    client.publish(FREE_HEAP_TOPIC, freeHeap.c_str(), true);
    String loopTime = String(getAndResetMeanLoopTime());
    client.publish(LOOPTIME_TOPIC, loopTime.c_str(), true);
    uint32_t avgPulseQuality = 0;
    if (g_pulseQualitySamples > 0) {
      avgPulseQuality = g_pulseQualitySum / g_pulseQualitySamples;
    }
    String pulseQualityStr = String(avgPulseQuality);
    client.publish(IR_PULSE_QUALITY_TOPIC, pulseQualityStr.c_str(), true);
    g_pulseQualitySum = 0;
    g_pulseQualitySamples = 0;
  }
}

void recordLoopTime() {
  unsigned long currentMicros = micros();

  if (loopStartTime != 0) {
    // Calculate the duration of the last loop iteration
    unsigned long loopDuration = currentMicros - loopStartTime;

    // Accumulate the total duration and increment the loop count
    loopDurationSum += loopDuration;
    loopCount++;
  }

  // Update the loop start time for the next iteration
  loopStartTime = currentMicros;
}

float getAndResetMeanLoopTime() {
  float meanLoopTime = 0.0;

  if (loopCount > 0) {
    // Calculate the mean loop time in microseconds
    meanLoopTime = (float)loopDurationSum / loopCount;
  }

  // Reset the counters for the next measurement period
  loopDurationSum = 0;
  loopCount = 0;

  return meanLoopTime;
}

void publishIPState() {
  String ipAddress = WiFi.localIP().toString();
  client.publish(IP_TOPIC, ipAddress.c_str(), true);
}

void publishCooldownState() {
  String cooldownStr = String(cooldownDuration, 1); // One decimal place
  client.publish(COOLDOWN_STATE_TOPIC, cooldownStr.c_str(), true);
}

// Call this each loop()
void updateIRBarrierState()
{
  if (!g_irSensingEnabled) return;  // masked (e.g., while taking a photo)

  const uint32_t now = millis();
  IrState reading = readIrRaw();

  // DEBUG: Show pulse counts and current reading
  if (false)
  {
    static uint32_t lastDebugPulseCount = 0;
    if (millis() - lastIrDebug > 500) {  // Check every 500ms
      lastIrDebug = millis();
      int raw = digitalRead(IR_PIN);
      uint32_t currentPulses = g_pulseCount;
      
      // Format the debug message
      char buffer[128];
      snprintf(buffer, sizeof(buffer), 
              "raw=%d, pulses=%lu, reading=%s, stable=%s",
              raw,
              (unsigned long)currentPulses,
              reading == IR_CLEAR ? "CLEAR" : "BROKEN",
              g_stable == IR_CLEAR ? "CLEAR" : "BROKEN");
      
      client.publish("catflap/ir_debug", buffer, false);
      lastDebugPulseCount = currentPulses;
    }
  }
 
  // new candidate?
  if (reading != g_candidate) {
    g_candidate = reading;
    g_lastChangeMs = now;                  // start debounce window
    return;
  }

  // candidate held long enough?
  if ((now - g_lastChangeMs) >= IR_DEBOUNCE_MS && g_candidate != g_stable) {
    // rate limit final commit (prevents oscillation in marginal sunlight)
    if ((now - g_lastStableMs) < IR_MIN_HOLD_MS) {
      // still within hold window; skip committing
      return;
    }

    g_stable = g_candidate;
    g_lastStableMs = now;

    handleIRBarrierStateChange(g_stable == IR_BROKEN);
  }
}

void setupOTA() {
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("esp32-catcam");

  // No authentication by default
  // If you want to set a password:
  // ArduinoOTA.setPassword("password123");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupWebServer() {
  // Model upload endpoint
  server.on("/upload_model", HTTP_POST, 
    [](AsyncWebServerRequest *request) {
      // Response is sent in onUpload handler
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      static File uploadFile;
      static String tempPath = "/model_temp.tflite";
      static size_t totalSize = 0;
      
      if (index == 0) {
        // Start of upload
        mqttDebugPrintln("Starting model upload...");
        totalSize = 0;
        if (!g_sd_ready) {
          mqttDebugPrintln("SD card not available for upload");
          request->send(500, "text/plain", "SD card not available");
          return;
        }
        
        // Open temp file for writing
        uploadFile = SD_MMC.open(tempPath, FILE_WRITE);
        if (!uploadFile) {
          mqttDebugPrintln("Failed to create temp model file");
          request->send(500, "text/plain", "Failed to create temp file");
          return;
        }
      }
      
      // Write chunk
      if (uploadFile) {
        uploadFile.write(data, len);
        totalSize += len;
      }
      
      if (final) {
        // End of upload
        uploadFile.close();
        mqttDebugPrintf("Model upload complete: %d bytes\n", totalSize);
        
        // Validate the model file
        File modelFile = SD_MMC.open(tempPath, FILE_READ);
        if (!modelFile) {
          mqttDebugPrintln("Failed to open uploaded model for validation");
          request->send(500, "text/plain", "Upload failed - validation error");
          SD_MMC.remove(tempPath);
          return;
        }
        
        // Check size
        size_t fileSize = modelFile.size();
        if (fileSize < 1000 || fileSize > 2 * 1024 * 1024) {  // Between 1KB and 2MB
          mqttDebugPrintf("Invalid model size: %d bytes\n", fileSize);
          modelFile.close();
          SD_MMC.remove(tempPath);
          request->send(400, "text/plain", "Invalid model size");
          return;
        }
        
        // Check TFLite magic bytes (first 4 bytes should indicate FlatBuffer)
        uint8_t header[8];
        modelFile.read(header, 8);
        modelFile.close();
        
        // TFLite files have "TFL3" at offset 4-7
        if (!(header[4] == 'T' && header[5] == 'F' && header[6] == 'L' && header[7] == '3')) {
          mqttDebugPrintln("Invalid TFLite model format");
          SD_MMC.remove(tempPath);
          request->send(400, "text/plain", "Invalid TFLite format");
          return;
        }
        
        // Model is valid, replace the old one
        if (SD_MMC.exists("/model.tflite")) {
          SD_MMC.remove("/model.tflite");
        }
        SD_MMC.rename(tempPath, "/model.tflite");
        
        // Reload the model from SD
        if (!reloadModel()) {
          mqttDebugPrintln("Failed to load the new model after upload");
          request->send(500, "text/plain", "Model uploaded but failed to load");
          return;
        }

        mqttDebugPrintln("New model installed successfully!");
        client.publish(DEBUG_TOPIC, "New model uploaded and validated", false);
        
        request->send(200, "text/plain", "Model uploaded successfully");
      }
    }
  );

  server.on("/upload_metadata", HTTP_POST,
	[](AsyncWebServerRequest *request) {
		// Response is sent in onUpload handler
	},
	[](AsyncWebServerRequest *request, String filename, size_t index,
	   uint8_t *data, size_t len, bool final) {
		static File uploadFile;
		static String tempPath = "/model_meta_temp.json";
		static size_t totalSize = 0;

		if (index == 0) {
			// Start of upload
			mqttDebugPrintln("Starting metadata upload...");
			totalSize = 0;

			if (!g_sd_ready) {
				mqttDebugPrintln("SD card not available for metadata upload");
				request->send(500, "text/plain", "SD card not available");
				return;
			}

			// Open temp file for writing
			uploadFile = SD_MMC.open(tempPath, FILE_WRITE);
			if (!uploadFile) {
				mqttDebugPrintln("Failed to create temp metadata file");
				request->send(500, "text/plain", "Failed to create temp metadata file");
				return;
			}
		}

		// Write chunk
		if (uploadFile) {
			uploadFile.write(data, len);
			totalSize += len;
		}

		if (final) {
			// End of upload
			uploadFile.close();
			mqttDebugPrintf("Metadata upload complete: %d bytes\n", totalSize);

			// Basic size sanity check (avoid huge/empty files)
			if (totalSize < 10 || totalSize > 4096) {
				mqttDebugPrintf("Invalid metadata size: %d bytes\n", totalSize);
				SD_MMC.remove(tempPath);
				request->send(400, "text/plain", "Invalid metadata size");
				return;
			}

			// Validate JSON structure
			File metaFile = SD_MMC.open(tempPath, FILE_READ);
			if (!metaFile) {
				mqttDebugPrintln("Failed to open uploaded metadata for validation");
				SD_MMC.remove(tempPath);
				request->send(500, "text/plain", "Upload failed - validation error");
				return;
			}

			StaticJsonDocument<256> doc;
			DeserializationError err = deserializeJson(doc, metaFile);
			metaFile.close();

			if (err) {
				mqttDebugPrint("Failed to parse metadata JSON: ");
				mqttDebugPrintln(err.c_str());
				SD_MMC.remove(tempPath);
				request->send(400, "text/plain", "Invalid JSON in metadata file");
				return;
			}

			// Check required fields and types
			if (!doc.containsKey("model_name") ||
				!doc.containsKey("number_of_labels") ||
				!doc.containsKey("threshold_value")) {
				mqttDebugPrintln("Metadata JSON missing required fields");
				SD_MMC.remove(tempPath);
				request->send(400, "text/plain",
				              "Missing fields (model_name, number_of_labels, threshold_value)");
				return;
			}

			// (Optional) light type sanity checks
			if (!doc["model_name"].is<const char*>() ||
			    !doc["number_of_labels"].is<int>() ||
			    (!doc["threshold_value"].is<float>() &&
			     !doc["threshold_value"].is<double>())) {
				mqttDebugPrintln("Metadata JSON fields have wrong types");
				SD_MMC.remove(tempPath);
				request->send(400, "text/plain", "Wrong field types in metadata JSON");
				return;
			}

			// Metadata is valid, replace the old one
			if (SD_MMC.exists("/model_meta.json")) {
				SD_MMC.remove("/model_meta.json");
			}
			SD_MMC.rename(tempPath, "/model_meta.json");

			// Reload metadata into RAM
			if (loadModelMetadata("/model_meta.json", currentModelMeta)) {
				mqttDebugPrintln("New metadata installed and loaded successfully!");
				mqttDebugPrintf("Model: %s, labels: %d, threshold: %.3f\n",
				                currentModelMeta.modelName.c_str(),
				                currentModelMeta.numberOfLabels,
				                currentModelMeta.threshold);
				client.publish(DEBUG_TOPIC,
				               "New model metadata uploaded and validated", false);
				request->send(200, "text/plain", "Metadata uploaded successfully");
			} else {
				mqttDebugPrintln("Metadata file stored but failed to load");
				request->send(500, "text/plain",
				              "Metadata stored but failed to load (see logs)");
			}
		}
	}
);

  
  // Simple health check endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String status = "OK";
    status += "\nSD Card ready: " + String(g_sd_ready ? "Yes" : "No");
    status += "\nModel: " + String(g_model_loaded ? "Loaded" : "Not Loaded");
    status += "\nFree Heap: " + String(ESP.getFreeHeap());
    status += "\nUptime: " + String(millis() / 1000) + "s";
    request->send(200, "text/plain", status);
  });
  
  // List inference images
  server.on("/inference", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><head><title>Inference Images</title></head><body>";
    html += "<h1>Inference Images</h1><ul>";
    if (!g_sd_ready) {
      // Try one remount attempt here
      if (SD_MMC.begin("/sdcard", true)) {
        g_sd_ready = true;
        if (!SD_MMC.exists("/inference")) {
          SD_MMC.mkdir("/inference");
        }
      }
    }
    if (!g_sd_ready) {
      request->send(500, "text/plain", "SD card not available");
      return;
    }
    File dir = SD_MMC.open("/inference");
    if (!dir || !dir.isDirectory()) {
      request->send(500, "text/plain", "SD card not available or /inference directory missing");
      return;
    }
    File file = dir.openNextFile();
    while (file) {
      String name = file.name();
      html += "<li><a href=\"/inference/" + name + "\">" + name + "</a></li>";
      file = dir.openNextFile();
    }
    html += "</ul></body></html>";
    request->send(200, "text/html", html);
  });

  // Serve individual inference images
  server.on("/inference/*", HTTP_GET, [](AsyncWebServerRequest *request) {
    String path = request->url();
    
    if (!SD_MMC.exists(path)) {
      request->send(404, "text/plain", "File not found");
      return;
    }
    
    request->send(SD_MMC, path, "image/x-portable-graymap");
  });

  server.begin();
  mqttDebugPrintln("HTTP server started");
  Serial.println("HTTP server started on port 80");
}

void saveSettingsToEEPROM() {
  mqttDebugPrintln("Saving settings to EEPROM...");
  
  bool wasEnabled = g_irSensingEnabled;
  if (wasEnabled) {
    irDisableForCritical();
  }

  // Save the version number
  EEPROM.write(EEPROM_ADDR_VERSION, EEPROM_VERSION);

  EEPROM.write(EEPROM_ADDR_DETECTION_MODE, detectionModeEnabled ? 1 : 0);

  // Save Cooldown Duration
  EEPROM.write(EEPROM_ADDR_COOLDOWN, (int)(cooldownDuration * 10));

  // Save cat location (1 = home/ON, 0 = away/OFF)
  EEPROM.write(EEPROM_ADDR_CAT_LOCATION, catLocation ? 1 : 0);

  // Save Inference Mode
  EEPROM.write(EEPROM_ADDR_INFERENCE_MODE, inferenceMode);

  // Save camera settings
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_QUALITY, s->status.quality);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_BRIGHTNESS, s->status.brightness);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_CONTRAST, s->status.contrast);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AWB, s->status.awb ? 1 : 0);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AE_LEVEL, (int8_t)s->status.ae_level);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC_VALUE_L, (uint8_t)(s->status.aec_value & 0xFF));
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC_VALUE_H, (uint8_t)((s->status.aec_value >> 8) & 0xFF));
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC, s->status.aec ? 1 : 0);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC2, s->status.aec2 ? 1 : 0);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AGC, s->status.agc ? 1 : 0);
    EEPROM.write(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AGC_GAIN, s->status.agc_gain);
  }

  EEPROM.commit();
  
  // Only re-enable if we were the ones who disabled it
  if (wasEnabled) {
    irEnableAfterCritical();
  }
}

void loadSettingsFromEEPROM() {
    mqttDebugPrintln("Loading settings from EEPROM...");

    // Check EEPROM version
    uint8_t storedVersion = EEPROM.read(EEPROM_ADDR_VERSION);
    if (storedVersion != EEPROM_VERSION) {
        mqttDebugPrintf("EEPROM version mismatch (found %d, expected %d). Initializing EEPROM...\n", 
                        storedVersion, EEPROM_VERSION);
        initializeEEPROM();
        return;
    }

    // Read and validate the Detection Mode setting
    uint8_t detectionModeValue = EEPROM.read(EEPROM_ADDR_DETECTION_MODE);
    if (detectionModeValue == 0 || detectionModeValue == 1) {
        detectionModeEnabled = detectionModeValue == 1;
    } else {
        mqttDebugPrintln("Invalid detection mode in EEPROM. Setting to default (enabled).");
        detectionModeEnabled = true;  // Default value
    }

  // Read and validate the Cooldown Duration
  int cooldownValue = EEPROM.read(EEPROM_ADDR_COOLDOWN);
  if (cooldownValue >= 0 && cooldownValue <= 50) {  // Valid range: 0 to 5.0 seconds (stored as integer with one decimal place)
    cooldownDuration = cooldownValue / 10.0;  // Convert back to float
  } else {
    mqttDebugPrintln("Invalid cooldown value in EEPROM. Setting to default (0.0).");
    cooldownDuration = 0.0;  // Default value
  }

  // Read and validate Cat Location
  uint8_t catLocationValue = EEPROM.read(EEPROM_ADDR_CAT_LOCATION);
  if (catLocationValue == 0 || catLocationValue == 1) {
    catLocation = (catLocationValue == 1);
  } else {
    mqttDebugPrintln("Invalid cat location in EEPROM. Setting to default (home).");
    catLocation = true;  // Default: home
  }

  // Read Inference Mode
  uint8_t infMode = EEPROM.read(EEPROM_ADDR_INFERENCE_MODE);
  if (infMode <= 2) {
    inferenceMode = infMode;
  } else {
    inferenceMode = INFERENCE_MODE_BOTH;
  }

  // Active camera preset (0=day, 1=night)
  uint8_t presetValue = EEPROM.read(EEPROM_ADDR_ACTIVE_PRESET);
  if (presetValue <= 1) {
    activeCameraPreset = presetValue;
  } else {
    activeCameraPreset = 0;
  }

    // Load and validate Camera Settings
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        // JPEG Quality
        uint8_t qualityValue = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_QUALITY);
        if (qualityValue >= 0 && qualityValue <= 63) {
            s->set_quality(s, qualityValue);
        } else {
            mqttDebugPrintln("Invalid JPEG quality in EEPROM. Setting to default (30).");
            s->set_quality(s, 30);
        }

        // Brightness
        int8_t brightnessValue = (int8_t)EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_BRIGHTNESS);
        if (brightnessValue >= -2 && brightnessValue <= 2) {
            s->set_brightness(s, brightnessValue);
        } else {
            mqttDebugPrintln("Invalid brightness in EEPROM. Setting to default (0).");
            s->set_brightness(s, 0);
        }

        // Contrast
        int8_t contrastValue = (int8_t)EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_CONTRAST);
        if (contrastValue >= -2 && contrastValue <= 2) {
            s->set_contrast(s, contrastValue);
        } else {
            mqttDebugPrintln("Invalid contrast in EEPROM. Setting to default (0).");
            s->set_contrast(s, 0);
        }

        // Automatic White Balance (AWB)
        uint8_t awbValue = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AWB);
        if (awbValue <= 1) {
            s->set_whitebal(s, awbValue == 1);
        } else {
            mqttDebugPrintln("Invalid AWB value in EEPROM. Setting to default (enabled).");
            s->set_whitebal(s, true);
        }

    // AE Level
    int8_t aeLevelValue = (int8_t)EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AE_LEVEL);
    if (aeLevelValue >= -2 && aeLevelValue <= 2) {
      s->set_ae_level(s, aeLevelValue);
    } else {
      mqttDebugPrintln("Invalid AE level in EEPROM. Setting to default (0).");
      s->set_ae_level(s, 0);
    }

    // AEC Value (two bytes, little-endian)
    uint8_t aecLo = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC_VALUE_L);
    uint8_t aecHi = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC_VALUE_H);
    uint16_t aecValue = (uint16_t)aecLo | ((uint16_t)aecHi << 8);
    if (aecValue <= 1200) {
      s->set_aec_value(s, aecValue);
    } else {
      mqttDebugPrintln("Invalid AEC value in EEPROM. Setting to default (0).");
      s->set_aec_value(s, 0);
    }

    // Exposure Control (AEC enable)
    uint8_t exposureCtrlValue = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC);
    if (exposureCtrlValue == 0 || exposureCtrlValue == 1) {
      s->set_exposure_ctrl(s, exposureCtrlValue == 1);
    } else {
      mqttDebugPrintln("Invalid exposure control in EEPROM. Setting to default (enabled).");
      s->set_exposure_ctrl(s, 1);
    }

    // AEC2
    uint8_t aec2Value = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AEC2);
    if (aec2Value == 0 || aec2Value == 1) {
      s->set_aec2(s, aec2Value == 1);
    } else {
      mqttDebugPrintln("Invalid AEC2 in EEPROM. Setting to default (disabled).");
      s->set_aec2(s, 0);
    }

    // Gain Control (AGC enable)
    uint8_t gainCtrlValue = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AGC);
    if (gainCtrlValue == 0 || gainCtrlValue == 1) {
      s->set_gain_ctrl(s, gainCtrlValue == 1);
    } else {
      mqttDebugPrintln("Invalid gain control in EEPROM. Setting to default (enabled).");
      s->set_gain_ctrl(s, 1);
    }

    // AGC Gain
    uint8_t agcGainValue = EEPROM.read(EEPROM_BASE_LIVE_SETTINGS + CAM_OFFSET_AGC_GAIN);
    if (agcGainValue <= 30) {
      s->set_agc_gain(s, agcGainValue);
    } else {
      mqttDebugPrintln("Invalid AGC gain in EEPROM. Setting to default (0).");
      s->set_agc_gain(s, 0);
    }

        mqttDebugPrintln("Camera settings loaded from EEPROM.");
    } else {
        mqttDebugPrintln("Failed to get camera sensor. Cannot load camera settings from EEPROM.");
    }

    // Apply the active camera preset (day/night)
    applyPresetFromEEPROM(activeCameraPreset);
    mqttDebugPrintf("Applied camera preset: %s\n", activeCameraPreset == 0 ? "day" : "night");
}

// Helper: save current sensor settings into a preset slot
// TODO: correct the adressing
void savePresetToEEPROM(uint8_t presetIndex) {
  if (presetIndex > 1) return;

  uint16_t base = (presetIndex == 0) ? EEPROM_BASE_PRESET_DAY : EEPROM_BASE_PRESET_NIGHT;

  sensor_t * s = esp_camera_sensor_get();
  if (s == NULL) {
    mqttDebugPrintln("savePresetToEEPROM: sensor NULL");
    return;
  }

  bool wasEnabled = g_irSensingEnabled;
  if (wasEnabled) {
    irDisableForCritical();
  }

  EEPROM.write(base + CAM_OFFSET_QUALITY, s->status.quality);
  EEPROM.write(base + CAM_OFFSET_BRIGHTNESS, s->status.brightness);
  EEPROM.write(base + CAM_OFFSET_CONTRAST, s->status.contrast);
  EEPROM.write(base + CAM_OFFSET_AWB, s->status.awb ? 1 : 0);
  EEPROM.write(base + CAM_OFFSET_AE_LEVEL, (int8_t)s->status.ae_level);
  EEPROM.write(base + CAM_OFFSET_AEC_VALUE_L, (uint8_t)(s->status.aec_value & 0xFF));
  EEPROM.write(base + CAM_OFFSET_AEC_VALUE_H, (uint8_t)((s->status.aec_value >> 8) & 0xFF));
  EEPROM.write(base + CAM_OFFSET_AEC, s->status.aec ? 1 : 0);
  EEPROM.write(base + CAM_OFFSET_AEC2, s->status.aec2 ? 1 : 0);
  EEPROM.write(base + CAM_OFFSET_AGC, s->status.agc ? 1 : 0);
  EEPROM.write(base + CAM_OFFSET_AGC_GAIN, s->status.agc_gain);

  EEPROM.commit();

  if (wasEnabled) {
    irEnableAfterCritical();
  }
}

// Helper: apply a preset from EEPROM to the sensor
void applyPresetFromEEPROM(uint8_t presetIndex) {
  if (presetIndex > 1) return;

  uint16_t base = (presetIndex == 0) ? EEPROM_BASE_PRESET_DAY : EEPROM_BASE_PRESET_NIGHT;

  sensor_t * s = esp_camera_sensor_get();
  if (s == NULL) {
    mqttDebugPrintln("applyPresetFromEEPROM: sensor NULL");
    return;
  }

  // JPEG Quality
  uint8_t qualityValue = EEPROM.read(base + CAM_OFFSET_QUALITY);
  if (qualityValue >= 0 && qualityValue <= 63) {
    s->set_quality(s, qualityValue);
  }

  // Brightness
  int8_t brightnessValue = (int8_t)EEPROM.read(base + CAM_OFFSET_BRIGHTNESS);
  if (brightnessValue >= -2 && brightnessValue <= 2) {
    s->set_brightness(s, brightnessValue);
  }

  // Contrast
  int8_t contrastValue = (int8_t)EEPROM.read(base + CAM_OFFSET_CONTRAST);
  if (contrastValue >= -2 && contrastValue <= 2) {
    s->set_contrast(s, contrastValue);
  }

  // AWB
  uint8_t awbValue = EEPROM.read(base + CAM_OFFSET_AWB);
  if (awbValue == 0 || awbValue == 1) {
    s->set_whitebal(s, awbValue == 1);
  }

  // AE Level
  int8_t aeLevelValue = (int8_t)EEPROM.read(base + CAM_OFFSET_AE_LEVEL);
  if (aeLevelValue >= -2 && aeLevelValue <= 2) {
    s->set_ae_level(s, aeLevelValue);
  }

  // AEC Value
  uint8_t aecLo = EEPROM.read(base + CAM_OFFSET_AEC_VALUE_L);
  uint8_t aecHi = EEPROM.read(base + CAM_OFFSET_AEC_VALUE_H);
  uint16_t aecValue = (uint16_t)aecLo | ((uint16_t)aecHi << 8);
  if (aecValue <= 1200) {
    s->set_aec_value(s, aecValue);
  }

  // Exposure Control
  uint8_t exposureCtrlValue = EEPROM.read(base + CAM_OFFSET_AEC);
  if (exposureCtrlValue == 0 || exposureCtrlValue == 1) {
    s->set_exposure_ctrl(s, exposureCtrlValue == 1);
  }

  // AEC2
  uint8_t aec2Value = EEPROM.read(base + CAM_OFFSET_AEC2);
  if (aec2Value == 0 || aec2Value == 1) {
    s->set_aec2(s, aec2Value == 1);
  }

  // Gain Control
  uint8_t gainCtrlValue = EEPROM.read(base + CAM_OFFSET_AGC);
  if (gainCtrlValue == 0 || gainCtrlValue == 1) {
    s->set_gain_ctrl(s, gainCtrlValue == 1);
  }

  // AGC Gain
  uint8_t agcGainValue = EEPROM.read(base + CAM_OFFSET_AGC_GAIN);
  if (agcGainValue <= 30) {
    s->set_agc_gain(s, agcGainValue);
  }

  // After applying, publish current settings so HA stays in sync
  publishCameraSettings();
}

void handleCameraPresetSelect(String presetStr) {
  uint8_t newPreset;
  if (presetStr.equalsIgnoreCase("day")) {
    newPreset = 0;
  } else if (presetStr.equalsIgnoreCase("night")) {
    newPreset = 1;
  } else {
    mqttDebugPrintln("Invalid camera preset");
    return;
  }

  activeCameraPreset = newPreset;
  
  // Disable IR for EEPROM write and entire preset application
  irDisableForCritical();
  
  EEPROM.write(EEPROM_ADDR_ACTIVE_PRESET, activeCameraPreset);
  EEPROM.commit();

  applyPresetFromEEPROM(activeCameraPreset);

  // Publish preset state
  client.publish(CAMERA_PRESET_TOPIC, activeCameraPreset == 0 ? "day" : "night", true);
  publishCameraSettings();
  mqttDebugPrintf("Camera preset %s applied", activeCameraPreset == 0 ? "day" : "night");

  lastSettingsChangeTime = millis();
  
  irEnableAfterCritical();
}

void handleCameraPresetSave() {
  // Save current sensor settings into the currently active preset
  savePresetToEEPROM(activeCameraPreset);
  mqttDebugPrintln("Camera preset saved to EEPROM");

  lastSettingsChangeTime = millis();
}

void handleInferenceModeCommand(String modeStr) {
  uint8_t newMode;
  if (modeStr.equalsIgnoreCase("local")) {
    newMode = INFERENCE_MODE_LOCAL;
  } else if (modeStr.equalsIgnoreCase("server")) {
    newMode = INFERENCE_MODE_SERVER;
  } else if (modeStr.equalsIgnoreCase("both")) {
    newMode = INFERENCE_MODE_BOTH;
  } else {
    mqttDebugPrintln("Invalid inference mode");
    return;
  }

  inferenceMode = newMode;
  
  // Save to EEPROM
  EEPROM.write(EEPROM_ADDR_INFERENCE_MODE, inferenceMode);
  EEPROM.commit();

  // Publish state
  client.publish(INFERENCE_MODE_TOPIC, modeStr.c_str(), true);
  mqttDebugPrintf("Inference mode set to: %s\n", modeStr.c_str());
}

uint32_t fnv1a_32(const uint8_t* data, size_t len)
{
    const uint32_t FNV_OFFSET_BASIS = 2166136261u;
    const uint32_t FNV_PRIME        = 16777619u;

    uint32_t hash = FNV_OFFSET_BASIS;
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= FNV_PRIME;
    }
    return hash;
}

String makeImageName(const uint8_t* data, size_t len)
{
    uint32_t h = fnv1a_32(data, len);
    char buf[11]; // "img_" + 8 hex + '\0' = 12 actually, be safe:
    snprintf(buf, sizeof(buf), "%08X", h);   // e.g. "3FA12C7E"
    return String(buf);
}