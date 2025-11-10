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
#include <ArduTFLite.h>
#include "model.h"
#include <SD.h>
#include <SPI.h>


// Wi-Fi credentials from secrets.h
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT topics
#define COMMAND_TOPIC "catflap/command"
#define DEBUG_TOPIC "catflap/debug"
#define ALERT_TOPIC "catflap/alert"
#define WIFI_SIGNAL_TOPIC "catflap/wifi_signal"
#define FREE_HEAP_TOPIC "catflap/free_heap"
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
#define SET_DETECTION_MODE_TOPIC "catflap/detection_mode/set"
#define SET_DEBUG_TOGGLE_TOPIC "catflap/debug_toggle/set"
#define SET_COOLDOWN_TOPIC "catflap/cooldown/set"
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
#define SET_SATURATION_TOPIC "catflap/saturation/set"
#define SET_AWB_TOPIC "catflap/awb/set"
#define MODEL_SOURCE_TOPIC "catflap/model_source"
#define SET_MODEL_SOURCE_TOPIC "catflap/model_source/set"

// Device information
#define DEVICE_NAME "catflap"
#define DEVICE_UNIQUE_ID "catflap_esp32"
#define MQTT_DISCOVERY_PREFIX "homeassistant"
#define DEVICE_SW_VERSION "1.0.2" //Increment together with git commits

// EEPROM Addresses
#define EEPROM_SIZE 64
#define EEPROM_VERSION 3  // Bump from 2 to 3
#define EEPROM_ADDRESS_VERSION 0
#define EEPROM_ADDRESS_DETECTION_MODE 1
#define EEPROM_ADDR_COOLDOWN 2
#define EEPROM_ADDRESS_CAMERA_QUALITY 3
#define EEPROM_ADDRESS_CAMERA_BRIGHTNESS 4
#define EEPROM_ADDRESS_CAMERA_CONTRAST 5
#define EEPROM_ADDRESS_CAMERA_SATURATION 6
#define EEPROM_ADDRESS_CAMERA_AWB 7
#define EEPROM_ADDRESS_CAMERA_EXPOSURE_CTRL 8
#define EEPROM_ADDRESS_CAMERA_AEC_VALUE 9  // Uses 2 bytes (9-10)

// GPIO Definitions
#define IR_PIN          32
//#define OPEN_SENSOR_PIN 32
#define ENABLE_FLAP_PIN 33

#define BPP 1  // Grayscale: 1 byte per pixel

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


// Structure to accumulate JPEG data
typedef struct {
  uint8_t *buf;      // Pointer to output buffer
  size_t buf_size;   // Total allocated size
  size_t offset;     // Current offset (number of bytes written)
} jpg_chunk_t;

// Forward declarations of functions
void checkResetReason();
void setup_wifi();
void mqttConnect();
void mqttReconnect();
void mqttSubscribe();
void mqttInitialPublish();
void initCamera();
void captureAndSendImage();
void processCroppedAndConvert(camera_fb_t *cropped);
unsigned int jpgOutputCallback(void *arg, size_t index, const void *data, size_t len);
camera_fb_t* cropImage(camera_fb_t *fb);
camera_fb_t* resizeImage(camera_fb_t *cropped, int originalSize, int targetSize);
camera_fb_t* cropFrame(camera_fb_t *fb);
camera_fb_t* resizeFrame(camera_fb_t *cropped);
bool setupInference();
bool loadModelFromSD(const char* path, uint8_t** model_data, size_t* model_size);
void enableFlap();
void disableFlap();
void handleMqttMessages(char* topic, byte* payload, unsigned int length);
void setupOTA();
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
void handleJpegQualityCommand(String qualityStr);
void publishCameraSettings();
void handleBrightnessCommand(String brightnessStr);
void handleContrastCommand(String contrastStr);
void handleSaturationCommand(String saturationStr);
void handleAWBCommand(String awbStateStr);
void handleExposureCtrlCommand(String stateStr);
void handleAECValueCommand(String aecStr);
void handleCooldownCommand(String cooldownStr);
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
void handleModelSourceCommand(String stateStr);

// Add more handler functions as needed

WiFiClient espClient;
PubSubClient client(espClient);

// Global variables
int lastIRState = HIGH;
bool flapEnabled = true;  // Initialize flap state
bool detectionModeEnabled = true;
bool mqttDebugEnabled = true;
bool catLocation = true; // true = home
bool barrierTriggered = false;  // Flag to indicate barrier has been triggered
bool barrierStableState = false; // Last confirmed stable state (false = not broken, true = broken)
bool useLocalModel = true;  // Default to local model.cc when no SD card

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
unsigned long roundTripTimer = 0;

// Debounce and Cooldown Settings
#define DEBOUNCE_DURATION_MS 30  // Debounce duration in milliseconds
float cooldownDuration = 0.0;    // Cooldown duration in seconds (default to 0)
unsigned long lastTriggerTime = 0;     // Timestamp of the last valid trigger
unsigned long debounceStartTime = 0;   // Timestamp when debounce started
bool debounceActive = false;           // Flag to indicate debounce is in progress

// The Tensor Arena memory area is used by TensorFlow Lite to store input, output and intermediate tensors
// It must be defined as a global array of byte (or u_int8 which is the same type on Arduino) 
// The Tensor Arena size must be defined by trials and errors. We use here a quite large value.
// The alignas(16) directive is used to ensure that the array is aligned on a 16-byte boundary,
// this is important for performance and to prevent some issues on ARM microcontroller architectures.
// Adjust this based on your model's requirements.
constexpr int kTensorArenaSize = 20 * 1024;  // For example, 20KB; adjust as needed.
uint8_t tensor_arena[kTensorArenaSize];

// Global pointers for the model and interpreter.
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;

// Global pointer to the active model (embedded flash or owned heap).
const uint8_t* g_model_data = nullptr;
size_t g_model_size = 0;
// If non-null, this holds a heap/PSRAM-allocated model we own and must free on switch.
static uint8_t* g_owned_heap_model = nullptr;


// -------- Model ownership helpers (safe switching between embedded and SD) --------
static void freeOwnedModel()
{
  if (g_owned_heap_model) {
    heap_caps_free(g_owned_heap_model);
    g_owned_heap_model = nullptr;
  }
}

static void adoptEmbeddedModel()
{
  freeOwnedModel();
  g_model_data = my_model_quant_tflite;
  g_model_size = my_model_quant_tflite_len;
  useLocalModel = true;
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
  useLocalModel = false;
  return true;
}
// -------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for Serial
  
  // Initialize SD card but don't fail if not present
if (!SD.begin()) {
  mqttDebugPrintln("SD card initialization failed - using embedded model");
  useLocalModel = true;
}

// Select model based on flag
if (!useLocalModel) {
  if (!adoptFileModel("/model.tflite")) {
    mqttDebugPrintln("Model loading from SD failed - falling back to embedded model");
    adoptEmbeddedModel();
  } else {
    mqttDebugPrintln("Using SD card model");
  }
} else {
  adoptEmbeddedModel();
  mqttDebugPrintln("Using embedded model");
}

initCamera();

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  if (!isEEPROMInitialized()) {
    initializeEEPROM();
  } else {
    loadSettingsFromEEPROM();
  }

  // Initialize GPIOs
  pinMode(IR_PIN, INPUT);
  //pinMode(OPEN_SENSOR_PIN, INPUT);
  pinMode(ENABLE_FLAP_PIN, OUTPUT);

  setup_wifi();
    // Disable Wi-Fi power save mode
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Initialize MQTT
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(handleMqttMessages);
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
    
  mqttConnect();

  setupOTA();  // Initialize OTA
  checkResetReason();
  publishIPState();

  setupInference();

  mqttDebugPrintln("ESP Setup finished");
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
  yield();
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
  uint8_t version = EEPROM.read(EEPROM_ADDRESS_VERSION);
  return version == EEPROM_VERSION;
}

void initializeEEPROM() {
  mqttDebugPrintln("Initializing EEPROM with default settings...");
  
  // Write the current version to EEPROM
  EEPROM.write(EEPROM_ADDRESS_VERSION, EEPROM_VERSION);
  
  // Initialize settings to default values
  detectionModeEnabled = true;
  cooldownDuration = 0.0;
  
  // Initialize camera settings to defaults
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 30);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, true);
    s->set_special_effect(s, 0);
  }

  // Save default settings to EEPROM
  saveSettingsToEEPROM();
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
      mqttInitialPublish();
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

    // Camera settings
    client.subscribe(SET_JPEG_QUALITY_TOPIC);
    client.subscribe(SET_BRIGHTNESS_TOPIC);
    client.subscribe(SET_CONTRAST_TOPIC);
    client.subscribe(SET_SATURATION_TOPIC);
    client.subscribe(SET_AWB_TOPIC);
    client.subscribe(SET_EXPOSURE_CTRL_TOPIC);
    client.subscribe(SET_AEC_VALUE_TOPIC);

    // Operational settings
    client.subscribe(SET_DETECTION_MODE_TOPIC);
    client.subscribe(SET_DEBUG_TOGGLE_TOPIC);
    client.subscribe(SET_COOLDOWN_TOPIC);
    client.subscribe(SET_FLAP_STATE_TOPIC);
    client.subscribe(SET_CAT_LOCATION_TOPIC);
    client.subscribe(SET_MODEL_SOURCE_TOPIC);
}

void mqttInitialPublish() {
  publishDiscoveryConfigs();
  publishCameraSettings();
  publishCooldownState();
  client.publish(DETECTION_MODE_TOPIC, detectionModeEnabled ? "ON" : "OFF", true);
  client.publish(DEBUG_TOGGLE_TOPIC, mqttDebugEnabled ? "ON" : "OFF", true);
  client.publish(CAT_LOCATION_TOPIC, catLocation ? "ON" : "OFF", true);
}

void initCamera() {
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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.fb_location = CAMERA_FB_IN_PSRAM;  // Use internal DRAM

  // Frame parameters
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 30; // Lower means higher quality
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

// Function to load the model file from the SD card into memory.
bool loadModelFromSD(const char* modelPath, uint8_t** modelBuffer, size_t* modelSize) {
  File modelFile = SD.open(modelPath, FILE_READ);
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
  // Check if a new model file exists, e.g. "model_new.tflite"
  if (SD.exists("/model_new.tflite")) {
    // Optional: Validate the new file (size, checksum, etc.)
    File newModel = SD.open("/model_new.tflite", FILE_READ);
    if (!newModel) {
      Serial.println("Failed to open new model file.");
      return false;
    }
    size_t newSize = newModel.size();
    newModel.close();
    
    // Remove the old model file.
    if (SD.exists("/model.tflite")) {
      SD.remove("/model.tflite");
    }

    // If validation passes, rename the file.
    if (SD.rename("/model_new.tflite", "/model.tflite")) {
      Serial.println("Model updated successfully.");
      return true;
    } else {
      Serial.println("Failed to rename the model file.");
    }
  }
  return false;
}

void reloadModel() {
  // Reload from SD and reinitialize interpreter
  if (!adoptFileModel("/model.tflite")) {
    Serial.println("Failed to load the new model from SD");
    return;
  }
  if (!setupInference()) {
    Serial.println("Failed to reinitialize inference with the new model");
  } else {
    Serial.println("New model loaded and interpreter reinitialized.");
  }
}

// Setup function to initialize the model and interpreter.
bool setupInference() {
  Serial.println("Setting up inference...");
  // Load the model from flash.
  model = tflite::GetModel(g_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("Model schema version (%d) does not match TFLite Micro schema version (%d).\n",
                  model->version(), TFLITE_SCHEMA_VERSION);
    return false;
  }
  
  // Create an op resolver.
  static tflite::AllOpsResolver resolver;
  
  // Create a persistent interpreter.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;
  
  // Allocate tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    return false;
  }
  
  Serial.println("Inference setup complete.");
  return true;
}

// Run inference using the persistent interpreter.
// 'input_data' must match the model's expected input size.
bool run_inference(uint8_t* input_data, size_t input_data_size) {
  // Get the model's input tensor.
  TfLiteTensor* input = interpreter->input(0);
  if (input_data_size != input->bytes) {
    Serial.printf("Input data size (%d) does not match model input size (%d)\n",
                  input_data_size, input->bytes);
    return false;  // or handle the error as needed
  }
  
  // Copy the input data into the input tensor.
  memcpy(input->data.uint8, input_data, input_data_size);
  
  // Run inference.
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Invoke failed");
    return false;
  }
  
  // Get the output tensor.
  TfLiteTensor* output = interpreter->output(0);
  
  // For a two-element output:
  // output->data.uint8[0] is the score for "no prey"
  // output->data.uint8[1] is the score for "prey"
  bool preyDetected = output->data.uint8[1] > output->data.uint8[0];
  
  return preyDetected;
}

// Example callback function for frame2jpg_cb.
// This function will be called repeatedly with chunks of JPEG data.
unsigned int jpgOutputCallback(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunk_t *chunk = (jpg_chunk_t*) arg;
  // Check if there is enough space in the buffer; you may choose to enlarge the buffer here.
  if (chunk->offset + len > chunk->buf_size) {
      // In this example, if the buffer is too small, we simply return 0 to signal an error.
      Serial.println("Output buffer overflow");
      return 0;
  }
  // Copy the data chunk into our output buffer.
  memcpy(chunk->buf + chunk->offset, data, len);
  chunk->offset += len;
  // Return the number of bytes written.
  return len;
}

// Function to process the cropped frame and convert it to JPEG.
void processCroppedAndConvert(camera_fb_t *cropped) {
  int quality = 12;  // Adjust quality as needed.
  
  // Prepare a structure to accumulate JPEG data.
  // Choose an output buffer size large enough for your expected JPEG image.
  size_t output_buffer_size = 20 * 1024; // e.g., 20KB (adjust as needed)
  jpg_chunk_t jpgChunk;
  jpgChunk.buf = (uint8_t*) malloc(output_buffer_size);
  if (!jpgChunk.buf) {
      Serial.println("Failed to allocate JPEG output buffer");
      // Free cropped frame before returning.
      heap_caps_free(cropped->buf);
      heap_caps_free(cropped);
      return;
  }
  jpgChunk.buf_size = output_buffer_size;
  jpgChunk.offset = 0;
  
  // Call frame2jpg_cb on the cropped frame.
  // The callback will be called repeatedly to fill our output buffer.
  esp_err_t res = frame2jpg_cb(cropped, quality, jpgOutputCallback, &jpgChunk);
  if (res != ESP_OK) {
      Serial.printf("frame2jpg_cb failed: %d\n", res);
      free(jpgChunk.buf);
  } else {
      Serial.printf("JPEG conversion succeeded, total %u bytes\n", jpgChunk.offset);
      // At this point, jpgChunk.buf contains the JPEG data, with jpgChunk.offset bytes.
      // You can now publish it via MQTT or store it as needed.
      // For example:
      // client.publish(IMAGE_TOPIC, jpgChunk.buf, jpgChunk.offset, true);
  }
  
  // Free resources.
  free(jpgChunk.buf);
  // Free the cropped frame memory.
  if (cropped->buf) heap_caps_free(cropped->buf);
  heap_caps_free(cropped);
}

void captureAndSendImage() {
  roundTripTimer = millis();
  camera_fb_t * fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    mqttDebugPrintln("Camera capture failed");

    return;
  }
  
  // Crop the image to a centered 384x384 square in PSRAM.
  camera_fb_t *cropped = cropFrame(fb);
  if (!cropped) {
      esp_camera_fb_return(fb);
      return;
  }

  // Resize the cropped image to 96x96.
  camera_fb_t *resized = resizeFrame(cropped);
  if (!resized) {
      esp_camera_fb_return(fb);
      return;
  }

  // For demonstration purposes, let's create a dummy input.
  static uint8_t processed_img[96 * 96] = {0};
  // Fill processed_img with dummy data if needed.
  
  // Run inference on the processed image. 
  // TODO: Handling of closing hatch. If working good, normally locked?
  if(run_inference(processed_img, sizeof(processed_img)))
  {
    handleFlapStateCommand("OFF");
    client.publish(INFERENCE_TOPIC, "Prey detected", true);
  }
  else
  {
    handleFlapStateCommand("ON");
  }  
  client.publish(ROUNDTRIP_TOPIC, String(millis()-roundTripTimer).c_str(), true);

  // Free the resized frame's buffer and struct.
  if (resized->buf) heap_caps_free(resized->buf);
  heap_caps_free(resized);

  processCroppedAndConvert(cropped);

  // Publish the image over MQTT
  if (client.connected()) {
    if (client.publish(IMAGE_TOPIC, cropped->buf, cropped->len, true))
    {
      mqttDebugPrintln("Image sent via mqtt");
    } else {
      mqttDebugPrintln("Image failed to send via mqtt");
    }
  }

  esp_camera_fb_return(fb);
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
  } else if (String(topic) == SET_SATURATION_TOPIC) {
    handleSaturationCommand(incomingMessage);
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
  } else if (String(topic) == INFERENCE_TOPIC) {
    handleInferenceTopic(incomingMessage);
  } else if (String(topic) == SET_CAT_LOCATION_TOPIC) {
    handleCatLocationCommand(incomingMessage);
  } else if (String(topic) == SET_EXPOSURE_CTRL_TOPIC) {
    handleExposureCtrlCommand(incomingMessage);
  } else if (String(topic) == SET_AEC_VALUE_TOPIC) {
    handleAECValueCommand(incomingMessage);
  } else if (String(topic) == SET_MODEL_SOURCE_TOPIC) {
    handleModelSourceCommand(incomingMessage);
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
  flapSwitchConfig["name"] = "Enable Cat Flap";
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
  jpegQualityConfig["min"] = 10;
  jpegQualityConfig["max"] = 63;
  jpegQualityConfig["step"] = 1;
  JsonObject deviceInfoJpegQuality = jpegQualityConfig.createNestedObject("device");
  deviceInfoJpegQuality["identifiers"] = DEVICE_UNIQUE_ID;
  String jpegQualityConfigPayload;
  serializeJson(jpegQualityConfig, jpegQualityConfigPayload);
  client.publish(jpegQualityConfigTopic.c_str(), jpegQualityConfigPayload.c_str(), true);

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

  // Saturation Number
  String saturationConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/saturation/config";
  DynamicJsonDocument saturationConfig(capacity);
  saturationConfig["name"] = "Camera Saturation";
  saturationConfig["command_topic"] = "catflap/saturation/set";
  //saturationConfig["entity_category"] = "configuration";
  saturationConfig["state_topic"] = "catflap/saturation";
  saturationConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_saturation";
  saturationConfig["min"] = -2;
  saturationConfig["max"] = 2;
  saturationConfig["step"] = 1;
  JsonObject deviceInfoSaturation = saturationConfig.createNestedObject("device");
  deviceInfoSaturation["identifiers"] = DEVICE_UNIQUE_ID;
  String saturationConfigPayload;
  serializeJson(saturationConfig, saturationConfigPayload);
  client.publish(saturationConfigTopic.c_str(), saturationConfigPayload.c_str(), true);

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
  snapshotButtonConfig["name"] = "Camera Snapshot";
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

  // Exposure Control Switch
  String exposureControlConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/exposure_ctrl/config";
  DynamicJsonDocument exposureControlConfig(512);
  exposureControlConfig["name"] = "Exposure Control";
  exposureControlConfig["command_topic"] = SET_EXPOSURE_CTRL_TOPIC;
  exposureControlConfig["state_topic"] = EXPOSURE_CTRL_TOPIC;
  exposureControlConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_exposure_ctrl";
  exposureControlConfig["payload_on"] = "ON";
  exposureControlConfig["payload_off"] = "OFF";
  JsonObject deviceInfoExposureCtrl = exposureControlConfig.createNestedObject("device");
  deviceInfoExposureCtrl["identifiers"] = DEVICE_UNIQUE_ID;
  String exposureControlConfigPayload;
  serializeJson(exposureControlConfig, exposureControlConfigPayload);
  client.publish(exposureControlConfigTopic.c_str(), exposureControlConfigPayload.c_str(), true);

  // AEC Value Number
  String aecValueConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/aec_value/config";
  DynamicJsonDocument aecValueConfig(512);
  aecValueConfig["name"] = "AEC Value";
  aecValueConfig["command_topic"] = SET_AEC_VALUE_TOPIC;
  aecValueConfig["state_topic"] = AEC_VALUE_TOPIC;
  aecValueConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_aec_value";
  aecValueConfig["min"] = 0;
  aecValueConfig["max"] = 1200;
  aecValueConfig["step"] = 10;
  JsonObject deviceInfoAecValue = aecValueConfig.createNestedObject("device");
  deviceInfoAecValue["identifiers"] = DEVICE_UNIQUE_ID;
  String aecValueConfigPayload;
  serializeJson(aecValueConfig, aecValueConfigPayload);
  client.publish(aecValueConfigTopic.c_str(), aecValueConfigPayload.c_str(), true);

  // Model Source Switch
  String modelSourceConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/model_source/config";
  DynamicJsonDocument modelSourceConfig(512);
  modelSourceConfig["name"] = "Use SD Card Model";
  modelSourceConfig["command_topic"] = SET_MODEL_SOURCE_TOPIC;
  modelSourceConfig["state_topic"] = MODEL_SOURCE_TOPIC;
  modelSourceConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_model_source";
  modelSourceConfig["payload_on"] = "ON";
  modelSourceConfig["payload_off"] = "OFF";
  JsonObject deviceInfoModelSource = modelSourceConfig.createNestedObject("device");
  deviceInfoModelSource["identifiers"] = DEVICE_UNIQUE_ID;
  String modelSourceConfigPayload;
  serializeJson(modelSourceConfig, modelSourceConfigPayload);
  client.publish(modelSourceConfigTopic.c_str(), modelSourceConfigPayload.c_str(), true);
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
    //handleFlapStateCommand("OFF");
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
  if (quality < 10 || quality > 63) {
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

void handleSaturationCommand(String saturationStr) {
  int saturation = saturationStr.toInt();
  if (saturation < -2 || saturation > 2) {
    mqttDebugPrintln("Invalid saturation value");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_saturation(s, saturation);
    mqttDebugPrintln("Saturation updated");
    client.publish("catflap/saturation", String(saturation).c_str(), true);
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

void handleExposureCtrlCommand(String stateStr) {
  bool enabled = stateStr.equalsIgnoreCase("ON");
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
      s->set_exposure_ctrl(s, enabled);
      mqttDebugPrintln("Exposure control updated");
      client.publish(EXPOSURE_CTRL_TOPIC, enabled ? "ON" : "OFF", true);
      lastSettingsChangeTime = millis();
  } else {
      mqttDebugPrintln("Failed to get camera sensor");
  }
}

void handleAECValueCommand(String aecStr) {
  int aec = aecStr.toInt();
  if (aec < 0 || aec > 1200) {
      mqttDebugPrintln("Invalid AEC value (0-1200)");
      return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
      s->set_aec_value(s, aec);
      mqttDebugPrintln("AEC value updated");
      client.publish(AEC_VALUE_TOPIC, String(aec).c_str(), true);
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

void handleIRBarrierStateChange(bool barrierBroken) {
  unsigned long currentTime = millis();
  if (barrierBroken) {
    if (currentTime - lastTriggerTime >= (cooldownDuration * 1000)) {
      // Update the last IR state
      lastIRState = LOW;  // Barrier is broken

      // Publish IR barrier state
      client.publish(IR_BARRIER_TOPIC, "broken", true);

      // If detection mode is enabled
      if (detectionModeEnabled) {
        captureAndSendImage();
      }
    } else {
      mqttDebugPrintln("Trigger ignored due to cooldown");
    }
  } else {
    // Update on falling edge
    lastTriggerTime = currentTime;

    // Update the last IR state
    lastIRState = HIGH;

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

    // Publish Saturation
    int saturation = s->status.saturation;
    client.publish("catflap/saturation", String(saturation).c_str(), true);

    // Publish AWB
    String awbState = s->status.awb ? "ON" : "OFF";
    client.publish("catflap/awb", awbState.c_str(), true);
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

void updateIRBarrierState() {
  // Read the current state of the IR barrier
  bool irBarrierState = (digitalRead(IR_PIN) == LOW); // true = barrier broken, false = barrier intact
  unsigned long currentTime = millis();

  if (irBarrierState != barrierStableState) {
    // Detected a potential state change
    if (!debounceActive) {
      // Start debouncing
      debounceActive = true;
      debounceStartTime = currentTime;
      // Debugging Output
      Serial.print("Potential state change detected. New state: ");
      Serial.println(irBarrierState ? "Broken" : "Intact");
    } else {
      // Check if debounce duration has passed
      if ((currentTime - debounceStartTime) >= DEBOUNCE_DURATION_MS) {
        // Debounce time met, confirm the state change
        barrierStableState = irBarrierState;
        barrierTriggered = barrierStableState;
        handleIRBarrierStateChange(barrierTriggered);
        debounceActive = false;
        // Debugging Output
        Serial.print("State change confirmed. New state: ");
        Serial.println(barrierTriggered ? "Broken" : "Intact");
      }
      // If debounce duration not met, continue waiting
    }
  } else {
    // No state change detected, reset debounce
    if (debounceActive) {
      debounceActive = false;
      // Debugging Output
      Serial.println("State reverted during debounce. Debounce reset.");
    }
    // No action needed if already stable
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

void saveSettingsToEEPROM() {
    mqttDebugPrintln("Saving settings to EEPROM...");

    // Save the version number
    EEPROM.write(EEPROM_ADDRESS_VERSION, EEPROM_VERSION);
    EEPROM.write(EEPROM_ADDRESS_DETECTION_MODE, detectionModeEnabled ? 1 : 0);
    EEPROM.write(EEPROM_ADDR_COOLDOWN, (int)(cooldownDuration * 10));

    // Save camera settings
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        EEPROM.write(EEPROM_ADDRESS_CAMERA_QUALITY, s->status.quality);
        EEPROM.write(EEPROM_ADDRESS_CAMERA_BRIGHTNESS, s->status.brightness);
        EEPROM.write(EEPROM_ADDRESS_CAMERA_CONTRAST, s->status.contrast);
        EEPROM.write(EEPROM_ADDRESS_CAMERA_SATURATION, s->status.saturation);
        EEPROM.write(EEPROM_ADDRESS_CAMERA_AWB, s->status.awb ? 1 : 0);
        
        // Save exposure control and AEC value
        EEPROM.write(EEPROM_ADDRESS_CAMERA_EXPOSURE_CTRL, s->status.aec ? 1 : 0);
        uint16_t aec_value = s->status.aec_value;
        EEPROM.write(EEPROM_ADDRESS_CAMERA_AEC_VALUE, aec_value & 0xFF);
        EEPROM.write(EEPROM_ADDRESS_CAMERA_AEC_VALUE + 1, (aec_value >> 8) & 0xFF);
    }

    EEPROM.commit();
    mqttDebugPrintln("Settings saved to EEPROM");
}

void loadSettingsFromEEPROM() {
    mqttDebugPrintln("Loading settings from EEPROM...");

    // Check EEPROM version
    uint8_t storedVersion = EEPROM.read(EEPROM_ADDRESS_VERSION);
    if (storedVersion != EEPROM_VERSION) {
        mqttDebugPrintf("EEPROM version mismatch (found %d, expected %d). Initializing EEPROM...\n", 
                        storedVersion, EEPROM_VERSION);
        initializeEEPROM();
        return;
    }

    // Read and validate the Detection Mode setting
    uint8_t detectionModeValue = EEPROM.read(EEPROM_ADDRESS_DETECTION_MODE);
    if (detectionModeValue == 0 || detectionModeValue == 1) {
        detectionModeEnabled = detectionModeValue == 1;
    } else {
        mqttDebugPrintln("Invalid detection mode in EEPROM. Setting to default (enabled).");
        detectionModeEnabled = true;  // Default value
    }

    // Read and validate the Cooldown Duration
    int cooldownValue = EEPROM.read(EEPROM_ADDR_COOLDOWN);
    if (cooldownValue >= 0 && cooldownValue <= 50) {  // Valid range: 0 to 5.0 seconds
        cooldownDuration = cooldownValue / 10.0;  // Convert back to float
    } else {
        mqttDebugPrintln("Invalid cooldown value in EEPROM. Setting to default (0.0).");
        cooldownDuration = 0.0;  // Default value
    }

    // Load and validate Camera Settings
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        // JPEG Quality
        uint8_t qualityValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_QUALITY);
        if (qualityValue >= 10 && qualityValue <= 63) {
            s->set_quality(s, qualityValue);
        } else {
            mqttDebugPrintln("Invalid JPEG quality in EEPROM. Setting to default (30).");
            s->set_quality(s, 30);
        }

        // Brightness
        int8_t brightnessValue = (int8_t)EEPROM.read(EEPROM_ADDRESS_CAMERA_BRIGHTNESS);
        if (brightnessValue >= -2 && brightnessValue <= 2) {
            s->set_brightness(s, brightnessValue);
        } else {
            mqttDebugPrintln("Invalid brightness in EEPROM. Setting to default (0).");
            s->set_brightness(s, 0);
        }

        // Contrast
        int8_t contrastValue = (int8_t)EEPROM.read(EEPROM_ADDRESS_CAMERA_CONTRAST);
        if (contrastValue >= -2 && contrastValue <= 2) {
            s->set_contrast(s, contrastValue);
        } else {
            mqttDebugPrintln("Invalid contrast in EEPROM. Setting to default (0).");
            s->set_contrast(s, 0);
        }

        // Saturation
        int8_t saturationValue = (int8_t)EEPROM.read(EEPROM_ADDRESS_CAMERA_SATURATION);
        if (saturationValue >= -2 && saturationValue <= 2) {
            s->set_saturation(s, saturationValue);
        } else {
            mqttDebugPrintln("Invalid saturation in EEPROM. Setting to default (0).");
            s->set_saturation(s, 0);
        }

        // Automatic White Balance (AWB)
        uint8_t awbValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_AWB);
        if (awbValue <= 1) {
            s->set_whitebal(s, awbValue == 1);
        } else {
            mqttDebugPrintln("Invalid AWB value in EEPROM. Setting to default (enabled).");
            s->set_whitebal(s, true);
        }

        // Exposure Control
        uint8_t exposureCtrlValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_EXPOSURE_CTRL);
        if (exposureCtrlValue <= 1) {
            s->set_exposure_ctrl(s, exposureCtrlValue == 1);
        } else {
            mqttDebugPrintln("Invalid exposure control in EEPROM. Setting to default (enabled).");
            s->set_exposure_ctrl(s, true);
        }

        // AEC Value (16-bit)
        uint16_t aecValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_AEC_VALUE) | 
                           (EEPROM.read(EEPROM_ADDRESS_CAMERA_AEC_VALUE + 1) << 8);
        if (aecValue <= 1200) {
            s->set_aec_value(s, aecValue);
        } else {
            mqttDebugPrintln("Invalid AEC value in EEPROM. Setting to default (400).");
            s->set_aec_value(s, 400);
        }

        mqttDebugPrintln("Camera settings loaded from EEPROM.");
    } else {
        mqttDebugPrintln("Failed to get camera sensor. Cannot load camera settings from EEPROM.");
    }
}

void handleModelSourceCommand(String stateStr) {
    bool wantSD = stateStr.equalsIgnoreCase("ON");

    if (wantSD) {
        if (!SD.begin()) {
            mqttDebugPrintln("Cannot switch to SD card model: No SD card found");
            client.publish(MODEL_SOURCE_TOPIC, "OFF", true);
            return;
        }
        if (!adoptFileModel("/model.tflite")) {
            mqttDebugPrintln("Model loading from SD failed - staying on embedded model");
            adoptEmbeddedModel();
            client.publish(MODEL_SOURCE_TOPIC, "OFF", true);
        } else {
            client.publish(MODEL_SOURCE_TOPIC, "ON", true);
            mqttDebugPrintln("Using SD card model");
        }
    } else {
        adoptEmbeddedModel();
        client.publish(MODEL_SOURCE_TOPIC, "OFF", true);
        mqttDebugPrintln("Using embedded model");
    }

    // Reinitialize the interpreter with the new model
    if (!setupInference()) {
        mqttDebugPrintln("Failed to reinitialize inference after model switch");
    }
}
