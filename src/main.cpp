#define MQTT_MAX_PACKET_SIZE 65535
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_camera.h>
#include <ArduinoOTA.h>
#include <base64.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "secrets.h"

// Wi-Fi credentials from secrets.h
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT topics
#define COMMAND_TOPIC "catflap/command"
#define DEBUG_TOPIC "catflap/debug"
#define ALERT_TOPIC "catflap/alert"
#define IMAGE_TOPIC "catflap/image"
#define IR_BARRIER_TOPIC "catflap/ir_barrier"
#define FLAP_STATE_TOPIC "catflap/flap_state"
#define DETECTION_MODE_TOPIC "catflap/detection_mode"
#define CAT_LOCATION_TOPIC "catflap/cat_location"
#define DEBUG_TOGGLE_TOPIC "catflap/debug_toggle"
#define COOLDOWN_SET_TOPIC "catflap/cooldown/set"
#define COOLDOWN_STATE_TOPIC "catflap/cooldown"
#define DETECTION_MODE_SET_TOPIC "catflap/detection_mode/set"
#define DEBUG_TOGGLE_SET_TOPIC "catflap/debug_toggle/set"



// Device information
#define DEVICE_NAME "catflap"
#define DEVICE_UNIQUE_ID "catflap_esp32"
#define MQTT_DISCOVERY_PREFIX "homeassistant"

// EEPROM Addresses
#define EEPROM_SIZE 64
#define EEPROM_VERSION 2  // Increment this number whenever you change the EEPROM layout
#define EEPROM_ADDRESS_VERSION 0
#define EEPROM_ADDRESS_DETECTION_MODE 1
#define EEPROM_ADDR_COOLDOWN 2  // Assuming previous addresses are 0 and 1
#define EEPROM_ADDRESS_CAMERA_SETTINGS 3  // Starting address for camera settings

// GPIO Definitions
#define IR_PIN          32
//#define OPEN_SENSOR_PIN 32
#define ENABLE_FLAP_PIN 33 //TODO: Double check 13 or 14

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

// Forward declarations of functions
void setup_wifi();
void reconnect();
void initCamera();
void captureAndSendImage();
void openFlap(int duration);
void closeFlap();
void handleMqttMessages(char* topic, byte* payload, unsigned int length);
void setupOTA();
void publishDiscoveryConfigs();
void publishCooldownState();
void mqttDebugPrint(const char* message);
void mqttDebugPrintln(const char* message);
void mqttDebugPrintf(const char* format, ...);
void handleFrameSizeCommand(String frameSizeStr);
void handleJpegQualityCommand(String qualityStr);
void publishCameraSettings();
void handleBrightnessCommand(String brightnessStr);
void handleContrastCommand(String contrastStr);
void handleSaturationCommand(String saturationStr);
void handleAWBCommand(String awbStateStr);
void handleSpecialEffectCommand(String effectStr);
void handleCooldownCommand(String cooldownStr);
void handleIRBarrierStateChange(bool barrierBroken);
void saveSettingsToEEPROM();
void loadSettingsFromEEPROM();
void handleDetectionModeCommand(String stateStr);
void handleDebugToggleCommand(String stateStr);
void handleCatLocationCommand(String locationStr);
void updateIRBarrierState();
bool isEEPROMInitialized();
void initializeEEPROM();
// Add more handler functions as needed

WiFiClient espClient;
PubSubClient client(espClient);

// Global variables
int lastIRState = HIGH;
String flapState = "closed";  // Initialize flap state
bool detectionModeEnabled = true;
bool mqttDebugEnabled = true;
String catLocation = "Home";
bool barrierTriggered = false;  // Flag to indicate barrier has been triggered
bool barrierStableState = false; // Last confirmed stable state (false = not broken, true = broken)

// Timer for saving settings
unsigned long lastSettingsChangeTime = 0;
const unsigned long SETTINGS_SAVE_DELAY = 15000; // 15 seconds

// Debounce and Cooldown Settings
#define DEBOUNCE_DURATION_MS 50  // Debounce duration in milliseconds
float cooldownDuration = 0.0;    // Cooldown duration in seconds (default to 0)
unsigned long lastTriggerTime = 0;     // Timestamp of the last valid trigger
unsigned long debounceStartTime = 0;   // Timestamp when debounce started
bool debounceActive = false;           // Flag to indicate debounce is in progress


void setup() {
  Serial.begin(115200);

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

  // Initialize MQTT
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(handleMqttMessages);
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
    
  reconnect();

  setupOTA();  // Initialize OTA

  mqttDebugPrintln("ESP Setup finished");
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

void loop() {
  // Reconnect to Wi-Fi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

  if (!client.connected()) {
    reconnect();
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

  delay(10);  // Small delay to prevent watchdog resets
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    // Generate a unique client ID using the MAC address
    String clientId = "ESP32Client-";
    clientId += String(WiFi.macAddress());

    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe to command topics
      client.subscribe(COMMAND_TOPIC);
      client.subscribe("catflap/frame_size/set");
      client.subscribe("catflap/jpeg_quality/set");
      client.subscribe("catflap/brightness/set");
      client.subscribe("catflap/contrast/set");
      client.subscribe("catflap/saturation/set");
      client.subscribe("catflap/awb/set");
      client.subscribe("catflap/special_effect/set");
      client.subscribe(DETECTION_MODE_SET_TOPIC);
      client.subscribe(DEBUG_TOGGLE_SET_TOPIC);
      client.subscribe(COOLDOWN_SET_TOPIC);

      // Add subscriptions for other settings
      //mqttDebugPrintf("MQTT buffer size set to: %d bytes\n", client.getBufferSize());

      // Publish discovery configs
      publishDiscoveryConfigs();

      // Publish current camera settings
      publishCameraSettings();

      publishCooldownState();


      // Publish detection mode state
      client.publish(DETECTION_MODE_TOPIC, detectionModeEnabled ? "ON" : "OFF", true);

      // Publish debug toggle state
      client.publish(DEBUG_TOGGLE_TOPIC, mqttDebugEnabled ? "ON" : "OFF", true);

      // Publish cat location
      client.publish(CAT_LOCATION_TOPIC, catLocation.c_str(), true);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
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
  config.pixel_format = PIXFORMAT_JPEG;
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

void captureAndSendImage() {
  camera_fb_t * fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    mqttDebugPrintln("Camera capture failed");

    return;
  }

  // Base64 encode the image
  String encodedImage = base64::encode(fb->buf, fb->len);
  mqttDebugPrintf("Encoded image size: %u bytes\n", encodedImage.length());

  // Publish the image over MQTT
  if (client.connected()) {
    if (client.publish(IMAGE_TOPIC, encodedImage.c_str(), true))
    {
      mqttDebugPrintln("Image sent via mqtt");
    } else {
      mqttDebugPrintln("Image failed to send via mqtt");
    }
  }

  esp_camera_fb_return(fb);
}

void enableFlap() {
  // Open the flap
  digitalWrite(ENABLE_FLAP_PIN, LOW);

  flapState = "Open";
  client.publish(FLAP_STATE_TOPIC, flapState.c_str(), true);
}

void disableFlap() {
  // Close the flap
  digitalWrite(ENABLE_FLAP_PIN, HIGH);

  flapState = "Closed";
  client.publish(FLAP_STATE_TOPIC, flapState.c_str(), true);
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
    if (incomingMessage.equalsIgnoreCase("enable_flap")) {
      enableFlap();
    } else if (incomingMessage.equalsIgnoreCase("disable_flap")) {
      disableFlap();
    } else if (incomingMessage.equalsIgnoreCase("snapshot")) {
      captureAndSendImage();
    } else if (incomingMessage.equalsIgnoreCase("restart")) {
      ESP.restart();
    } else if (incomingMessage.equalsIgnoreCase("set_home") || incomingMessage.equalsIgnoreCase("set_away")) {
      handleCatLocationCommand(incomingMessage);
    }
  } else if (String(topic) == "catflap/frame_size/set") {
    handleFrameSizeCommand(incomingMessage);
  } else if (String(topic) == "catflap/jpeg_quality/set") {
    handleJpegQualityCommand(incomingMessage);
  } else if (String(topic) == "catflap/brightness/set") {
    handleBrightnessCommand(incomingMessage);
  } else if (String(topic) == "catflap/contrast/set") {
    handleContrastCommand(incomingMessage);
  } else if (String(topic) == "catflap/saturation/set") {
    handleSaturationCommand(incomingMessage);
  } else if (String(topic) == "catflap/awb/set") {
    handleAWBCommand(incomingMessage);
  } else if (String(topic) == "catflap/special_effect/set") {
    handleSpecialEffectCommand(incomingMessage);
  } else if (String(topic) == DETECTION_MODE_SET_TOPIC) {
  handleDetectionModeCommand(incomingMessage);
  } else if (String(topic) == DEBUG_TOGGLE_SET_TOPIC) {
    handleDebugToggleCommand(incomingMessage);
  }
  else if (String(topic) == COOLDOWN_SET_TOPIC) {
    handleCooldownCommand(incomingMessage);
    lastSettingsChangeTime = millis();  // Update settings change timer
  }

  // Add handling for other settings
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
  irSensorConfig["name"] = "Cat Flap IR Barrier";
  irSensorConfig["state_topic"] = IR_BARRIER_TOPIC;
  irSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_ir_barrier";
  irSensorConfig["device_class"] = "motion";
  irSensorConfig["payload_on"] = "closed";
  irSensorConfig["payload_off"] = "open";
  JsonObject deviceInfo1 = irSensorConfig.createNestedObject("device");
  deviceInfo1["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfo1["name"] = "Cat Flap";
  deviceInfo1["model"] = "AI Cat Flap";
  deviceInfo1["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String irSensorConfigPayload;
  serializeJson(irSensorConfig, irSensorConfigPayload);
  //mqttDebugPrintf("Payload size for %s: %d bytes\n", irSensorConfigTopic.c_str(), irSensorConfigPayload.length());
  //mqttDebugPrintf("Free heap before publishing to %s: %d bytes\n", irSensorConfigTopic.c_str(), ESP.getFreeHeap());

  if (client.publish(irSensorConfigTopic.c_str(), irSensorConfigPayload.c_str(), true))
  {
    //mqttDebugPrintln("irSensorConfigTopic - Sent");
  } else {
    mqttDebugPrintln("irSensorConfigTopic - Failed");
  }

  // Flap State Sensor
  String flapStateConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/flap_state/config";
  DynamicJsonDocument flapStateConfig(capacity);
  flapStateConfig["name"] = "Cat Flap State";
  flapStateConfig["state_topic"] = FLAP_STATE_TOPIC;
  flapStateConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_flap_state";
  JsonObject deviceInfo2 = flapStateConfig.createNestedObject("device");
  deviceInfo2["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfo2["name"] = "Cat Flap";
  deviceInfo2["model"] = "AI Cat Flap";
  deviceInfo2["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String flapStateConfigPayload;
  serializeJson(flapStateConfig, flapStateConfigPayload);
  //mqttDebugPrintf("Payload size for %s: %d bytes\n", flapStateConfigTopic.c_str(), flapStateConfigPayload.length());
  if (client.publish(flapStateConfigTopic.c_str(), flapStateConfigPayload.c_str(), true))
  {
    //mqttDebugPrintln("flapStateConfigTopic - Sent");
  } else {
    mqttDebugPrintln("flapStateConfigTopic - Failed");
  }

  // Flap Control Switch
  String flapSwitchConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/control/config";
  DynamicJsonDocument flapSwitchConfig(capacity);
  flapSwitchConfig["name"] = "Cat Flap Control";
  flapSwitchConfig["command_topic"] = COMMAND_TOPIC;
  flapSwitchConfig["payload_on"] = "enable_flap";
  flapSwitchConfig["payload_off"] = "disable_flap";
  flapSwitchConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_control";
  JsonObject deviceInfo3 = flapSwitchConfig.createNestedObject("device");
  deviceInfo3["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfo3["name"] = "Cat Flap";
  deviceInfo3["model"] = "AI Cat Flap";
  deviceInfo3["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String flapSwitchConfigPayload;
  serializeJson(flapSwitchConfig, flapSwitchConfigPayload);
  //mqttDebugPrintf("Payload size for %s: %d bytes\n", flapSwitchConfigTopic.c_str(), flapSwitchConfigPayload.length());
  if (client.publish(flapSwitchConfigTopic.c_str(), flapSwitchConfigPayload.c_str(), true))
  {
    //mqttDebugPrintln("flapSwitchConfigTopic - Sent");
  } else {
    mqttDebugPrintln("flapSwitchConfigTopic - Failed");
  }

  // Detection Mode Switch
  String detectionModeConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/detection_mode/config";
  DynamicJsonDocument detectionModeConfig(capacity);
  detectionModeConfig["name"] = "Detection Mode";
  detectionModeConfig["command_topic"] = DETECTION_MODE_SET_TOPIC;
  detectionModeConfig["state_topic"] = DETECTION_MODE_TOPIC;
  detectionModeConfig["payload_on"] = "ON";
  detectionModeConfig["payload_off"] = "OFF";
  detectionModeConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_detection_mode";
  JsonObject deviceInfoDetection = detectionModeConfig.createNestedObject("device");
  deviceInfoDetection["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoDetection["name"] = "Cat Flap";
  deviceInfoDetection["model"] = "AI Cat Flap";
  deviceInfoDetection["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String detectionModeConfigPayload;
  serializeJson(detectionModeConfig, detectionModeConfigPayload);
  client.publish(detectionModeConfigTopic.c_str(), detectionModeConfigPayload.c_str(), true);

  // Cat Location Sensor
  String catLocationConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/cat_location/config";
  DynamicJsonDocument catLocationConfig(capacity);
  catLocationConfig["name"] = "Cat Location";
  catLocationConfig["state_topic"] = CAT_LOCATION_TOPIC;
  catLocationConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_cat_location";
  JsonObject deviceInfoCatLocation = catLocationConfig.createNestedObject("device");
  deviceInfoCatLocation["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoCatLocation["name"] = "Cat Flap";
  deviceInfoCatLocation["model"] = "AI Cat Flap";
  deviceInfoCatLocation["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String catLocationConfigPayload;
  serializeJson(catLocationConfig, catLocationConfigPayload);
  client.publish(catLocationConfigTopic.c_str(), catLocationConfigPayload.c_str(), true);

  // Set Home Button
  String setHomeButtonConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/set_home/config";
  DynamicJsonDocument setHomeButtonConfig(capacity);
  setHomeButtonConfig["name"] = "Set Cat Location Home";
  setHomeButtonConfig["command_topic"] = COMMAND_TOPIC;
  setHomeButtonConfig["payload_press"] = "set_home";
  setHomeButtonConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_set_home";
  JsonObject deviceInfoSetHome = setHomeButtonConfig.createNestedObject("device");
  deviceInfoSetHome["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoSetHome["name"] = "Cat Flap";
  deviceInfoSetHome["model"] = "AI Cat Flap";
  deviceInfoSetHome["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String setHomeButtonConfigPayload;
  serializeJson(setHomeButtonConfig, setHomeButtonConfigPayload);
  client.publish(setHomeButtonConfigTopic.c_str(), setHomeButtonConfigPayload.c_str(), true);

  // Set Away Button
  String setAwayButtonConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/set_away/config";
  DynamicJsonDocument setAwayButtonConfig(capacity);
  setAwayButtonConfig["name"] = "Set Cat Location Away";
  setAwayButtonConfig["command_topic"] = COMMAND_TOPIC;
  setAwayButtonConfig["payload_press"] = "set_away";
  setAwayButtonConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_set_away";
  JsonObject deviceInfoSetAway = setAwayButtonConfig.createNestedObject("device");
  deviceInfoSetAway["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoSetAway["name"] = "Cat Flap";
  deviceInfoSetAway["model"] = "AI Cat Flap";
  deviceInfoSetAway["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String setAwayButtonConfigPayload;
  serializeJson(setAwayButtonConfig, setAwayButtonConfigPayload);
  client.publish(setAwayButtonConfigTopic.c_str(), setAwayButtonConfigPayload.c_str(), true);

  // Debug Toggle Switch
  String debugToggleConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/debug_toggle/config";
  DynamicJsonDocument debugToggleConfig(capacity);
  debugToggleConfig["name"] = "Debug Output";
  debugToggleConfig["command_topic"] = DEBUG_TOGGLE_SET_TOPIC;
  debugToggleConfig["state_topic"] = DEBUG_TOGGLE_TOPIC;
  debugToggleConfig["payload_on"] = "ON";
  debugToggleConfig["payload_off"] = "OFF";
  debugToggleConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_debug_toggle";
  JsonObject deviceInfoDebugToggle = debugToggleConfig.createNestedObject("device");
  deviceInfoDebugToggle["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoDebugToggle["name"] = "Cat Flap";
  deviceInfoDebugToggle["model"] = "AI Cat Flap";
  deviceInfoDebugToggle["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String debugToggleConfigPayload;
  serializeJson(debugToggleConfig, debugToggleConfigPayload);
  client.publish(debugToggleConfigTopic.c_str(), debugToggleConfigPayload.c_str(), true);

   // Camera Entity
  String cameraConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/camera/" + DEVICE_NAME + "/camera/config";
  DynamicJsonDocument cameraConfig(capacity);
  cameraConfig["name"] = "Cat Flap Camera";
  cameraConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_camera";
  cameraConfig["topic"] = IMAGE_TOPIC;
  cameraConfig["image_encoding"] = "b64";  // Specify the image encoding as Base64
  JsonObject deviceInfo4 = cameraConfig.createNestedObject("device");
  deviceInfo4["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfo4["name"] = "Cat Flap";
  deviceInfo4["model"] = "AI Cat Flap";
  deviceInfo4["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String cameraConfigPayload;
  serializeJson(cameraConfig, cameraConfigPayload);
  //mqttDebugPrintf("Payload size for %s: %d bytes\n", cameraConfigTopic.c_str(), cameraConfigPayload.length());
  if (client.publish(cameraConfigTopic.c_str(), cameraConfigPayload.c_str(), true))
  {
    //mqttDebugPrintln("cameraConfigTopic - Sent");
  } else {
    mqttDebugPrintln("cameraConfigTopic - Failed");
  }

  // Frame Size Select
  String frameSizeConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/select/" + DEVICE_NAME + "/frame_size/config";
  DynamicJsonDocument frameSizeConfig(capacity);
  frameSizeConfig["name"] = "Cat Flap Camera Frame Size";
  frameSizeConfig["command_topic"] = "catflap/frame_size/set";
  frameSizeConfig["state_topic"] = "catflap/frame_size";
  frameSizeConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_frame_size";
  JsonArray frameSizeOptions = frameSizeConfig.createNestedArray("options");
  frameSizeOptions.add("QQVGA");
  frameSizeOptions.add("QVGA");
  frameSizeOptions.add("VGA");
  frameSizeOptions.add("SVGA");
  // Add other frame sizes as needed
  JsonObject deviceInfoFrameSize = frameSizeConfig.createNestedObject("device");
  deviceInfoFrameSize["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoFrameSize["name"] = "Cat Flap";
  deviceInfoFrameSize["model"] = "AI Cat Flap";
  deviceInfoFrameSize["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String frameSizeConfigPayload;
  serializeJson(frameSizeConfig, frameSizeConfigPayload);
  client.publish(frameSizeConfigTopic.c_str(), frameSizeConfigPayload.c_str(), true);

  // JPEG Quality Number
  String jpegQualityConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/jpeg_quality/config";
  DynamicJsonDocument jpegQualityConfig(capacity);
  jpegQualityConfig["name"] = "Cat Flap Camera JPEG Quality";
  jpegQualityConfig["command_topic"] = "catflap/jpeg_quality/set";
  jpegQualityConfig["state_topic"] = "catflap/jpeg_quality";
  jpegQualityConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_jpeg_quality";
  jpegQualityConfig["min"] = 10;
  jpegQualityConfig["max"] = 63;
  jpegQualityConfig["step"] = 1;
  JsonObject deviceInfoJpegQuality = jpegQualityConfig.createNestedObject("device");
  deviceInfoJpegQuality["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoJpegQuality["name"] = "Cat Flap";
  deviceInfoJpegQuality["model"] = "AI Cat Flap";
  deviceInfoJpegQuality["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String jpegQualityConfigPayload;
  serializeJson(jpegQualityConfig, jpegQualityConfigPayload);
  client.publish(jpegQualityConfigTopic.c_str(), jpegQualityConfigPayload.c_str(), true);

  // Brightness Number
  String brightnessConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/brightness/config";
  DynamicJsonDocument brightnessConfig(capacity);
  brightnessConfig["name"] = "Cat Flap Camera Brightness";
  brightnessConfig["command_topic"] = "catflap/brightness/set";
  brightnessConfig["state_topic"] = "catflap/brightness";
  brightnessConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_brightness";
  brightnessConfig["min"] = -2;
  brightnessConfig["max"] = 2;
  brightnessConfig["step"] = 1;
  JsonObject deviceInfoBrightness = brightnessConfig.createNestedObject("device");
  deviceInfoBrightness["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoBrightness["name"] = "Cat Flap";
  deviceInfoBrightness["model"] = "AI Cat Flap";
  deviceInfoBrightness["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String brightnessConfigPayload;
  serializeJson(brightnessConfig, brightnessConfigPayload);
  client.publish(brightnessConfigTopic.c_str(), brightnessConfigPayload.c_str(), true);

  // Contrast Number
  String contrastConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/contrast/config";
  DynamicJsonDocument contrastConfig(capacity);
  contrastConfig["name"] = "Cat Flap Camera Contrast";
  contrastConfig["command_topic"] = "catflap/contrast/set";
  contrastConfig["state_topic"] = "catflap/contrast";
  contrastConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_contrast";
  contrastConfig["min"] = -2;
  contrastConfig["max"] = 2;
  contrastConfig["step"] = 1;
  JsonObject deviceInfoContrast = contrastConfig.createNestedObject("device");
  deviceInfoContrast["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoContrast["name"] = "Cat Flap";
  deviceInfoContrast["model"] = "AI Cat Flap";
  deviceInfoContrast["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String contrastConfigPayload;
  serializeJson(contrastConfig, contrastConfigPayload);
  client.publish(contrastConfigTopic.c_str(), contrastConfigPayload.c_str(), true);

  // Saturation Number
  String saturationConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/saturation/config";
  DynamicJsonDocument saturationConfig(capacity);
  saturationConfig["name"] = "Cat Flap Camera Saturation";
  saturationConfig["command_topic"] = "catflap/saturation/set";
  saturationConfig["state_topic"] = "catflap/saturation";
  saturationConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_saturation";
  saturationConfig["min"] = -2;
  saturationConfig["max"] = 2;
  saturationConfig["step"] = 1;
  JsonObject deviceInfoSaturation = saturationConfig.createNestedObject("device");
  deviceInfoSaturation["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoSaturation["name"] = "Cat Flap";
  deviceInfoSaturation["model"] = "AI Cat Flap";
  deviceInfoSaturation["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String saturationConfigPayload;
  serializeJson(saturationConfig, saturationConfigPayload);
  client.publish(saturationConfigTopic.c_str(), saturationConfigPayload.c_str(), true);

  // AWB Switch
  String awbConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + DEVICE_NAME + "/awb/config";
  DynamicJsonDocument awbConfig(capacity);
  awbConfig["name"] = "Cat Flap Camera AWB";
  awbConfig["command_topic"] = "catflap/awb/set";
  awbConfig["state_topic"] = "catflap/awb";
  awbConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_awb";
  awbConfig["payload_on"] = "ON";
  awbConfig["payload_off"] = "OFF";
  JsonObject deviceInfoAWB = awbConfig.createNestedObject("device");
  deviceInfoAWB["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoAWB["name"] = "Cat Flap";
  deviceInfoAWB["model"] = "AI Cat Flap";
  deviceInfoAWB["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String awbConfigPayload;
  serializeJson(awbConfig, awbConfigPayload);
  client.publish(awbConfigTopic.c_str(), awbConfigPayload.c_str(), true);

  // Special Effect Select
  String specialEffectConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/select/" + DEVICE_NAME + "/special_effect/config";
  DynamicJsonDocument specialEffectConfig(capacity);
  specialEffectConfig["name"] = "Cat Flap Camera Special Effect";
  specialEffectConfig["command_topic"] = "catflap/special_effect/set";
  specialEffectConfig["state_topic"] = "catflap/special_effect";
  specialEffectConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_special_effect";
  JsonArray effectOptions = specialEffectConfig.createNestedArray("options");
  effectOptions.add("No Effect");
  effectOptions.add("Negative");
  effectOptions.add("Grayscale");
  effectOptions.add("Red Tint");
  effectOptions.add("Green Tint");
  effectOptions.add("Blue Tint");
  effectOptions.add("Sepia");
  // Add other effects as needed
  JsonObject deviceInfoEffect = specialEffectConfig.createNestedObject("device");
  deviceInfoEffect["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoEffect["name"] = "Cat Flap";
  deviceInfoEffect["model"] = "AI Cat Flap";
  deviceInfoEffect["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String specialEffectConfigPayload;
  serializeJson(specialEffectConfig, specialEffectConfigPayload);
  client.publish(specialEffectConfigTopic.c_str(), specialEffectConfigPayload.c_str(), true);

  // Snapshot Button
  String snapshotButtonConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/snapshot/config";
  DynamicJsonDocument snapshotButtonConfig(capacity);
  snapshotButtonConfig["name"] = "Cat Flap Camera Snapshot";
  snapshotButtonConfig["command_topic"] = COMMAND_TOPIC;
  snapshotButtonConfig["payload_press"] = "snapshot";
  snapshotButtonConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_snapshot";
  JsonObject deviceInfoSnapshot = snapshotButtonConfig.createNestedObject("device");
  deviceInfoSnapshot["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoSnapshot["name"] = "Cat Flap";
  deviceInfoSnapshot["model"] = "AI Cat Flap";
  deviceInfoSnapshot["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String snapshotButtonConfigPayload;
  serializeJson(snapshotButtonConfig, snapshotButtonConfigPayload);
  client.publish(snapshotButtonConfigTopic.c_str(), snapshotButtonConfigPayload.c_str(), true);

  // Restart Button
  String restartButtonConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/button/" + DEVICE_NAME + "/restart/config";
  DynamicJsonDocument restartButtonConfig(capacity);
  restartButtonConfig["name"] = "Cat Flap Camera Restart";
  restartButtonConfig["command_topic"] = COMMAND_TOPIC;
  restartButtonConfig["payload_press"] = "restart";
  restartButtonConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_restart";
  JsonObject deviceInfoRestart = restartButtonConfig.createNestedObject("device");
  deviceInfoRestart["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoRestart["name"] = "Cat Flap";
  deviceInfoRestart["model"] = "AI Cat Flap";
  deviceInfoRestart["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String restartButtonConfigPayload;
  serializeJson(restartButtonConfig, restartButtonConfigPayload);
  client.publish(restartButtonConfigTopic.c_str(), restartButtonConfigPayload.c_str(), true);

  // Debug Sensor
  String debugSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/debug/config";
  DynamicJsonDocument debugSensorConfig(512);
  debugSensorConfig["name"] = "Cat Flap Debug";
  debugSensorConfig["state_topic"] = DEBUG_TOPIC;
  debugSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_debug";
  debugSensorConfig["icon"] = "mdi:message-text";
  // Optionally set the device class if appropriate
  // debugSensorConfig["device_class"] = "timestamp";  // Use if messages are timestamps
  JsonObject deviceInfoDebug = debugSensorConfig.createNestedObject("device");
  deviceInfoDebug["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoDebug["name"] = "Cat Flap";
  deviceInfoDebug["model"] = "AI Cat Flap";
  deviceInfoDebug["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String debugSensorConfigPayload;
  serializeJson(debugSensorConfig, debugSensorConfigPayload);
  client.publish(debugSensorConfigTopic.c_str(), debugSensorConfigPayload.c_str(), true);

  // Alert Sensor
  String alertSensorConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + DEVICE_NAME + "/alert/config";
  DynamicJsonDocument alertSensorConfig(512);
  alertSensorConfig["name"] = "Cat Flap Alert";
  alertSensorConfig["state_topic"] = ALERT_TOPIC;
  alertSensorConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_alert";
  alertSensorConfig["icon"] = "mdi:message-text";
  JsonObject deviceInfoAlert = alertSensorConfig.createNestedObject("device");
  deviceInfoAlert["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoAlert["name"] = "Cat Flap";
  deviceInfoAlert["model"] = "AI Cat Flap";
  deviceInfoAlert["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String alertSensorConfigPayload;
  serializeJson(alertSensorConfig, alertSensorConfigPayload);
  client.publish(alertSensorConfigTopic.c_str(), alertSensorConfigPayload.c_str(), true);

  // Cooldown Number Entity
  String cooldownConfigTopic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + DEVICE_NAME + "/cooldown/config";
  DynamicJsonDocument cooldownConfig(512);
  cooldownConfig["name"] = "IR Barrier Cooldown";
  cooldownConfig["command_topic"] = COOLDOWN_SET_TOPIC;
  cooldownConfig["state_topic"] = COOLDOWN_STATE_TOPIC;
  cooldownConfig["unique_id"] = String(DEVICE_UNIQUE_ID) + "_cooldown";
  cooldownConfig["min"] = 0.0;
  cooldownConfig["max"] = 5.0;
  cooldownConfig["step"] = 0.1;
  cooldownConfig["unit_of_measurement"] = "s";
  JsonObject deviceInfoCooldown = cooldownConfig.createNestedObject("device");
  deviceInfoCooldown["identifiers"] = DEVICE_UNIQUE_ID;
  deviceInfoCooldown["name"] = "Cat Flap";
  deviceInfoCooldown["model"] = "AI Cat Flap";
  deviceInfoCooldown["manufacturer"] = "RB Advanced Intelligence Labs Inc.";
  String cooldownConfigPayload;
  serializeJson(cooldownConfig, cooldownConfigPayload);
  client.publish(cooldownConfigTopic.c_str(), cooldownConfigPayload.c_str(), true);

}

void handleDetectionModeCommand(String stateStr) {
  bool newState = stateStr.equalsIgnoreCase("ON");
  detectionModeEnabled = newState;
  client.publish(DETECTION_MODE_TOPIC, detectionModeEnabled ? "ON" : "OFF", true);
  mqttDebugPrintln("Detection mode updated");
  lastSettingsChangeTime = millis();
}

void handleDebugToggleCommand(String stateStr) {
  mqttDebugEnabled = stateStr.equalsIgnoreCase("ON");
  client.publish(DEBUG_TOGGLE_TOPIC, mqttDebugEnabled ? "ON" : "OFF", true);
  mqttDebugPrintln("Debug output toggled");
}

void handleCatLocationCommand(String locationStr) {
  if (locationStr.equalsIgnoreCase("set_home")) {
    catLocation = "Home";
  } else if (locationStr.equalsIgnoreCase("set_away")) {
    catLocation = "Away";
  }
  client.publish(CAT_LOCATION_TOPIC, catLocation.c_str(), true);
  mqttDebugPrintln("Cat location updated");
}

void handleFrameSizeCommand(String frameSizeStr) {
  framesize_t newFrameSize;

  if (frameSizeStr.equalsIgnoreCase("QQVGA")) {
    newFrameSize = FRAMESIZE_QQVGA;
  } else if (frameSizeStr.equalsIgnoreCase("QVGA")) {
    newFrameSize = FRAMESIZE_QVGA;
  } else if (frameSizeStr.equalsIgnoreCase("VGA")) {
    newFrameSize = FRAMESIZE_VGA;
  } else if (frameSizeStr.equalsIgnoreCase("SVGA")) {
    newFrameSize = FRAMESIZE_SVGA;  
  } else {
    mqttDebugPrintln("Invalid frame size command");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_framesize(s, newFrameSize);
    mqttDebugPrintln("Frame size updated");
    client.publish("catflap/frame_size", frameSizeStr.c_str(), true);
    lastSettingsChangeTime = millis();
  } else {
    mqttDebugPrintln("Failed to get camera sensor");
  }
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

void handleSpecialEffectCommand(String effectStr) {
  int effect;
  if (effectStr.equalsIgnoreCase("No Effect")) {
    effect = 0;
  } else if (effectStr.equalsIgnoreCase("Negative")) {
    effect = 1;
  } else if (effectStr.equalsIgnoreCase("Grayscale")) {
    effect = 2;
  } else if (effectStr.equalsIgnoreCase("Red Tint")) {
    effect = 3;
  } else if (effectStr.equalsIgnoreCase("Green Tint")) {
    effect = 4;
  } else if (effectStr.equalsIgnoreCase("Blue Tint")) {
    effect = 5;
  } else if (effectStr.equalsIgnoreCase("Sepia")) {
    effect = 6;
  } else {
    mqttDebugPrintln("Invalid special effect");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_special_effect(s, effect);
    mqttDebugPrintln("Special effect updated");
    client.publish("catflap/special_effect", effectStr.c_str(), true);
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
  
  cooldownDuration = newCooldown;
  mqttDebugPrintf("Cooldown duration updated to %.1f seconds\n", cooldownDuration);
  publishCooldownState();
}

void handleIRBarrierStateChange(bool barrierBroken) {
  if (barrierBroken) {
    unsigned long currentTime = millis();
    if (currentTime - lastTriggerTime >= (cooldownDuration * 1000)) {
      lastTriggerTime = currentTime;
      mqttDebugPrintln("IR Barrier Triggered");

      // Update the last IR state
      lastIRState = LOW;  // Barrier is broken

      // Publish IR barrier state
      client.publish(IR_BARRIER_TOPIC, "closed", true);

      // If detection mode is enabled
      if (detectionModeEnabled) {
        captureAndSendImage();
      }
    } else {
      mqttDebugPrintln("Trigger ignored due to cooldown");
    }
  } else {
    // Barrier is not broken
    mqttDebugPrintln("IR Barrier is open");

    // Update the last IR state
    lastIRState = HIGH;

    // Publish IR barrier state
    client.publish(IR_BARRIER_TOPIC, "open", true);
  }
}

void publishCameraSettings() {
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // Publish Frame Size
    String frameSizeStr;
    switch (s->status.framesize) {
      case FRAMESIZE_QQVGA: frameSizeStr = "QQVGA"; break;
      case FRAMESIZE_QVGA:  frameSizeStr = "QVGA";  break;
      case FRAMESIZE_VGA:   frameSizeStr = "VGA";   break;
      case FRAMESIZE_SVGA:   frameSizeStr = "SVGA";   break;
      // Add other cases as needed
      default: frameSizeStr = "UNKNOWN"; break;
    }
    client.publish("catflap/frame_size", frameSizeStr.c_str(), true);

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

    // Publish Special Effect
    String effectStr;
    switch (s->status.special_effect) {
      case 0: effectStr = "No Effect"; break;
      case 1: effectStr = "Negative"; break;
      case 2: effectStr = "Grayscale"; break;
      case 3: effectStr = "Red Tint"; break;
      case 4: effectStr = "Green Tint"; break;
      case 5: effectStr = "Blue Tint"; break;
      case 6: effectStr = "Sepia"; break;
      default: effectStr = "Unknown"; break;
    }
    client.publish("catflap/special_effect", effectStr.c_str(), true);

    // Publish other settings as needed
  }
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

  // Save Cooldown Duration
  EEPROM.write(EEPROM_ADDR_COOLDOWN, (int)(cooldownDuration * 10));

  // Save camera settings
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 0, s->status.framesize);
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 1, s->status.quality);
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 2, s->status.brightness);
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 3, s->status.contrast);
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 4, s->status.saturation);
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 5, s->status.awb ? 1 : 0);
    EEPROM.write(EEPROM_ADDRESS_CAMERA_SETTINGS + 6, s->status.special_effect);
  }

  EEPROM.commit();
}


void loadSettingsFromEEPROM() {
  mqttDebugPrintln("Loading settings from EEPROM...");

  // Check EEPROM version
  uint8_t storedVersion = EEPROM.read(EEPROM_ADDRESS_VERSION);
  if (storedVersion != EEPROM_VERSION) {
    mqttDebugPrintf("EEPROM version mismatch (found %d, expected %d). Initializing EEPROM...\n", storedVersion, EEPROM_VERSION);
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
  if (cooldownValue >= 0 && cooldownValue <= 50) {  // Valid range: 0 to 5.0 seconds (stored as integer with one decimal place)
    cooldownDuration = cooldownValue / 10.0;  // Convert back to float
  } else {
    mqttDebugPrintln("Invalid cooldown value in EEPROM. Setting to default (0.0).");
    cooldownDuration = 0.0;  // Default value
  }

  // Load and validate Camera Settings
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // Framesize
    uint8_t framesizeValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 0);
    if (framesizeValue >= FRAMESIZE_QQVGA && framesizeValue <= FRAMESIZE_UXGA) {
      s->set_framesize(s, (framesize_t)framesizeValue);
    } else {
      mqttDebugPrintln("Invalid framesize in EEPROM. Setting to default (VGA).");
      s->set_framesize(s, FRAMESIZE_VGA);  // Default value
    }

    // JPEG Quality
    uint8_t qualityValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 1);
    if (qualityValue >= 10 && qualityValue <= 63) {
      s->set_quality(s, qualityValue);
    } else {
      mqttDebugPrintln("Invalid JPEG quality in EEPROM. Setting to default (30).");
      s->set_quality(s, 30);  // Default value
    }

    // Brightness
    int8_t brightnessValue = (int8_t)EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 2);
    if (brightnessValue >= -2 && brightnessValue <= 2) {
      s->set_brightness(s, brightnessValue);
    } else {
      mqttDebugPrintln("Invalid brightness in EEPROM. Setting to default (0).");
      s->set_brightness(s, 0);  // Default value
    }

    // Contrast
    int8_t contrastValue = (int8_t)EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 3);
    if (contrastValue >= -2 && contrastValue <= 2) {
      s->set_contrast(s, contrastValue);
    } else {
      mqttDebugPrintln("Invalid contrast in EEPROM. Setting to default (0).");
      s->set_contrast(s, 0);  // Default value
    }

    // Saturation
    int8_t saturationValue = (int8_t)EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 4);
    if (saturationValue >= -2 && saturationValue <= 2) {
      s->set_saturation(s, saturationValue);
    } else {
      mqttDebugPrintln("Invalid saturation in EEPROM. Setting to default (0).");
      s->set_saturation(s, 0);  // Default value
    }

    // Automatic White Balance (AWB)
    uint8_t awbValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 5);
    if (awbValue == 0 || awbValue == 1) {
      s->set_whitebal(s, awbValue == 1);
    } else {
      mqttDebugPrintln("Invalid AWB value in EEPROM. Setting to default (enabled).");
      s->set_whitebal(s, true);  // Default value
    }

    // Special Effect
    uint8_t effectValue = EEPROM.read(EEPROM_ADDRESS_CAMERA_SETTINGS + 6);
    if (effectValue >= 0 && effectValue <= 6) {
      s->set_special_effect(s, effectValue);
    } else {
      mqttDebugPrintln("Invalid special effect in EEPROM. Setting to default (No Effect).");
      s->set_special_effect(s, 0);  // Default value
    }

    mqttDebugPrintln("Camera settings loaded from EEPROM.");
  } else {
    mqttDebugPrintln("Failed to get camera sensor. Cannot load camera settings from EEPROM.");
  }
}
