#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

namespace AppConfig {
  // WiFiManager sẽ tự lưu SSID/password vào bộ nhớ flash của ESP32.
  // Khi chưa có WiFi, ESP32 sẽ phát AP để bạn cấu hình nhanh.
  constexpr char WIFI_MANAGER_AP_NAME[] = "SmartTarp_Setup";
  constexpr char WIFI_MANAGER_AP_PASSWORD[] = "12345678";
  constexpr uint16_t WIFI_MANAGER_PORTAL_TIMEOUT_SEC = 180;

  // ThingsBoard MQTT
  constexpr char THINGSBOARD_HOST[] = "mqtt.thingsboard.cloud";
  constexpr uint16_t THINGSBOARD_PORT = 8883;

  // Device Access Token trong ThingsBoard
  constexpr char THINGSBOARD_TOKEN[] = "7r9VAupeLY0aqH7Sz6uD";

  constexpr char TOPIC_TELEMETRY[] = "v1/devices/me/telemetry";
  constexpr char TOPIC_ATTRIBUTES[] = "v1/devices/me/attributes";
  constexpr char TOPIC_RPC_REQUEST[] = "v1/devices/me/rpc/request/+";

  constexpr uint8_t PIN_RAIN = 33;
  constexpr uint8_t PIN_LDR = 34;
  constexpr uint8_t PIN_WIND = 35;
  constexpr uint8_t PIN_LIMIT_OPEN = 18;
  constexpr uint8_t PIN_LIMIT_CLOSE = 19;
  constexpr uint8_t PIN_BUTTON_OPEN = 21;
  constexpr uint8_t PIN_BUTTON_CLOSE = 22;
  constexpr uint8_t PIN_BUTTON_AUTO = 23;
  constexpr uint8_t PIN_MOTOR_IN1 = 26;
  constexpr uint8_t PIN_MOTOR_IN2 = 27;
  constexpr uint8_t PIN_MOTOR_EN = 25;

  constexpr bool RAIN_ACTIVE_LOW = true;
  constexpr bool LIMIT_ACTIVE_LOW = false;
  constexpr bool LDR_ACTIVE_LOW = true;

  constexpr uint32_t SENSOR_SAMPLE_INTERVAL_MS = 200;
  constexpr uint32_t BUTTON_SCAN_INTERVAL_MS = 20;
  constexpr uint32_t CONTROL_INTERVAL_MS = 50;
  constexpr uint32_t MQTT_LOOP_INTERVAL_MS = 30;
  constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
  constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
  constexpr uint32_t SENSOR_PUBLISH_INTERVAL_MS = 2000;
  constexpr uint32_t STATUS_PUBLISH_INTERVAL_MS = 1500;
  constexpr uint32_t HISTORY_PUBLISH_INTERVAL_MS = 5000;
  constexpr uint32_t AUTO_DECISION_INTERVAL_MS = 600;
  constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
  constexpr uint32_t MOTOR_TIMEOUT_MS = 15000;
  constexpr uint32_t MOTOR_DIRECTION_GUARD_MS = 700;
  constexpr uint8_t SYSTEM_HISTORY_SIZE = 20;

  constexpr float LIGHT_FILTER_ALPHA = 0.18f;
  constexpr float WIND_FILTER_ALPHA = 0.20f;
  constexpr float LIGHT_DAY_THRESHOLD_PERCENT = 62.0f;
  constexpr float LIGHT_NIGHT_THRESHOLD_PERCENT = 35.0f;
  constexpr float WIND_STRONG_THRESHOLD_KMH = 18.0f;
  constexpr float WIND_VOLTAGE_DEADZONE = 0.18f;
  constexpr float WIND_SPEED_SCALE = 23.0f;
}

enum SystemMode : uint8_t {
  MODE_AUTO = 0,
  MODE_MANUAL = 1
};

enum TarpState : uint8_t {
  TARP_UNKNOWN = 0,
  TARP_OPEN = 1,
  TARP_CLOSED = 2,
  TARP_OPENING = 3,
  TARP_CLOSING = 4,
  TARP_STOPPED = 5
};

enum MotorState : uint8_t {
  MOTOR_STOPPED = 0,
  MOTOR_OPENING = 1,
  MOTOR_CLOSING = 2
};

enum CommandType : uint8_t {
  CMD_NONE = 0,
  CMD_OPEN = 1,
  CMD_CLOSE = 2,
  CMD_STOP = 3,
  CMD_AUTO_ON = 4,
  CMD_AUTO_OFF = 5,
  CMD_RESET_WIFI = 6
};

struct SensorSnapshot {
  bool rainDetected;
  uint16_t rainRaw;
  uint16_t lightRaw;
  float lightPercent;
  bool isNight;
  uint16_t windRaw;
  float windVoltage;
  float windSpeed;
  unsigned long updatedAtMs;
};

struct RuntimeSnapshot {
  SystemMode mode;
  TarpState tarpState;
  MotorState motorState;
  bool limitOpen;
  bool limitClose;
  bool wifiConnected;
  bool mqttConnected;
  int32_t wifiRssi;
  bool sensorDirty;
  bool statusDirty;
  unsigned long lastMotorChangeMs;
  unsigned long motorStartMs;
  unsigned long lastAutoEvalMs;
  unsigned long lastSensorPublishMs;
  unsigned long lastStatusPublishMs;
  unsigned long lastHistoryPublishMs;
  unsigned long lastWiFiAttemptMs;
  unsigned long lastMqttAttemptMs;
  unsigned long lastCommandMs;
};

struct AutoThresholds {
  float lightDayPercent;
  float lightNightPercent;
  float windStrongKmh;
};

struct CommandMessage {
  CommandType type;
  char source[16];
  unsigned long queuedAtMs;
};

struct SystemHistoryItem {
  unsigned long atMs;
  char level[10];
  char event[28];
  char detail[96];
};

struct ButtonTracker {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastDebounceMs;
  CommandType pressCommand;
  bool isAutoToggle;
};

WiFiManager wifiManager;
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

SemaphoreHandle_t stateMutex = nullptr;
QueueHandle_t commandQueue = nullptr;

SensorSnapshot sensorState = {};
RuntimeSnapshot runtimeState = {};

ButtonTracker buttons[] = {
  {AppConfig::PIN_BUTTON_OPEN, HIGH, HIGH, 0, CMD_OPEN, false},
  {AppConfig::PIN_BUTTON_CLOSE, HIGH, HIGH, 0, CMD_CLOSE, false},
  {AppConfig::PIN_BUTTON_AUTO, HIGH, HIGH, 0, CMD_NONE, true}
};

float filteredLightRaw = 0.0f;
float filteredWindRaw = 0.0f;

AutoThresholds autoThresholds = {
  AppConfig::LIGHT_DAY_THRESHOLD_PERCENT,
  AppConfig::LIGHT_NIGHT_THRESHOLD_PERCENT,
  AppConfig::WIND_STRONG_THRESHOLD_KMH
};

char mqttClientId[48] = {0};

SystemHistoryItem systemHistory[AppConfig::SYSTEM_HISTORY_SIZE] = {};
uint8_t systemHistoryHead = 0;
uint8_t systemHistoryCount = 0;
bool wifiPortalRequested = false;


void addSystemHistory(const char* level, const char* event, const char* detail) {
  SystemHistoryItem item = {};
  item.atMs = millis();
  snprintf(item.level, sizeof(item.level), "%s", level);
  snprintf(item.event, sizeof(item.event), "%s", event);
  snprintf(item.detail, sizeof(item.detail), "%s", detail);

  for (char* p = item.event; *p; ++p) {
    if (*p == '\"' || *p == '\\' || *p == '\n' || *p == '\r') *p = '\'';
  }
  for (char* p = item.detail; *p; ++p) {
    if (*p == '\"' || *p == '\\' || *p == '\n' || *p == '\r') *p = '\'';
  }

  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }

  systemHistory[systemHistoryHead] = item;
  systemHistoryHead = (systemHistoryHead + 1) % AppConfig::SYSTEM_HISTORY_SIZE;
  if (systemHistoryCount < AppConfig::SYSTEM_HISTORY_SIZE) {
    systemHistoryCount++;
  }

  if (stateMutex != nullptr) {
    runtimeState.statusDirty = true;
    xSemaphoreGive(stateMutex);
  }

  Serial.printf("[HISTORY] %lu | %s | %s | %s\n", item.atMs, item.level, item.event, item.detail);
}

void startWiFiConfigPortal(const char* reason) {
  addSystemHistory("INFO", "WIFI_PORTAL", reason);
  Serial.println("[WIFI] Open config portal");
  Serial.printf("[WIFI] AP: %s | PASS: %s\n", AppConfig::WIFI_MANAGER_AP_NAME, AppConfig::WIFI_MANAGER_AP_PASSWORD);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  wifiManager.setConfigPortalTimeout(AppConfig::WIFI_MANAGER_PORTAL_TIMEOUT_SEC);
  wifiManager.setConnectTimeout(15);

  const bool connected = wifiManager.autoConnect(
    AppConfig::WIFI_MANAGER_AP_NAME,
    AppConfig::WIFI_MANAGER_AP_PASSWORD
  );

  if (connected) {
    char detail[96];
    snprintf(detail, sizeof(detail), "SSID=%s IP=%s", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    addSystemHistory("INFO", "WIFI_CONNECTED", detail);
  } else {
    addSystemHistory("WARN", "WIFI_PORTAL_TIMEOUT", "No WiFi configured within timeout");
  }
}

void requestWiFiReset() {
  wifiPortalRequested = true;
  addSystemHistory("WARN", "WIFI_RESET_REQUEST", "Reset WiFi by command/RPC");
}

const char* modeToString(SystemMode mode) {
  return mode == MODE_MANUAL ? "MANUAL" : "AUTO";
}

const char* tarpStateToString(TarpState state) {
  switch (state) {
    case TARP_OPEN: return "OPEN";
    case TARP_CLOSED: return "CLOSED";
    case TARP_OPENING: return "OPENING";
    case TARP_CLOSING: return "CLOSING";
    case TARP_STOPPED: return "STOPPED";
    default: return "UNKNOWN";
  }
}

const char* motorStateToString(MotorState state) {
  switch (state) {
    case MOTOR_OPENING: return "OPENING";
    case MOTOR_CLOSING: return "CLOSING";
    default: return "STOPPED";
  }
}

float applyFilter(float currentValue, float inputValue, float alpha) {
  return currentValue + alpha * (inputValue - currentValue);
}

float computeLightPercent(uint16_t rawValue) {
  float percent = AppConfig::LDR_ACTIVE_LOW
    ? 100.0f - ((static_cast<float>(rawValue) / 4095.0f) * 100.0f)
    : (static_cast<float>(rawValue) / 4095.0f) * 100.0f;

  if (percent < 0.0f) return 0.0f;
  if (percent > 100.0f) return 100.0f;
  return percent;
}

float computeWindSpeed(float voltage) {
  if (voltage <= AppConfig::WIND_VOLTAGE_DEADZONE) {
    return 0.0f;
  }

  const float speed = (voltage - AppConfig::WIND_VOLTAGE_DEADZONE) * AppConfig::WIND_SPEED_SCALE;
  return speed < 0.0f ? 0.0f : speed;
}

bool readRainPin() {
  return digitalRead(AppConfig::PIN_RAIN) == (AppConfig::RAIN_ACTIVE_LOW ? LOW : HIGH);
}

bool readLimitPin(uint8_t pin) {
  return digitalRead(pin) == (AppConfig::LIMIT_ACTIVE_LOW ? LOW : HIGH);
}

void setMotorOutputs(MotorState direction) {
  switch (direction) {
    case MOTOR_OPENING:
      digitalWrite(AppConfig::PIN_MOTOR_IN1, HIGH);
      digitalWrite(AppConfig::PIN_MOTOR_IN2, LOW);
      digitalWrite(AppConfig::PIN_MOTOR_EN, HIGH);
      break;

    case MOTOR_CLOSING:
      digitalWrite(AppConfig::PIN_MOTOR_IN1, LOW);
      digitalWrite(AppConfig::PIN_MOTOR_IN2, HIGH);
      digitalWrite(AppConfig::PIN_MOTOR_EN, HIGH);
      break;

    default:
      digitalWrite(AppConfig::PIN_MOTOR_IN1, LOW);
      digitalWrite(AppConfig::PIN_MOTOR_IN2, LOW);
      digitalWrite(AppConfig::PIN_MOTOR_EN, LOW);
      break;
  }
}

SensorSnapshot copySensorState() {
  SensorSnapshot copy;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  copy = sensorState;
  xSemaphoreGive(stateMutex);
  return copy;
}

RuntimeSnapshot copyRuntimeState() {
  RuntimeSnapshot copy;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  copy = runtimeState;
  xSemaphoreGive(stateMutex);
  return copy;
}

AutoThresholds copyAutoThresholds() {
  AutoThresholds copy;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  copy = autoThresholds;
  xSemaphoreGive(stateMutex);
  return copy;
}

void enqueueCommand(CommandType type, const char* source) {
  if (type == CMD_NONE) return;

  CommandMessage message = {};
  message.type = type;
  message.queuedAtMs = millis();
  snprintf(message.source, sizeof(message.source), "%s", source);
  if (xQueueSend(commandQueue, &message, 0) != pdPASS) {
    addSystemHistory("WARN", "COMMAND_QUEUE_FULL", source);
  }
}

bool canChangeMotorDirection(unsigned long now, const RuntimeSnapshot& state) {
  return (now - state.lastMotorChangeMs) >= AppConfig::MOTOR_DIRECTION_GUARD_MS;
}

void stopMotor(const char* reason) {
  setMotorOutputs(MOTOR_STOPPED);

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  runtimeState.limitOpen = readLimitPin(AppConfig::PIN_LIMIT_OPEN);
  runtimeState.limitClose = readLimitPin(AppConfig::PIN_LIMIT_CLOSE);
  runtimeState.motorState = MOTOR_STOPPED;
  runtimeState.lastMotorChangeMs = millis();
  runtimeState.statusDirty = true;

  if (runtimeState.limitOpen) {
    runtimeState.tarpState = TARP_OPEN;
  } else if (runtimeState.limitClose) {
    runtimeState.tarpState = TARP_CLOSED;
  } else {
    runtimeState.tarpState = TARP_STOPPED;
  }

  xSemaphoreGive(stateMutex);

  Serial.printf("[CTRL] Motor stopped: %s\n", reason);
  addSystemHistory("INFO", "MOTOR_STOPPED", reason);
}

bool beginOpenMotion(unsigned long now) {
  RuntimeSnapshot state = copyRuntimeState();

  if (state.limitOpen) {
    stopMotor("open limit already active");
    return false;
  }

  if (!canChangeMotorDirection(now, state)) {
    return false;
  }

  setMotorOutputs(MOTOR_OPENING);

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  runtimeState.motorState = MOTOR_OPENING;
  runtimeState.tarpState = TARP_OPENING;
  runtimeState.motorStartMs = now;
  runtimeState.lastMotorChangeMs = now;
  runtimeState.lastCommandMs = now;
  runtimeState.statusDirty = true;
  xSemaphoreGive(stateMutex);

  Serial.println("[CTRL] Opening tarp");
  addSystemHistory("INFO", "MOTOR_OPENING", "Open command accepted");
  return true;
}

bool beginCloseMotion(unsigned long now) {
  RuntimeSnapshot state = copyRuntimeState();

  if (state.limitClose) {
    stopMotor("close limit already active");
    return false;
  }

  if (!canChangeMotorDirection(now, state)) {
    return false;
  }

  setMotorOutputs(MOTOR_CLOSING);

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  runtimeState.motorState = MOTOR_CLOSING;
  runtimeState.tarpState = TARP_CLOSING;
  runtimeState.motorStartMs = now;
  runtimeState.lastMotorChangeMs = now;
  runtimeState.lastCommandMs = now;
  runtimeState.statusDirty = true;
  xSemaphoreGive(stateMutex);

  Serial.println("[CTRL] Closing tarp");
  addSystemHistory("INFO", "MOTOR_CLOSING", "Close command accepted");
  return true;
}

String extractCommandToken(String payload) {
  payload.trim();

  if (payload.startsWith("{")) {
    int keyIndex = payload.indexOf("\"params\"");
    if (keyIndex >= 0) {
      int colonIndex = payload.indexOf(':', keyIndex);
      int firstQuote = payload.indexOf('"', colonIndex + 1);
      int secondQuote = payload.indexOf('"', firstQuote + 1);

      if (firstQuote >= 0 && secondQuote > firstQuote) {
        return payload.substring(firstQuote + 1, secondQuote);
      }

      String numberOrBool = payload.substring(colonIndex + 1);
      numberOrBool.replace("}", "");
      numberOrBool.trim();
      return numberOrBool;
    }

    keyIndex = payload.indexOf("\"command\"");
    if (keyIndex >= 0) {
      int colonIndex = payload.indexOf(':', keyIndex);
      int firstQuote = payload.indexOf('"', colonIndex + 1);
      int secondQuote = payload.indexOf('"', firstQuote + 1);

      if (firstQuote >= 0 && secondQuote > firstQuote) {
        return payload.substring(firstQuote + 1, secondQuote);
      }
    }

    keyIndex = payload.indexOf("\"method\"");
    if (keyIndex >= 0) {
      int colonIndex = payload.indexOf(':', keyIndex);
      int firstQuote = payload.indexOf('"', colonIndex + 1);
      int secondQuote = payload.indexOf('"', firstQuote + 1);

      if (firstQuote >= 0 && secondQuote > firstQuote) {
        return payload.substring(firstQuote + 1, secondQuote);
      }
    }
  }

  payload.replace("\"", "");
  payload.replace("{", "");
  payload.replace("}", "");
  payload.trim();

  return payload;
}

CommandType parseCommandType(String payload) {
  payload = extractCommandToken(payload);
  payload.toUpperCase();

  if (payload == "OPEN" || payload == "1") return CMD_OPEN;
  if (payload == "CLOSE" || payload == "2") return CMD_CLOSE;
  if (payload == "STOP" || payload == "0") return CMD_STOP;
  if (payload == "AUTO_ON" || payload == "AUTO") return CMD_AUTO_ON;
  if (payload == "AUTO_OFF" || payload == "MANUAL") return CMD_AUTO_OFF;
  if (payload == "RESET_WIFI" || payload == "WIFI_RESET") return CMD_RESET_WIFI;

  return CMD_NONE;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

bool extractJsonFloat(const String& payload, const char* key, float& value) {
  const String lookup = String("\"") + key + "\"";
  const int keyIndex = payload.indexOf(lookup);

  if (keyIndex < 0) return false;

  const int colonIndex = payload.indexOf(':', keyIndex + lookup.length());
  if (colonIndex < 0) return false;

  int valueEnd = payload.indexOf(',', colonIndex + 1);
  const int braceEnd = payload.indexOf('}', colonIndex + 1);

  if (valueEnd < 0 || (braceEnd >= 0 && braceEnd < valueEnd)) {
    valueEnd = braceEnd;
  }

  if (valueEnd < 0) {
    valueEnd = payload.length();
  }

  String numberText = payload.substring(colonIndex + 1, valueEnd);
  numberText.trim();

  value = numberText.toFloat();
  return numberText.length() > 0;
}

bool applyConfigMessage(const String& payload) {
  AutoThresholds updated = copyAutoThresholds();
  float incomingValue = 0.0f;
  bool changed = false;

  if (extractJsonFloat(payload, "lightDayPercent", incomingValue)) {
    updated.lightDayPercent = clampFloat(incomingValue, 0.0f, 100.0f);
    changed = true;
  }

  if (extractJsonFloat(payload, "lightNightPercent", incomingValue)) {
    updated.lightNightPercent = clampFloat(incomingValue, 0.0f, 100.0f);
    changed = true;
  }

  if (extractJsonFloat(payload, "windStrongKmh", incomingValue)) {
    updated.windStrongKmh = clampFloat(incomingValue, 0.0f, 120.0f);
    changed = true;
  }

  if (!changed) return false;

  if (updated.lightNightPercent >= updated.lightDayPercent) {
    Serial.println("[CONFIG] Ignored thresholds: night light must be lower than day light");
    return false;
  }

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  autoThresholds = updated;
  runtimeState.statusDirty = true;
  runtimeState.sensorDirty = true;
  xSemaphoreGive(stateMutex);

  Serial.printf(
    "[CONFIG] Thresholds updated day=%.1f night=%.1f wind=%.1f\n",
    updated.lightDayPercent,
    updated.lightNightPercent,
    updated.windStrongKmh
  );
  addSystemHistory("INFO", "CONFIG_UPDATED", "Thresholds changed from ThingsBoard");

  return true;
}

String getRpcRequestId(const char* topic) {
  String topicText(topic);
  int lastSlash = topicText.lastIndexOf('/');
  if (lastSlash < 0) return "";
  return topicText.substring(lastSlash + 1);
}

void sendRpcResponse(const String& requestId, bool success, const char* message) {
  if (!mqttClient.connected() || requestId.length() == 0) return;

  char topic[96];
  char payload[160];

  snprintf(
    topic,
    sizeof(topic),
    "v1/devices/me/rpc/response/%s",
    requestId.c_str()
  );

  snprintf(
    payload,
    sizeof(payload),
    "{\"success\":%s,\"message\":\"%s\"}",
    success ? "true" : "false",
    message
  );

  mqttClient.publish(topic, payload);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char buffer[512] = {0};
  const unsigned int safeLength = length >= sizeof(buffer) ? sizeof(buffer) - 1 : length;
  memcpy(buffer, payload, safeLength);
  buffer[safeLength] = '\0';

  String payloadText(buffer);
  String topicText(topic);

  Serial.printf("[MQTT] Topic: %s\n", topic);
  Serial.printf("[MQTT] Payload: %s\n", buffer);

  if (topicText.startsWith("v1/devices/me/rpc/request/")) {
    String requestId = getRpcRequestId(topic);

    if (payloadText.indexOf("lightDayPercent") >= 0 ||
        payloadText.indexOf("lightNightPercent") >= 0 ||
        payloadText.indexOf("windStrongKmh") >= 0) {
      bool ok = applyConfigMessage(payloadText);
      sendRpcResponse(requestId, ok, ok ? "Config updated" : "Invalid config");
      return;
    }

    CommandType commandType = parseCommandType(payloadText);

    if (commandType == CMD_NONE) {
      addSystemHistory("WARN", "UNKNOWN_RPC", payloadText.c_str());
      sendRpcResponse(requestId, false, "Unknown command");
      return;
    }

    enqueueCommand(commandType, "rpc");
    sendRpcResponse(requestId, true, "Command queued");
    return;
  }

  if (topicText == AppConfig::TOPIC_ATTRIBUTES) {
    applyConfigMessage(payloadText);
  }
}

void ensureWiFi(unsigned long now) {
  const wl_status_t wifiStatus = WiFi.status();

  if (wifiStatus == WL_CONNECTED) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (!runtimeState.wifiConnected) {
      runtimeState.statusDirty = true;
    }
    runtimeState.wifiConnected = true;
    runtimeState.wifiRssi = WiFi.RSSI();
    xSemaphoreGive(stateMutex);
    return;
  }

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool shouldAttempt = (now - runtimeState.lastWiFiAttemptMs) >= AppConfig::WIFI_RECONNECT_INTERVAL_MS;

  if (runtimeState.wifiConnected || runtimeState.mqttConnected) {
    runtimeState.statusDirty = true;
  }

  runtimeState.wifiConnected = false;
  runtimeState.mqttConnected = false;
  runtimeState.wifiRssi = 0;

  if (shouldAttempt) {
    runtimeState.lastWiFiAttemptMs = now;
  }

  xSemaphoreGive(stateMutex);

  if (shouldAttempt) {
    Serial.println("[NET] Reconnecting WiFi");
    WiFi.reconnect();
    addSystemHistory("WARN", "WIFI_RECONNECT", "Trying saved WiFi credentials");
  }
}

bool mqttConnect() {
  return mqttClient.connect(
    mqttClientId,
    AppConfig::THINGSBOARD_TOKEN,
    nullptr,
    AppConfig::TOPIC_ATTRIBUTES,
    1,
    true,
    "{\"online\":false}"
  );
}

void ensureMqtt(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (mqttClient.connected()) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (!runtimeState.mqttConnected) {
      runtimeState.statusDirty = true;
    }
    runtimeState.mqttConnected = true;
    xSemaphoreGive(stateMutex);
    return;
  }

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool shouldAttempt = (now - runtimeState.lastMqttAttemptMs) >= AppConfig::MQTT_RECONNECT_INTERVAL_MS;
  runtimeState.mqttConnected = false;
  runtimeState.statusDirty = true;

  if (shouldAttempt) {
    runtimeState.lastMqttAttemptMs = now;
  }

  xSemaphoreGive(stateMutex);

  if (!shouldAttempt) return;

  Serial.println("[NET] Reconnecting ThingsBoard MQTT");

  if (mqttConnect()) {
    mqttClient.subscribe(AppConfig::TOPIC_RPC_REQUEST, 1);
    mqttClient.subscribe(AppConfig::TOPIC_ATTRIBUTES, 1);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    runtimeState.mqttConnected = true;
    runtimeState.statusDirty = true;
    runtimeState.sensorDirty = true;
    xSemaphoreGive(stateMutex);

    mqttClient.publish(AppConfig::TOPIC_ATTRIBUTES, "{\"online\":true}", true);

    Serial.println("[NET] Connected to ThingsBoard");
    addSystemHistory("INFO", "MQTT_CONNECTED", "ThingsBoard connected");
  } else {
    Serial.printf("[NET] ThingsBoard MQTT failed rc=%d\n", mqttClient.state());
    char mqttFailDetail[40];
    snprintf(mqttFailDetail, sizeof(mqttFailDetail), "rc=%d", mqttClient.state());
    addSystemHistory("WARN", "MQTT_FAILED", mqttFailDetail);
  }
}

bool publishPayload(const char* topic, const char* payload, bool retained = false) {
  if (!mqttClient.connected()) {
    return false;
  }

  return mqttClient.publish(topic, payload, retained);
}

\nvoid publishSystemHistory(bool force) {\n  RuntimeSnapshot runtime = copyRuntimeState();\n\n  if (!runtime.mqttConnected) return;\n\n  const unsigned long now = millis();\n\n  if (!force && (now - runtime.lastHistoryPublishMs) < AppConfig::HISTORY_PUBLISH_INTERVAL_MS) {\n    return;\n  }\n\n  SystemHistoryItem copy[AppConfig::SYSTEM_HISTORY_SIZE];\n  uint8_t count = 0;\n  uint8_t head = 0;\n\n  xSemaphoreTake(stateMutex, portMAX_DELAY);\n  count = systemHistoryCount;\n  head = systemHistoryHead;\n  for (uint8_t i = 0; i < count; i++) {\n    const uint8_t index = (head + AppConfig::SYSTEM_HISTORY_SIZE - count + i) % AppConfig::SYSTEM_HISTORY_SIZE;\n    copy[i] = systemHistory[index];\n  }\n  runtimeState.lastHistoryPublishMs = now;\n  xSemaphoreGive(stateMutex);\n\n  char payload[900];\n  size_t used = 0;\n  used += snprintf(payload + used, sizeof(payload) - used, "{\"systemHistory\":[");\n\n  for (uint8_t i = 0; i < count && used < sizeof(payload); i++) {\n    used += snprintf(\n      payload + used,\n      sizeof(payload) - used,\n      "%s{\"t\":%lu,\"level\":\"%s\",\"event\":\"%s\",\"detail\":\"%s\"}",\n      i == 0 ? "" : ",",\n      copy[i].atMs,\n      copy[i].level,\n      copy[i].event,\n      copy[i].detail\n    );\n  }\n\n  snprintf(payload + used, sizeof(payload) - used, "]}");\n  publishPayload(AppConfig::TOPIC_TELEMETRY, payload, false);\n}\n
void publishSensorTelemetry(bool force) {
  RuntimeSnapshot runtime = copyRuntimeState();

  if (!runtime.mqttConnected) return;

  const unsigned long now = millis();

  if (!force &&
      !runtime.sensorDirty &&
      (now - runtime.lastSensorPublishMs) < AppConfig::SENSOR_PUBLISH_INTERVAL_MS) {
    return;
  }

  SensorSnapshot sensors = copySensorState();

  char payload[512];

  snprintf(
    payload,
    sizeof(payload),
    "{"
      "\"rainDetected\":%s,"
      "\"rainRaw\":%u,"
      "\"lightRaw\":%u,"
      "\"lightPercent\":%.1f,"
      "\"isNight\":%s,"
      "\"windRaw\":%u,"
      "\"windVoltage\":%.2f,"
      "\"windSpeed\":%.1f"
    "}",
    sensors.rainDetected ? "true" : "false",
    sensors.rainRaw,
    sensors.lightRaw,
    sensors.lightPercent,
    sensors.isNight ? "true" : "false",
    sensors.windRaw,
    sensors.windVoltage,
    sensors.windSpeed
  );

  publishPayload(AppConfig::TOPIC_TELEMETRY, payload, false);

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  runtimeState.sensorDirty = false;
  runtimeState.lastSensorPublishMs = now;
  xSemaphoreGive(stateMutex);
}

void publishStatusTelemetry(bool force) {
  RuntimeSnapshot runtime = copyRuntimeState();

  if (!runtime.mqttConnected) return;

  const unsigned long now = millis();

  if (!force &&
      !runtime.statusDirty &&
      (now - runtime.lastStatusPublishMs) < AppConfig::STATUS_PUBLISH_INTERVAL_MS) {
    return;
  }

  AutoThresholds thresholds = copyAutoThresholds();

  char attrPayload[512];

  snprintf(
    attrPayload,
    sizeof(attrPayload),
    "{"
      "\"mode\":\"%s\","
      "\"tarpState\":\"%s\","
      "\"motorState\":\"%s\","
      "\"limitOpen\":%s,"
      "\"limitClose\":%s,"
      "\"online\":true,"
      "\"wifiConnected\":%s,"
      "\"mqttConnected\":%s,"
      "\"rssi\":%ld,"
      "\"lightDayPercent\":%.1f,"
      "\"lightNightPercent\":%.1f,"
      "\"windStrongKmh\":%.1f"
    "}",
    modeToString(runtime.mode),
    tarpStateToString(runtime.tarpState),
    motorStateToString(runtime.motorState),
    runtime.limitOpen ? "true" : "false",
    runtime.limitClose ? "true" : "false",
    runtime.wifiConnected ? "true" : "false",
    runtime.mqttConnected ? "true" : "false",
    static_cast<long>(runtime.wifiRssi),
    thresholds.lightDayPercent,
    thresholds.lightNightPercent,
    thresholds.windStrongKmh
  );

  publishPayload(AppConfig::TOPIC_ATTRIBUTES, attrPayload, true);

  char telemetryPayload[256];

  snprintf(
    telemetryPayload,
    sizeof(telemetryPayload),
    "{"
      "\"mode\":\"%s\","
      "\"tarpState\":\"%s\","
      "\"motorState\":\"%s\","
      "\"limitOpen\":%s,"
      "\"limitClose\":%s,"
      "\"rssi\":%ld"
    "}",
    modeToString(runtime.mode),
    tarpStateToString(runtime.tarpState),
    motorStateToString(runtime.motorState),
    runtime.limitOpen ? "true" : "false",
    runtime.limitClose ? "true" : "false",
    static_cast<long>(runtime.wifiRssi)
  );

  publishPayload(AppConfig::TOPIC_TELEMETRY, telemetryPayload, false);

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  runtimeState.statusDirty = false;
  runtimeState.lastStatusPublishMs = now;
  xSemaphoreGive(stateMutex);
}

void handleCommand(const CommandMessage& message, unsigned long now) {
  switch (message.type) {
    case CMD_OPEN:
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      runtimeState.mode = MODE_MANUAL;
      runtimeState.statusDirty = true;
      xSemaphoreGive(stateMutex);
      addSystemHistory("INFO", "CMD_OPEN", message.source);
      beginOpenMotion(now);
      break;

    case CMD_CLOSE:
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      runtimeState.mode = MODE_MANUAL;
      runtimeState.statusDirty = true;
      xSemaphoreGive(stateMutex);
      addSystemHistory("INFO", "CMD_CLOSE", message.source);
      beginCloseMotion(now);
      break;

    case CMD_STOP:
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      runtimeState.mode = MODE_MANUAL;
      runtimeState.statusDirty = true;
      xSemaphoreGive(stateMutex);
      addSystemHistory("INFO", "CMD_STOP", message.source);
      stopMotor(message.source);
      break;

    case CMD_AUTO_ON:
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      runtimeState.mode = MODE_AUTO;
      runtimeState.statusDirty = true;
      xSemaphoreGive(stateMutex);
      Serial.println("[CTRL] Auto mode enabled");
      addSystemHistory("INFO", "AUTO_ON", message.source);
      break;

    case CMD_AUTO_OFF:
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      runtimeState.mode = MODE_MANUAL;
      runtimeState.statusDirty = true;
      xSemaphoreGive(stateMutex);
      Serial.println("[CTRL] Auto mode disabled");
      addSystemHistory("INFO", "AUTO_OFF", message.source);
      break;

    case CMD_RESET_WIFI:
      requestWiFiReset();
      break;

    default:
      break;
  }
}

void evaluateAutomaticLogic(unsigned long now) {
  SensorSnapshot sensors = copySensorState();
  RuntimeSnapshot runtime = copyRuntimeState();
  AutoThresholds thresholds = copyAutoThresholds();

  if (runtime.mode != MODE_AUTO) return;

  if ((now - runtime.lastAutoEvalMs) < AppConfig::AUTO_DECISION_INTERVAL_MS) {
    return;
  }

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  runtimeState.lastAutoEvalMs = now;
  xSemaphoreGive(stateMutex);

  const bool strongWind = sensors.windSpeed >= thresholds.windStrongKmh;
  const bool sunny = sensors.lightPercent >= thresholds.lightDayPercent;
  const bool shouldCloseForNight = sensors.isNight && !sensors.rainDetected;
  const bool shouldOpen = sensors.rainDetected || sunny;

  if (strongWind) {
    if (runtime.tarpState != TARP_CLOSED && runtime.tarpState != TARP_CLOSING) {
      addSystemHistory("INFO", "AUTO_CLOSE", "Strong wind");
      beginCloseMotion(now);
    }
    return;
  }

  if (shouldCloseForNight) {
    if (runtime.tarpState != TARP_CLOSED && runtime.tarpState != TARP_CLOSING) {
      addSystemHistory("INFO", "AUTO_CLOSE", "Night mode");
      beginCloseMotion(now);
    }
    return;
  }

  if (shouldOpen && runtime.tarpState != TARP_OPEN && runtime.tarpState != TARP_OPENING) {
    addSystemHistory("INFO", "AUTO_OPEN", sensors.rainDetected ? "Rain detected" : "Light threshold reached");
    beginOpenMotion(now);
  }
}

void taskSensors(void* parameter) {
  (void) parameter;
  unsigned long lastRun = 0;

  for (;;) {
    const unsigned long now = millis();

    if ((now - lastRun) >= AppConfig::SENSOR_SAMPLE_INTERVAL_MS) {
      lastRun = now;

      const uint16_t rainRaw = readRainPin() ? 1 : 0;
      const uint16_t lightRaw = analogRead(AppConfig::PIN_LDR);
      const uint16_t windRaw = analogRead(AppConfig::PIN_WIND);

      filteredLightRaw = filteredLightRaw == 0.0f
        ? lightRaw
        : applyFilter(filteredLightRaw, lightRaw, AppConfig::LIGHT_FILTER_ALPHA);

      filteredWindRaw = filteredWindRaw == 0.0f
        ? windRaw
        : applyFilter(filteredWindRaw, windRaw, AppConfig::WIND_FILTER_ALPHA);

      const float lightPercent = computeLightPercent(static_cast<uint16_t>(filteredLightRaw));
      const float windVoltage = (filteredWindRaw / 4095.0f) * 3.3f;
      const float windSpeed = computeWindSpeed(windVoltage);

      AutoThresholds thresholds = copyAutoThresholds();

      SensorSnapshot current = copySensorState();
      bool isNight = current.isNight;

      if (isNight) {
        isNight = lightPercent < thresholds.lightDayPercent;
      } else {
        isNight = lightPercent <= thresholds.lightNightPercent;
      }

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      sensorState.rainDetected = rainRaw > 0;
      sensorState.rainRaw = rainRaw;
      sensorState.lightRaw = static_cast<uint16_t>(filteredLightRaw);
      sensorState.lightPercent = lightPercent;
      sensorState.isNight = isNight;
      sensorState.windRaw = static_cast<uint16_t>(filteredWindRaw);
      sensorState.windVoltage = windVoltage;
      sensorState.windSpeed = windSpeed;
      sensorState.updatedAtMs = now;

      runtimeState.sensorDirty = true;
      runtimeState.wifiRssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void processButton(ButtonTracker& button, unsigned long now) {
  const bool reading = digitalRead(button.pin);

  if (reading != button.lastReading) {
    button.lastDebounceMs = now;
    button.lastReading = reading;
  }

  if ((now - button.lastDebounceMs) < AppConfig::BUTTON_DEBOUNCE_MS) {
    return;
  }

  if (reading != button.stableState) {
    button.stableState = reading;

    if (button.stableState == LOW) {
      if (button.isAutoToggle) {
        RuntimeSnapshot runtime = copyRuntimeState();
        enqueueCommand(runtime.mode == MODE_AUTO ? CMD_AUTO_OFF : CMD_AUTO_ON, "button");
      } else {
        enqueueCommand(button.pressCommand, "button");
      }
    }
  }
}

void taskButtons(void* parameter) {
  (void) parameter;

  for (;;) {
    const unsigned long now = millis();

    for (ButtonTracker& button : buttons) {
      processButton(button, now);
    }

    vTaskDelay(pdMS_TO_TICKS(AppConfig::BUTTON_SCAN_INTERVAL_MS));
  }
}

void taskControl(void* parameter) {
  (void) parameter;
  CommandMessage command;

  for (;;) {
    const unsigned long now = millis();

    while (xQueueReceive(commandQueue, &command, 0) == pdPASS) {
      handleCommand(command, now);
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    runtimeState.limitOpen = readLimitPin(AppConfig::PIN_LIMIT_OPEN);
    runtimeState.limitClose = readLimitPin(AppConfig::PIN_LIMIT_CLOSE);

    const MotorState motorState = runtimeState.motorState;
    const unsigned long motorStartMs = runtimeState.motorStartMs;
    const bool limitOpen = runtimeState.limitOpen;
    const bool limitClose = runtimeState.limitClose;
    xSemaphoreGive(stateMutex);

    if (motorState == MOTOR_OPENING && limitOpen) {
      stopMotor("open limit reached");
    } else if (motorState == MOTOR_CLOSING && limitClose) {
      stopMotor("close limit reached");
    } else if (motorState != MOTOR_STOPPED && (now - motorStartMs) >= AppConfig::MOTOR_TIMEOUT_MS) {
      stopMotor("motor timeout");
    }

    evaluateAutomaticLogic(now);

    vTaskDelay(pdMS_TO_TICKS(AppConfig::CONTROL_INTERVAL_MS));
  }
}

void taskMQTT(void* parameter) {
  (void) parameter;

  for (;;) {
    const unsigned long now = millis();

    if (wifiPortalRequested) {
      wifiPortalRequested = false;
      mqttClient.disconnect();
      WiFi.disconnect(true, true);
      wifiManager.resetSettings();
      startWiFiConfigPortal("Reset saved WiFi and open portal");
    }

    ensureWiFi(now);

    if (WiFi.status() == WL_CONNECTED) {
      mqttClient.loop();
      ensureMqtt(now);
      publishSensorTelemetry(false);
      publishStatusTelemetry(false);
      publishSystemHistory(false);
    }

    vTaskDelay(pdMS_TO_TICKS(AppConfig::MQTT_LOOP_INTERVAL_MS));
  }
}

void configurePins() {
  pinMode(AppConfig::PIN_RAIN, INPUT_PULLUP);
  pinMode(AppConfig::PIN_LDR, INPUT);
  pinMode(AppConfig::PIN_WIND, INPUT);

  pinMode(AppConfig::PIN_LIMIT_OPEN, INPUT_PULLUP);
  pinMode(AppConfig::PIN_LIMIT_CLOSE, INPUT_PULLUP);

  pinMode(AppConfig::PIN_BUTTON_OPEN, INPUT_PULLUP);
  pinMode(AppConfig::PIN_BUTTON_CLOSE, INPUT_PULLUP);
  pinMode(AppConfig::PIN_BUTTON_AUTO, INPUT_PULLUP);

  pinMode(AppConfig::PIN_MOTOR_IN1, OUTPUT);
  pinMode(AppConfig::PIN_MOTOR_IN2, OUTPUT);
  pinMode(AppConfig::PIN_MOTOR_EN, OUTPUT);

  setMotorOutputs(MOTOR_STOPPED);
}

void setupRuntimeState() {
  runtimeState.mode = MODE_AUTO;
  runtimeState.limitOpen = readLimitPin(AppConfig::PIN_LIMIT_OPEN);
  runtimeState.limitClose = readLimitPin(AppConfig::PIN_LIMIT_CLOSE);

  runtimeState.tarpState = runtimeState.limitOpen
    ? TARP_OPEN
    : (runtimeState.limitClose ? TARP_CLOSED : TARP_UNKNOWN);

  runtimeState.motorState = MOTOR_STOPPED;
  runtimeState.wifiConnected = false;
  runtimeState.mqttConnected = false;
  runtimeState.wifiRssi = 0;
  runtimeState.sensorDirty = true;
  runtimeState.statusDirty = true;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  analogReadResolution(12);

  configurePins();

  stateMutex = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(12, sizeof(CommandMessage));

  setupRuntimeState();

  const uint64_t mac = ESP.getEfuseMac();

  snprintf(
    mqttClientId,
    sizeof(mqttClientId),
    "esp32-smart-tarp-%04X",
    static_cast<uint16_t>(mac & 0xFFFF)
  );

  startWiFiConfigPortal("Boot WiFi setup / saved WiFi connect");

  wifiClient.setInsecure();

  mqttClient.setServer(AppConfig::THINGSBOARD_HOST, AppConfig::THINGSBOARD_PORT);
  mqttClient.setBufferSize(512);
  mqttClient.setCallback(mqttCallback);

  xTaskCreatePinnedToCore(taskSensors, "taskSensors", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(taskButtons, "taskButtons", 3072, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskControl, "taskControl", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 10240, nullptr, 1, nullptr, 0);

  Serial.println("[BOOT] Smart tarp controller started");
  Serial.printf("[BOOT] MQTT Client ID: %s\n", mqttClientId);
  addSystemHistory("INFO", "BOOT", "Smart tarp controller started");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}