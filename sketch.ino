#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

const char* MQTT_HOST = "test.mosquitto.org";
const int   MQTT_PORT = 1883;

const char* DEVICE_ID = "esp32-01";
String TOPIC_GPS   = String("accident/") + DEVICE_ID + "/gps";
String TOPIC_EVENT = String("accident/") + DEVICE_ID + "/event";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

MPU6050 mpu(Wire);

const int BUZZER_PIN = 18;

const float ACC_THRESHOLD = 18.0;     // m/s^2
const int GPS_PERIOD_MS = 1000;
const int ACC_PRINT_MS = 300;

double lat = -6.792400;
double lon = 39.208300;

unsigned long lastGpsMs = 0;
unsigned long lastReconnectMs = 0;
unsigned long lastAccPrintMs = 0;

bool accidentLatched = false;
unsigned long accidentLatchMs = 0;

void beep(int ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepPatternAccident() {
  for (int i = 0; i < 3; i++) {
    beep(120);
    delay(80);
  }
}

void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - start > 15000) {
      Serial.println("\n[WiFi] Timeout, retrying...");
      start = millis();
    }
  }

  Serial.println();
  Serial.print("[WiFi] Connected, IP: ");
  Serial.println(WiFi.localIP());
}

bool connectMQTT() {
  String clientId = String("wokwi-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.print("[MQTT] Connecting to ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  bool ok = mqtt.connect(clientId.c_str());
  if (ok) {
    Serial.println("[MQTT] Connected");
  } else {
    Serial.print("[MQTT] Failed, rc=");
    Serial.println(mqtt.state());
  }
  return ok;
}

bool publishJson(const String& topic, const String& payload) {
  bool ok = mqtt.publish(topic.c_str(), payload.c_str());
  Serial.print("[PUB] ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FAIL");
  if (!ok) {
    Serial.println(payload);
  }
  return ok;
}

float totalAccMS2() {
  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  return sqrt(ax * ax + ay * ay + az * az) * 9.80665;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("BOOT: serial OK");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22); // SDA=D21, SCL=D22 (your diagram)
  mpu.begin();
  mpu.calcGyroOffsets(true);

  Serial.println("=== Accident MQTT Simulation Boot ===");
  Serial.print("GPS topic: "); Serial.println(TOPIC_GPS);
  Serial.print("EVT topic: "); Serial.println(TOPIC_EVENT);
  Serial.print("ACC threshold (m/s^2): "); Serial.println(ACC_THRESHOLD, 2);

  connectWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  connectMQTT();

  Serial.println("[READY] Open Serial Monitor to see logs.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection, reconnecting...");
    connectWiFi();
  }

  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectMs > 1500) {
      lastReconnectMs = now;
      connectMQTT();
    }
  } else {
    mqtt.loop();
  }

  unsigned long now = millis();

  if (now - lastAccPrintMs > (unsigned long)ACC_PRINT_MS) {
    lastAccPrintMs = now;
    float acc = totalAccMS2();
    Serial.print("[ACC] total m/s^2 = ");
    Serial.println(acc, 2);
  }

  if (mqtt.connected() && (now - lastGpsMs > (unsigned long)GPS_PERIOD_MS)) {
    lastGpsMs = now;

    lat += 0.000010;
    lon += 0.000008;

    String payload =
      String("{\"device\":\"") + DEVICE_ID + "\"," +
      "\"lat\":" + String(lat, 6) + "," +
      "\"lon\":" + String(lon, 6) + "," +
      "\"ts\":" + String((uint32_t)(now / 1000)) + "}";

    Serial.print("[GPS] ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.println(lon, 6);

    publishJson(TOPIC_GPS, payload);
  }

  float accNow = totalAccMS2();
  if (!accidentLatched && mqtt.connected() && accNow > ACC_THRESHOLD) {
    accidentLatched = true;
    accidentLatchMs = now;

    Serial.println("!!! ACCIDENT DETECTED !!!");
    Serial.print("acc m/s^2: "); Serial.println(accNow, 2);
    Serial.print("at: "); Serial.print(lat, 6); Serial.print(", "); Serial.println(lon, 6);

    beepPatternAccident();

    String payload =
      String("{\"device\":\"") + DEVICE_ID + "\"," +
      "\"type\":\"ACCIDENT\"," +
      "\"acc_ms2\":" + String(accNow, 2) + "," +
      "\"lat\":" + String(lat, 6) + "," +
      "\"lon\":" + String(lon, 6) + "," +
      "\"ts\":" + String((uint32_t)(now / 1000)) + "}";

    publishJson(TOPIC_EVENT, payload);
  }

  if (accidentLatched && (now - accidentLatchMs > 10000)) {
    accidentLatched = false;
    Serial.println("[ACCIDENT] latch cleared");
  }

  delay(60);
}
