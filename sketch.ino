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

const float ACC_THRESHOLD = 18.0;
const int GPS_PERIOD_MS = 1000;
const int ACC_PRINT_MS = 300;

double lat = -6.792400;
double lon = 39.208300;

unsigned long lastGpsMs = 0;
unsigned long lastReconnectMs = 0;
unsigned long lastAccPrintMs = 0;

bool accidentLatched = false;
unsigned long accidentLatchMs = 0;

// ----------- Fixed Accident Points -----------
struct Point {
  double lat;
  double lon;
};

Point accidentPoints[] = {
  {-6.82354469038552, 39.28146069337019},
  {-6.821362918736765, 39.281295543719466},
  {-6.81598127193375, 39.28003309433296},
  {-6.810852848155616, 39.26699648419273},
  {-6.798020126855109, 39.22982267917766},
  {-6.8423013484097845, 39.245180570065315},
  {-6.834318619600618, 39.32230296448238},
  {-6.660014272182926, 39.18238932884291}
};

Point selectedAccidentPoint;

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
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
}

bool connectMQTT() {
  String clientId = String("wokwi-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  return mqtt.connect(clientId.c_str());
}

bool publishJson(const String& topic, const String& payload) {
  return mqtt.publish(topic.c_str(), payload.c_str());
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

  randomSeed((uint32_t)esp_random());

  // Select one accident point per restart
  int index = random(0, sizeof(accidentPoints) / sizeof(accidentPoints[0]));
  selectedAccidentPoint = accidentPoints[index];

  Serial.println("BOOT OK");
  Serial.print("Selected accident location: ");
  Serial.print(selectedAccidentPoint.lat, 6);
  Serial.print(", ");
  Serial.println(selectedAccidentPoint.lon, 6);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);
  mpu.begin();
  mpu.calcGyroOffsets(true);

  connectWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  connectMQTT();
}

void loop() {
  if (!mqtt.connected()) {
    connectMQTT();
  } else {
    mqtt.loop();
  }

  unsigned long now = millis();

  if (mqtt.connected() && (now - lastGpsMs > GPS_PERIOD_MS)) {
    lastGpsMs = now;

    lat += 0.000010;
    lon += 0.000008;

    String payload =
      String("{\"device\":\"") + DEVICE_ID + "\"," +
      "\"lat\":" + String(lat, 6) + "," +
      "\"lon\":" + String(lon, 6) + "}";

    publishJson(TOPIC_GPS, payload);
  }

  float accNow = totalAccMS2();
  if (!accidentLatched && mqtt.connected() && accNow > ACC_THRESHOLD) {
    accidentLatched = true;
    accidentLatchMs = now;

    beepPatternAccident();

    String payload =
      String("{\"device\":\"") + DEVICE_ID + "\"," +
      "\"type\":\"ACCIDENT\"," +
      "\"lat\":" + String(selectedAccidentPoint.lat, 6) + "," +
      "\"lon\":" + String(selectedAccidentPoint.lon, 6) + "}";

    publishJson(TOPIC_EVENT, payload);

    Serial.println("ACCIDENT DETECTED");
  }

  if (accidentLatched && (now - accidentLatchMs > 10000)) {
    accidentLatched = false;
  }

  delay(60);
}
