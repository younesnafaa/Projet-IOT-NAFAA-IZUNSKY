#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <younesnafaa-project-1_inferencing.h>  // lib TinyML

// ===== WIFI / MQTT =====
const char *WIFI_SSID     = "pi";            // A ADAPTER
const char *WIFI_PASSWORD = "raspberry";     // A ADAPTER
const char *MQTT_HOST     = "172.18.32.48";  // IP du Raspberry A ADAPTER
const uint16_t MQTT_PORT  = 1883;

const char *MQTT_CLIENT_ID    = "ESP32_MPU_TINYML";
const char *TOPIC_MOVEMENT    = "penibilite/movement/data";
const char *TOPIC_MOVEMENT_CT = "penibilite/movement/count";

// ===== MPU6050 =====
Adafruit_MPU6050 mpu;

// Buffer d'entrée du modèle Edge Impulse
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// On prend une fenêtre telle que 3 * WINDOW_SIZE <= taille attendue
const int WINDOW_SIZE = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 3;

float windowAx[WINDOW_SIZE];
float windowAy[WINDOW_SIZE];
float windowAz[WINDOW_SIZE];
int   windowIndex = 0;
bool  windowFull  = false;

// ===== Compteur de répétitions =====
unsigned long lastClassifTime        = 0;
const unsigned long CLASSIF_PERIOD_MS = 500;

int  repCount = 0;
bool inRep    = false;

// ===== Réseau =====
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
char         payload[256];

// ===== Protos =====
void connectWifi();
void connectMqtt();
void acquireSample();
String classifyWindow();
void publishMovement(const String &gesture, int reps);
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 MPU6050 + TinyML + MQTT ===");

  // MPU6050
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("ERREUR: MPU6050 non detecte !");
    while (true) {
      delay(500);
      Serial.println("MPU6050 absent...");
    }
  }
  Serial.println("MPU6050 initialise.");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // WiFi + MQTT
  connectWifi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectMqtt();

  Serial.print("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE = ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.print("WINDOW_SIZE = ");
  Serial.println(WINDOW_SIZE);
}

// ================== WIFI / MQTT ==================
void connectWifi() {
  Serial.print("Connexion WiFi a ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connecte, IP = ");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Connexion au broker MQTT... ");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("OK");
    } else {
      Serial.print("ECHEC, code = ");
      Serial.print(mqttClient.state());
      Serial.println(" ; nouvel essai dans 3s");
      delay(3000);
    }
  }
}

// ================== LOOP ==================
void loop() {
  if (!mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();

  acquireSample();

  if (!windowFull) {
    return;
  }

  unsigned long now = millis();
  if (now - lastClassifTime < CLASSIF_PERIOD_MS) {
    return;
  }
  lastClassifTime = now;

  String gesture = classifyWindow();
  Serial.print("Geste detecte : ");
  Serial.println(gesture);

  // logique reps : on compte une rep quand on sort d'un geste vers "none"
  if (!gesture.equalsIgnoreCase("none")) {
    inRep = true;
  } else {
    if (inRep) {
      repCount++;
      inRep = false;
      Serial.print("Repetition ++ => ");
      Serial.println(repCount);
    }
  }

  publishMovement(gesture, repCount);
}

// ================== ACQUISITION MPU ==================
void acquireSample() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  windowAx[windowIndex] = a.acceleration.x;
  windowAy[windowIndex] = a.acceleration.y;
  windowAz[windowIndex] = a.acceleration.z;

  windowIndex++;
  if (windowIndex >= WINDOW_SIZE) {
    windowIndex = 0;
    windowFull  = true;
  }
}

// ================== TINYML / EDGE IMPULSE =============
String classifyWindow() {
  // Remplir features[] : [ax0, ay0, az0, ax1, ay1, az1, ...]
  size_t idx = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (idx + 3 > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) break;
    features[idx++] = windowAx[i];
    features[idx++] = windowAy[i];
    features[idx++] = windowAz[i];
  }
  while (idx < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    features[idx++] = 0.0f;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data     = &raw_feature_get_data;

  ei_impulse_result_t result = { 0 };

  // Essaie d'abord cette version :
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  // Si compilation KO, commente la ligne au-dessus et décommente celle-ci :
  // EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false, false);

  if (res != EI_IMPULSE_OK) {
    Serial.print("run_classifier ERROR: ");
    Serial.println(res);
    return "None";
  }

  float  best_score = 0.0f;
  String best_label = "None";

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    float v = result.classification[ix].value;
    if (v > best_score) {
      best_score = v;
      best_label = String(result.classification[ix].label);
    }
  }

  if (best_score < 0.6f) {
    return "None";
  }
  return best_label;
}

static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

// ================== MQTT PUBLISH ==================
void publishMovement(const String &gesture, int reps) {
  snprintf(payload, sizeof(payload),
           "{\"gesture\":\"%s\",\"reps\":%d}",
           gesture.c_str(), reps);

  mqttClient.publish(TOPIC_MOVEMENT, payload);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d", reps);
  mqttClient.publish(TOPIC_MOVEMENT_CT, buf);

  Serial.print("MQTT -> ");
  Serial.println(payload);
}
