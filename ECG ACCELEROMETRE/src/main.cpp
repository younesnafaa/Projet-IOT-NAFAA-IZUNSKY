#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <younesnafaa-project-1_inferencing.h>  // lib TinyML

// ===== WIFI / MQTT =====
const char *WIFI_SSID     = "iot";            
const char *WIFI_PASSWORD = "iotisis;";     
const char *MQTT_HOST     = "172.18.32.48";  // IP du Raspberry 
const uint16_t MQTT_PORT  = 1883;

const char *MQTT_CLIENT_ID    = "ESP32_MPU_TINYML";
const char *TOPIC_MOVEMENT    = "penibilite/movement/data";
const char *TOPIC_MOVEMENT_CT = "penibilite/movement/count";

// ===== MPU6050 =====
#define I2C_SDA 21
#define I2C_SCL 22
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

// Compteurs par geste (max 10 gestes différents)
struct GestureCounter {
  String name;
  int count;
};
GestureCounter gestureCounts[10];
int gestureCountSize = 0;

String lastGesture = "None";

// ===== Réseau =====
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
char         payload[256];

// ===== Protos =====
void connectWifi();
void connectMqtt();
void acquireSample();
String classifyWindow();
void publishMovement(const String &gesture);
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 MPU6050 + TinyML + MQTT ===");

  // WiFi + MQTT
  connectWifi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectMqtt();

  // Initialisation I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialisation MPU6050 avec adresse I2C explicite
  if (!mpu.begin(0x68)) {
    Serial.println("ERREUR: MPU6050 non detecte!");
    Serial.println("Verifiez les connexions I2C (SDA=21, SCL=22)");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 connecte avec succes");
  
  // Configuration du MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.print("Accelerometer range set to: +-8G\n");
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

  // Logique : on incrémente +1 à chaque acquisition d'un geste (sauf "none")
  if (!gesture.equalsIgnoreCase("none")) {
    // Trouver ou créer le compteur pour ce geste
    int idx = -1;
    for (int i = 0; i < gestureCountSize; i++) {
      if (gestureCounts[i].name.equalsIgnoreCase(gesture)) {
        idx = i;
        break;
      }
    }
    
    if (idx == -1 && gestureCountSize < 10) {
      // Nouveau geste, créer un compteur
      idx = gestureCountSize;
      gestureCounts[idx].name = gesture;
      gestureCounts[idx].count = 0;
      gestureCountSize++;
    }
    
    if (idx != -1) {
      gestureCounts[idx].count++;
      Serial.print("Compteur [" + gesture + "] => ");
      Serial.println(gestureCounts[idx].count);
    }
  }
  
  lastGesture = gesture;
  publishMovement(gesture);
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

  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

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
void publishMovement(const String &gesture) {
  // Construire le JSON avec tous les compteurs
  String jsonPayload = "{\"gesture\":\"" + gesture + "\",\"counters\":{";
  
  for (int i = 0; i < gestureCountSize; i++) {
    if (i > 0) jsonPayload += ",";
    jsonPayload += "\"" + gestureCounts[i].name + "\":" + String(gestureCounts[i].count);
  }
  jsonPayload += "}}";
  
  mqttClient.publish(TOPIC_MOVEMENT, jsonPayload.c_str());

  // Publier aussi le compteur du geste actuel
  if (!gesture.equalsIgnoreCase("none")) {
    for (int i = 0; i < gestureCountSize; i++) {
      if (gestureCounts[i].name.equalsIgnoreCase(gesture)) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", gestureCounts[i].count);
        mqttClient.publish(TOPIC_MOVEMENT_CT, buf);
        break;
      }
    }
  }

  Serial.print("MQTT -> ");
  Serial.println(jsonPayload);
}