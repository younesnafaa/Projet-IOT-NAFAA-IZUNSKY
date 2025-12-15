#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ====================== CONFIG WIFI / MQTT ======================
const char *WIFI_SSID     = "iot";   
const char *WIFI_PASSWORD = "iotisis;";     
const char *MQTT_HOST     = "172.18.32.48"; // IP du Raspberry (broker MQTT)
const uint16_t MQTT_PORT  = 1883;

// Topics MQTT 
const char *TOPIC_ECG_DATA = "penibilite/ecg/data";
const char *TOPIC_ECG_CMD  = "penibilite/ecg/cmd";

// ====================== CONFIG CAPTEUR ECG ======================
const int PIN_ECG_SIGNAL = 15;  // AD8232
const int PIN_ECG_LOP    = 10;  // LO+ 
const int PIN_ECG_LOM    = 9;  // LO-
const int PIN_STATUS_LED = 2;   // LED

// ====================== OBJETS RESEAU ===========================
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// Buffer pour construire les messages JSON
char payload[128];

// ====================== PROTOTYPES ==============================
void connectWifi();
void connectMqtt();
void mqttCallback(char *topic, byte *message, unsigned int length);

// ====================== SETUP ===================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== ESP32 ECG - Projet Penibilite ===");

  pinMode(PIN_ECG_SIGNAL, INPUT);
  pinMode(PIN_ECG_LOP,    INPUT);
  pinMode(PIN_ECG_LOM,    INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

#ifdef ARDUINO_ARCH_ESP32
  analogReadResolution(12);          // 0..4095
  analogSetAttenuation(ADC_11db);    // ~0..3.3V
  analogSetPinAttenuation(PIN_ECG_SIGNAL, ADC_11db);
#endif

  connectWifi();

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  connectMqtt();

  Serial.println("ECG prêt, envoi des données vers MQTT...");
}

// ====================== WIFI / MQTT =============================
void connectWifi() {
  Serial.print("Connexion au WiFi : ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connecté");
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Connexion au broker MQTT... ");
    if (mqttClient.connect("ESP32_ECG_PENIBILITE")) { // ID client MQTT
      Serial.println("OK");
      mqttClient.subscribe(TOPIC_ECG_CMD);
    } else {
      Serial.print("échec, code = ");
      Serial.print(mqttClient.state());
      Serial.println(" ; nouvelle tentative dans 5s");
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message MQTT sur ");
  Serial.print(topic);
  Serial.print(" : ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
  Serial.println();
  }

// ====================== LOOP ====================================
void loop() {
  if (!mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();

  static unsigned long lastPublish = 0;
  const unsigned long PUBLISH_PERIOD_MS = 1000;  // 1 message / seconde

  unsigned long now = millis();
  if (now - lastPublish < PUBLISH_PERIOD_MS) {
    return;
  }
  lastPublish = now;

  bool lo_plus  = (digitalRead(PIN_ECG_LOP) == HIGH);
  bool lo_minus = (digitalRead(PIN_ECG_LOM) == HIGH);
  bool electrodes_ok = !(lo_plus || lo_minus);

  if (!electrodes_ok) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.println("ECG : electrodes deconnectees / mauvais contact");

    snprintf(payload, sizeof(payload),
             "{\"ecg\":0,\"status\":\"electrodes_off\"}");
  } else {
    digitalWrite(PIN_STATUS_LED, LOW);

    int raw = analogRead(PIN_ECG_SIGNAL);
    Serial.print("ECG brut = ");
    Serial.println(raw);

    snprintf(payload, sizeof(payload),
             "{\"ecg\":%d,\"status\":\"ok\"}", raw);
  }

  mqttClient.publish(TOPIC_ECG_DATA, payload);
}
