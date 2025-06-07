#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>

const char* ssid = "Wokwi-GUEST"; 
const char* password = ""; 

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "esp32-climatech-simple-pub-11223";
const char* topic_env_data = "sos_climatech_simple/env/data";
const char* topic_alert = "sos_climatech_simple/alert/trigger";

#define DHTPIN 4
#define BUTTONPIN 15
#define LEDPIN 2

#define DHTTYPE DHT22

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastEnvMsg = 0;
const long envInterval = 10000;

int lastButtonState = HIGH;
bool ledState = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conectar ao MQTT Broker...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

void sendAlert() {
  StaticJsonDocument<50> doc;
  doc["alert"] = "triggered";

  char buffer[50];
  size_t n = serializeJson(doc, buffer);

  Serial.println("Enviando Alerta MQTT...");
  client.publish(topic_alert, buffer, n);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  dht.begin();

  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int reading = digitalRead(BUTTONPIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("Botão Pressionado!");
      sendAlert();
      ledState = !ledState;
      digitalWrite(LEDPIN, ledState ? HIGH : LOW);
    }
  }
  lastButtonState = reading;

  unsigned long now = millis();
  if (now - lastEnvMsg > envInterval) {
    lastEnvMsg = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Falha ao ler do sensor DHT!"));
    } else {
      StaticJsonDocument<100> doc;
      doc["temperature"] = t;
      doc["humidity"] = h;

      char buffer[100];
      size_t n = serializeJson(doc, buffer);

      Serial.print("Publicando dados ambientais MQTT: ");
      Serial.println(buffer);
      client.publish(topic_env_data, buffer, n);
    }
  }
}
