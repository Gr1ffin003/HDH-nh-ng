#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define led1 15
#define led2 4

const char* ssid = "Redmi 10";
const char* password = "05102003";
const char* mqtt_server = "192.168.16.26";
const int mqtt_port = 1885;
const char* mqtt_username = "qwer";
const char* mqtt_password = "123";
const char* mqtt_topics[] = {"light", "led"};
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 10000;

void setup_wifi() {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi.");
        return;
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++) {
        messageTemp += (char)message[i];
    }
    Serial.println(messageTemp);

    if (messageTemp == "all_on") {
        digitalWrite(led1, HIGH);
        digitalWrite(led2, HIGH);
        return;
    }
    if (messageTemp == "all_off") {
        digitalWrite(led1, LOW);
        digitalWrite(led2, LOW);
        return;
    }

    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, messageTemp);
    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.f_str());
        return;
    }

    if (doc.containsKey("led1")) {
        const char* led1State = doc["led1"];
        digitalWrite(led1, strcmp(led1State, "on") == 0 ? HIGH : LOW);
    }

    if (doc.containsKey("led2")) {
        const char* led2State = doc["led2"];
        digitalWrite(led2, strcmp(led2State, "on") == 0 ? HIGH : LOW);
    }
}


void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
            Serial.println("connected");
            for (int i = 0; i < 2; i++) {
                client.subscribe(mqtt_topics[i]);
                Serial.print("Subscribed to: ");
                Serial.println(mqtt_topics[i]);
            }
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));  // khởi tạo random seed từ chân analog không kết nối
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void sendSensorData() {
  StaticJsonDocument<200> doc;
  float light = random(0, 10001) / 10.0;
  String lightStr = String(light, 1) + "lux";

  doc["light"] = lightStr;
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);  // Chuyển dữ liệu thành chuỗi JSON

  // Gửi dữ liệu tới topic "sensor"
  client.publish("light", jsonBuffer);
  Serial.print("Published: ");
  Serial.println(jsonBuffer);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsgTime > interval) {
    lastMsgTime = now;
    sendSensorData();
  }
}
