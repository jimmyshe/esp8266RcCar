#include "TickerScheduler.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <WiFiManager.h>
#include <ctime>
#include <sstream>
#include <string>

//输入输出针脚
constexpr uint8_t FORWARD_SPEED_CHANNEL = D6;
constexpr uint8_t BACKWARD_SPEED_CHANNEL = D5;
constexpr uint8_t MOTOR_ENABLE_CHANNEL = LED_BUILTIN;
constexpr uint8_t STEER_CHANNEL = D7;
constexpr double PwmRange = 2048;
ADC_MODE(ADC_VCC);

void initializePin();

// global variables
std::unique_ptr<TickerScheduler> ts;
std::unique_ptr<WiFiManager> wifiManager;
std::unique_ptr<WiFiClient> wifiClient;
std::unique_ptr<PubSubClient> mqttClient;
std::unique_ptr<Servo> steeringServo;
unsigned long controlMsgUpdateTime = 0;

void heartbeat(void *argc) {
  static unsigned int beat{0};
  if (mqttClient->connected()) {
    DynamicJsonDocument doc(200);
    doc["beat"] = beat;
    doc["vcc"] = EspClass::getVcc();
    doc["uptime"] = millis();
    std::string ss;
    serializeJson(doc, ss);
    mqttClient->publish("esp8266", ss.c_str());
    ++beat;
  }
}

void WatchDog(void *argc) {
  // controlMsgUpdateTime
  auto diff = millis() - controlMsgUpdateTime;
  if (diff > 200) {
    digitalWrite(MOTOR_ENABLE_CHANNEL, LOW);
  }
}

void MsgCallback(char *topic, byte *payload, unsigned int length) {
  //    auto s = std::string((char *)payload, length);
  //    Serial.printf("Message arrived [%s] \n \t %s", topic, s.c_str());
  //    Serial.println();
  static double oldThrottle = 0;
  static DynamicJsonDocument doc(1000);
  auto error = deserializeJson(doc, payload);
  if (error) {
    Serial.printf("Error deserializing %s", error.c_str());
    Serial.println("msg invalid");

    return;
  }

  if (doc.containsKey("throttle") and doc.containsKey("angle") and
      doc.containsKey("brake") and doc.containsKey("reverse")) {

    bool reverse = doc["reverse"].as<bool>();
    double throttle = doc["throttle"].as<double>();
    double brake = doc["brake"].as<double>();
    double steeringAngle = doc["angle"].as<double>();

    Serial.printf("%f %f %f", throttle, brake, steeringAngle);
    Serial.println("");

    steeringServo->write(steeringAngle);
    digitalWrite(MOTOR_ENABLE_CHANNEL, HIGH);

    if (brake > 0) {
      analogWrite(FORWARD_SPEED_CHANNEL, 0);
      analogWrite(BACKWARD_SPEED_CHANNEL, 0);
      analogWrite(MOTOR_ENABLE_CHANNEL, brake * PwmRange);
    } else {
      if (oldThrottle > throttle) {
        analogWrite(MOTOR_ENABLE_CHANNEL, 0);
      } else {
        analogWrite(MOTOR_ENABLE_CHANNEL, PwmRange - 1);

        int throttlePWM = throttle * PwmRange;
        throttlePWM = throttlePWM / 2;

        if (not reverse) {
          analogWrite(FORWARD_SPEED_CHANNEL, throttlePWM);
          analogWrite(BACKWARD_SPEED_CHANNEL, 0);
        } else {
          throttlePWM = throttlePWM / 2;
          analogWrite(FORWARD_SPEED_CHANNEL, 0);
          analogWrite(BACKWARD_SPEED_CHANNEL, throttlePWM);
        }
      }
    }
    oldThrottle = throttle;
    controlMsgUpdateTime = millis();
  } else {
    Serial.println("msg doest not have required keys");
  }
}

void reconnect() {
  static time_t lastConnectedTimeStamp = 0;
  constexpr int waitForConnectionTime = 5;
  auto now = std::time(nullptr);
  auto diff = now - lastConnectedTimeStamp;
  // Loop until we're reconnected
  if (!mqttClient->connected() and diff > waitForConnectionTime) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(EspClass::random(), HEX);
    // Attempt to connect
    if (mqttClient->connect(clientId.c_str(), "cisai", "cisai123456")) {
      Serial.println("connected");
      // ... and resubscribe
      mqttClient->subscribe("RcControl");
      lastConnectedTimeStamp = std::time(nullptr);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient->state());
      Serial.printf(" try again in %d seconds", waitForConnectionTime);
      Serial.println();
      lastConnectedTimeStamp = std::time(nullptr);
    }
  }
}

void setup(void) {
  initializePin();

  analogWriteFreq(80);
  analogWriteRange(PwmRange);

  Serial.begin(115200);

  ts = std::make_unique<TickerScheduler>(2);
  wifiManager = std::make_unique<WiFiManager>();
  wifiClient = std::make_unique<WiFiClient>();
  mqttClient = std::make_unique<PubSubClient>(*wifiClient);
  mqttClient->setServer(IPAddress(192, 168, 0, 2), 1883);
  mqttClient->setCallback(MsgCallback);

  auto res = wifiManager->autoConnect("my8266", "67871218");
  if (!res) {
    Serial.println("Failed to connect");
    EspClass::restart();
  } else {
    // if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  }

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  ts->add(0, 2000, heartbeat, nullptr, true);
  ts->add(1, 200, WatchDog, nullptr, true);

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  reconnect();
}
void initializePin() {
  steeringServo = std::make_unique<Servo>();
  steeringServo->attach(STEER_CHANNEL);
  pinMode(FORWARD_SPEED_CHANNEL, OUTPUT);
  pinMode(BACKWARD_SPEED_CHANNEL, OUTPUT);
  pinMode(MOTOR_ENABLE_CHANNEL, OUTPUT);
}

void loop(void) {
  ts->update();
  if (!mqttClient->connected()) {
    reconnect();
  }
  mqttClient->loop();
}
