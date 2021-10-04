#include "TickerScheduler.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Servo.h>
#include <WiFiManager.h>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "ESPAsyncUDP.h"

AsyncUDP udp;

//输入输出针脚
constexpr uint8_t FORWARD_SPEED_CHANNEL = D6;
constexpr uint8_t BACKWARD_SPEED_CHANNEL = D5;
constexpr uint8_t MOTOR_ENABLE_CHANNEL = LED_BUILTIN;
constexpr uint8_t STEER_CHANNEL = D7;
constexpr double PwmRange = 2048;
ADC_MODE(ADC_VCC);

void initializePin();

// global variables
TickerScheduler ts(2);
std::unique_ptr<WiFiManager> wifiManager;
Servo steeringServo;
unsigned long controlMsgUpdateTime = 0;

// void heartbeat(void *argc) {
//   static unsigned int beat{0};
//   if (mqttClient->connected()) {
//     DynamicJsonDocument doc(200);
//     doc["beat"] = beat;
//     doc["vcc"] = EspClass::getVcc();
//     doc["uptime"] = millis();
//     std::string ss;
//     serializeJson(doc, ss);
//     mqttClient->publish("esp8266", ss.c_str());
//     ++beat;
//   }
// }

void WatchDog(void *argc) {
  // controlMsgUpdateTime
  auto diff = millis() - controlMsgUpdateTime;
  if (diff > 200) {
    digitalWrite(MOTOR_ENABLE_CHANNEL, LOW);
  }
}

void MsgCallback(AsyncUDPPacket packet) {

//  Serial.print("UDP Packet Type: ");
//  Serial.print(packet.isBroadcast()   ? "Broadcast"
//               : packet.isMulticast() ? "Multicast"
//                                      : "Unicast");
//  Serial.print(", From: ");
//  Serial.print(packet.remoteIP());
//  Serial.print(":");
//  Serial.print(packet.remotePort());
//  Serial.print(", To: ");
//  Serial.print(packet.localIP());
//  Serial.print(":");
//  Serial.print(packet.localPort());
//  Serial.print(", Length: ");
//  Serial.print(packet.length());
//  Serial.print(", Data: ");
//  Serial.write(packet.data(), packet.length());
//  Serial.println();
//  // reply to the client
//  packet.printf("Got %u bytes of data", packet.length());

  static double oldThrottle = 0;
  static DynamicJsonDocument doc(1000);
  auto error = deserializeJson(doc, packet.data());
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

    steeringServo.write(steeringAngle);
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
//        throttlePWM = throttlePWM / 2;

        if (not reverse) {
          analogWrite(FORWARD_SPEED_CHANNEL, throttlePWM);
          analogWrite(BACKWARD_SPEED_CHANNEL, 0);
        } else {
          throttlePWM = throttlePWM / 4;
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

void setup(void) {
  initializePin();

  analogWriteFreq(80);
  analogWriteRange(PwmRange);

  Serial.begin(115200);

  wifiManager = std::make_unique<WiFiManager>();

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

  ts.add(1, 200, WatchDog, nullptr, true);

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (udp.listen(8080)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](const AsyncUDPPacket &packet) { MsgCallback(packet); });
  }
}
void initializePin() {
  Serial.println("initialize Pin");
  steeringServo.attach(STEER_CHANNEL);
  pinMode(FORWARD_SPEED_CHANNEL, OUTPUT);
  pinMode(BACKWARD_SPEED_CHANNEL, OUTPUT);
  pinMode(MOTOR_ENABLE_CHANNEL, OUTPUT);
}

void loop(void) { ts.update(); }
