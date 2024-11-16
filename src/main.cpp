#include <Arduino.h>
#include <ESP32Servo.h>
#include "mqtt_client.hpp"
#include "servo.hpp"
#include "secret.h"

ServoController servo1(D0, 1, 1000, 2000, 1500);
ServoController servo2(D1, 2, 1000, 2000, 1500);

MqttClient mqtt(APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

void setup() {
  mqtt.init();
  mqtt.registerTopic<std::string>("watchdog/heartbeat");
  mqtt.registerTopic<std::string>("watchdog/status");
  mqtt.registerTopic<std::string>("control/startStop");
  mqtt.registerTopic<std::string>("control/slider");
  mqtt.registerTopic<double>("control/joystick/x");
  mqtt.registerTopic<double>("control/joystick/y");
}

void loop() {
  mqtt.mqttLoop();

  double x = 0; double y = 0; std::string heartbeat;

  mqtt.getLastValue("control/joystick/x", x);
  mqtt.getLastValue("control/joystick/y", y);
  mqtt.getLastValue("watchdog/heartbeat", heartbeat);

  servo1.setPosition(x);
  servo2.setPosition(y);
}