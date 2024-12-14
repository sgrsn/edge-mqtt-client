#include <Arduino.h>
#include <ESP32Servo.h>
#include "mqtt_client.hpp"
#include "servo.hpp"
#include "esc.hpp"
#include "esp_timer.h"
#include "secret.h"

ServoController servo1(D0, 1, 1000, 2000, 1500);
ServoController servo2(D1, 2, 1000, 2000, 1500);
EscController esc   (D2, 3, 800, 2000);
MqttClient mqtt(APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

static void TimerCallback() {
  std::string heartbeat;
  static std::string last_heartbeat = "";
  mqtt.getLastValue("watchdog/heartbeat", heartbeat);
  debug.println("TimerCallback: ", heartbeat.c_str());
  if (last_heartbeat == heartbeat) {
    debug.println("Reconnecting...");
    //mqtt.init();
  } 
  last_heartbeat = heartbeat;
}

void initializeTimer()
{
  esp_timer_handle_t testTimer;
  esp_timer_create_args_t timerConfig;
  timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(TimerCallback);
  timerConfig.dispatch_method = ESP_TIMER_TASK;
  timerConfig.name = "MainTimer";
  esp_timer_create(&timerConfig, &testTimer);
  esp_timer_start_periodic(testTimer, 1000000);
}

void setup() {
  esc.initializeTimer();
  esc.stop();
  mqtt.init();
  mqtt.registerTopic<std::string>("watchdog/heartbeat");
  mqtt.registerTopic<std::string>("watchdog/status");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<std::string>("control/slider");
  mqtt.registerTopic<double>("control/joystick/x");
  mqtt.registerTopic<double>("control/joystick/y");
  initializeTimer();
}

void loop() {
  mqtt.mqttLoop();

  double x = 0; double y = 0; std::string heartbeat; double slider; bool startStop;

  mqtt.getLastValue("control/joystick/x", x);
  mqtt.getLastValue("control/joystick/y", y);
  mqtt.getLastValue("watchdog/heartbeat", heartbeat);
  mqtt.getLastValue("control/slider", slider);
  mqtt.getLastValue("control/startStop", startStop);

  servo1.setPosition(x);
  servo2.setPosition(y);

  if (startStop) {
    esc.linearAccelSpeed(slider, 0.1, 20);
  } else {
    esc.stop();
  }
}