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

const uint32_t WATCHDOG_RATE_US = 500000;
const uint32_t MONITORING_RATE_US = 1000000;
const uint32_t MQTT_RATE_US = 10000;
const uint32_t CONTROL_RATE_US = 50000;

esp_timer_handle_t timer_watchdog;
esp_timer_handle_t timer_monitoring;
esp_timer_handle_t timer_mqtt_loop;
esp_timer_handle_t timer_control;

static void HeartbeatTimer()
{
  std::string heartbeat = std::to_string(millis());
  mqtt.publish("esp/watchdog/heartbeat", heartbeat);
}

static void MonitoringTimer()
{
  static std::string heartbeat;
  static bool stop = false;
  mqtt.getLastValue("esp/watchdog/heartbeat", heartbeat);
  unsigned long last_heartbeat = 0;
  if (!heartbeat.empty())
    last_heartbeat = std::stoi(heartbeat);
  unsigned long error_time = millis() - last_heartbeat;

  // 4回以上連続で受信がない場合モーターを停止
  if (error_time > WATCHDOG_RATE_US * 4) {
    debug.println("(MonitoringTimer) ", "error_time: ", error_time);
    esp_timer_stop(timer_control);
    servo1.setPosition(0);
    servo2.setPosition(0);
    esc.stop();
    stop = true;
  }
  else if (stop) {
    debug.println("(MonitoringTimer) ", "restart control timer");
    esp_timer_start_periodic(timer_control, CONTROL_RATE_US);
    stop = false;
  }
}

static void MqttLoop()
{
  mqtt.mqttLoop();
}

static void Control()
{
  double x = 0; double y = 0; std::string heartbeat; double slider; bool startStop;

  mqtt.getLastValue("control/joystick/x", x);
  mqtt.getLastValue("control/joystick/y", y);
  mqtt.getLastValue("control/slider", slider);
  mqtt.getLastValue("control/startStop", startStop);

  servo1.setPosition(x);
  servo2.setPosition(y);

  if (startStop) {
    esc.linearAccelSpeed(slider, 1.0, 20);
  } else {
    esc.stop();
  }
}

void initializeTimer()
{
  esp_timer_create_args_t timerConfig;
  timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(HeartbeatTimer);
  timerConfig.dispatch_method = ESP_TIMER_TASK;
  timerConfig.name = "MainTimer";
  esp_timer_create(&timerConfig, &timer_watchdog);
  esp_timer_start_periodic(timer_watchdog, WATCHDOG_RATE_US);

  esp_timer_create_args_t timerConfig2;
  timerConfig2.callback = reinterpret_cast<esp_timer_cb_t>(MonitoringTimer);
  timerConfig2.dispatch_method = ESP_TIMER_TASK;
  timerConfig2.name = "MonitoringTimer";
  esp_timer_create(&timerConfig2, &timer_monitoring);
  esp_timer_start_periodic(timer_monitoring, MONITORING_RATE_US);

  esp_timer_create_args_t timerConfig3;
  timerConfig3.callback = reinterpret_cast<esp_timer_cb_t>(MqttLoop);
  timerConfig3.dispatch_method = ESP_TIMER_TASK;
  timerConfig3.name = "MqttTimer";
  esp_timer_create(&timerConfig3, &timer_mqtt_loop);
  esp_timer_start_periodic(timer_mqtt_loop, MQTT_RATE_US);
  
  esp_timer_create_args_t timerConfig4;
  timerConfig4.callback = reinterpret_cast<esp_timer_cb_t>(Control);
  timerConfig4.dispatch_method = ESP_TIMER_TASK;
  timerConfig4.name = "ControlTimer";
  esp_timer_create(&timerConfig4, &timer_control);
  esp_timer_start_periodic(timer_control, CONTROL_RATE_US);
}

void setup() {
  debug.println("(setup)", "start");
  esc.initializeTimer();
  esc.stop();
  mqtt.init();
  mqtt.registerTopic<std::string>("esp/watchdog/heartbeat");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<std::string>("control/slider");
  mqtt.registerTopic<double>("control/joystick/x");
  mqtt.registerTopic<double>("control/joystick/y");
  initializeTimer();
  debug.println("(setup)", "end");
}

void loop() {
}