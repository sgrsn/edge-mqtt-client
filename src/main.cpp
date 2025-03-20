#include <Arduino.h>
#include "mqtt_client.hpp"
#include "esp_timer.h"
#include "secret.h"
#include "i2cslave.hpp"

MqttClient mqtt(APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

const uint32_t WATCHDOG_RATE_US = 10000 * 1e3;
const uint32_t MONITORING_RATE_US = 10000 * 1e3;
const uint32_t MQTT_SUBSCRIBE_RATE_US = 100 * 1e3;
const uint32_t MQTT_RATE_US = 10 * 1e3;

esp_timer_handle_t timer_watchdog;
esp_timer_handle_t timer_monitoring;
esp_timer_handle_t timer_mqtt_subscribe;
esp_timer_handle_t timer_mqtt_loop;

I2CSlave i2cSlave;

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

  debug.println("MonitoringTimer", error_time);
}

static void MqttSubscribe()
{
  double x, y;
  mqtt.getLastValue("control/joystick/x", x);
  mqtt.getLastValue("control/joystick/y", y);
  i2cSlave.setRegister(0, int(x));
  i2cSlave.setRegister(1, int(y));
  debug.println("X: ",  int(x), " Y: ", int(y));
}

static void MqttLoop()
{
  mqtt.mqttLoop();
}

void initializeTimer()
{
  esp_timer_create_args_t timerConfig;
  timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(HeartbeatTimer);
  timerConfig.dispatch_method = ESP_TIMER_TASK;
  timerConfig.name = "MainTimer";
  esp_timer_create(&timerConfig, &timer_watchdog);
  esp_timer_start_periodic(timer_watchdog, WATCHDOG_RATE_US);

  esp_timer_create_args_t timerConfig1;
  timerConfig1.callback = reinterpret_cast<esp_timer_cb_t>(MqttSubscribe);
  timerConfig1.dispatch_method = ESP_TIMER_TASK;
  timerConfig1.name = "MqttSubscribeTimer";
  esp_timer_create(&timerConfig1, &timer_mqtt_subscribe);
  esp_timer_start_periodic(timer_mqtt_subscribe, MQTT_SUBSCRIBE_RATE_US);

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
}

void setup() {
  debug.println("(setup)", "start");
  mqtt.init();
  mqtt.registerTopic<std::string>("esp/watchdog/heartbeat");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<std::string>("control/slider");
  mqtt.registerTopic<double>("control/joystick/x");
  mqtt.registerTopic<double>("control/joystick/y");
  initializeTimer();
  debug.println("(setup)", "end");

  i2cSlave.init(0x08);
}

void loop() 
{
}