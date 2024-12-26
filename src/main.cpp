#include <Arduino.h>
#include <IntervalTimer.h>
#include "mqtt_client.hpp"
#include "secret.h"

#define debug Serial

// ServoController servo1(D0, 1, 1000, 2000, 1500);
// ServoController servo2(D1, 2, 1000, 2000, 1500);
// EscController esc   (D2, 3, 800, 2000);

MqttClient mqtt(Serial1, APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

const uint32_t WATCHDOG_RATE_US = 500000;
const uint32_t MONITORING_RATE_US = 1000000;
const uint32_t MQTT_RATE_US = 10000;
const uint32_t CONTROL_RATE_US = 50000;

IntervalTimer timer_watchdog;
IntervalTimer timer_monitoring;
IntervalTimer timer_mqtt_loop;
IntervalTimer timer_control;

static void HeartbeatTimer();
static void MonitoringTimer();
static void MqttLoop();
static void Control();

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
    // debug.println("(MonitoringTimer) ", "error_time: ", error_time);
    //esp_timer_stop(timer_control);
    timer_control.end();
    //servo1.setPosition(0);
    //servo2.setPosition(0);
    //esc.stop();
    stop = true;
  }
  else if (stop) {
    // debug.println("(MonitoringTimer) ", "restart control timer");
    //esp_timer_start_periodic(timer_control, CONTROL_RATE_US);
    timer_control.begin(Control, CONTROL_RATE_US);

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

  //servo1.setPosition(x);
  //servo2.setPosition(y);

  if (startStop) {
    //esc.linearAccelSpeed(slider, 1.0, 20);
  } else {
    //esc.stop();
  }
}

void initializeTimer()
{
  timer_watchdog.begin(HeartbeatTimer, WATCHDOG_RATE_US);
  timer_monitoring.begin(MonitoringTimer, MONITORING_RATE_US);
  timer_mqtt_loop.begin(MqttLoop, MQTT_RATE_US);
  timer_control.begin(Control, CONTROL_RATE_US);
}

void setup() {
  Serial1.begin(115200);
  debug.begin(115200);
  debug.println("(setup) start");
  mqtt.init();
  mqtt.registerTopic<std::string>("esp/watchdog/heartbeat");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<std::string>("control/slider");
  mqtt.registerTopic<double>("control/joystick/x");
  mqtt.registerTopic<double>("control/joystick/y");
  initializeTimer();
  debug.println("(setup) end");
}

void loop() {
}