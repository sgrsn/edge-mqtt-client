#include <Arduino.h>
#include <IntervalTimer.h>
#include "mqtt_client.hpp"
#include "RCInterface.hpp"
#include "secret.h"

#define debug Serial

const uint8_t SARA_R410_PWR_ON = 2;
const uint8_t ESC_PIN = 33;
const uint8_t SERVO1_PIN = 36;
const uint8_t SERVO2_PIN = 37;

ESC     esc(ESC_PIN, 800, 2200, 8.0f);
RCServo servo1(SERVO1_PIN, 60, 90, 120);
RCServo servo2(SERVO2_PIN, 60, 90, 120);

MqttClient mqtt(Serial1, APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

// Heartbeatを高頻度にするとスループットが下がるので注意
const uint32_t WATCHDOG_RATE_US = 1000000;
const uint32_t MONITORING_RATE_US = 100000;
const uint32_t MQTT_RATE_US = 1000;
const uint32_t CONTROL_RATE_US = 10000;

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
  if (error_time*1e3 > WATCHDOG_RATE_US * 4) {
    debug.print("(MonitoringTimer) error_time: ");
    debug.println(error_time);
    timer_control.end();
    servo1.setPosition(0);
    servo2.setPosition(0);
    esc.setSpeed(0);
    stop = true;
  }
  else if (stop) {
    debug.println("(MonitoringTimer) restart control timer");
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
  double x = 0; double y = 0; double slider=0; bool startStop=false;

  mqtt.getLastValue("control/joystick/x", x);
  mqtt.getLastValue("control/joystick/y", y);
  mqtt.getLastValue("control/slider", slider);
  mqtt.getLastValue("control/startStop", startStop);

  servo1.setPosition(x);
  servo2.setPosition(y);

  if (startStop) {
    esc.setSpeed(slider);
  } else {
    esc.setSpeed(0);
  }
  esc.update();
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
  pinMode(SARA_R410_PWR_ON, INPUT);
  esc.begin();
  servo1.begin();
  servo2.begin();
  esc.setSpeed(0);
  servo1.setPosition(0);
  servo2.setPosition(0);

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