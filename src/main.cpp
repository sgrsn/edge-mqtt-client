#include <Arduino.h>
#include <IntervalTimer.h>
#include "mqtt_client.hpp"
#include "RCInterface.hpp"
#include "AS5600.h"

#include "secret.h"

#define debug Serial

const uint8_t SARA_R410_PWR_ON = 2;
const uint8_t ESC_PIN = 33;
const uint8_t SERVO1_PIN = 36;
const uint8_t SERVO2_PIN = 37;

ESC     esc(ESC_PIN, 800, 2100, 20.0f);
RCServo servo1(SERVO1_PIN, 60, 90, 120);
RCServo servo2(SERVO2_PIN, 25, 75, 120);
AS5600 as5600;

MqttClient mqtt(Serial1, APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);

const float GEAR_RATIO = 7.0f;

// Heartbeatを高頻度にするとスループットが下がるので注意
const uint32_t MONITORING_RATE_US = 1000000;
const uint32_t MQTT_RATE_US = 15000;
const uint32_t CONTROL_RATE_US = 50000;
const uint32_t SENSOR_RATE_US = 1000;

// 最大4つのIntervalTimerを使える
IntervalTimer timer_monitoring;
IntervalTimer timer_mqtt_loop;
IntervalTimer timer_control;
IntervalTimer timer_sensor;

static void MonitoringTimer();
static void MqttLoop();
static void Control();
static void Sensor();

/* 緊急停止 */
bool emergency_stop = false;
static void EmergencyStop()
{
  servo1.setPosition(0);
  servo2.setPosition(0);
  esc.setSpeed(0);
  emergency_stop = true;
}

static void ResumeControl()
{
  emergency_stop = false;
}

static bool isThisHeartbeatDown()
{
  uint64_t heartbeat;
  if( !mqtt.getLastValue("robot/heartbeat", heartbeat) )
  {
    return true;
  }
  unsigned long error_time = millis() - heartbeat;
  // 4倍以上のエラーが発生していたらダウンと判断
  return error_time*1e3 > MONITORING_RATE_US * 4;
}

static bool isPCHeartbeatDown()
{
  // 前回と今回のwatchdog/heartbeat(millisとはオフセットがある)の差を計算
  // 呼び出しごとのmillis()の差を計算
  static uint64_t start_heartbeat = 0;
  static uint64_t start_millis = 0;

  // 初回呼び出し
  if (start_millis == 0 || start_heartbeat == 0)
  {
    start_millis = millis();
    mqtt.getLastValue<uint64_t>("watchdog/heartbeat", start_heartbeat);
    return true;
  }
  
  uint64_t heartbeat;
  if( !mqtt.getLastValue<uint64_t>("watchdog/heartbeat", heartbeat) )
  {
    // 取得エラー
    return true;
  }
  // ハートビート経過時間
  uint64_t heartbeat_error = heartbeat - start_heartbeat;
  // 内部経過時間
  uint64_t current_millis = millis() - start_millis;

  uint64_t error = std::max(heartbeat_error, current_millis) - std::min(heartbeat_error, current_millis);

  return error > MONITORING_RATE_US * 4;
}

static void MonitoringTimer()
{
  static bool stop = false;
  bool this_heartbeat_down = isThisHeartbeatDown();
  bool pc_heartbeat_down = isPCHeartbeatDown();

  if (this_heartbeat_down || pc_heartbeat_down)
  {
    debug.println("(MonitoringTimer) stop control timer");
    std::cout << "this_heartbeat_down: " << this_heartbeat_down << std::endl;
    std::cout << "pc_heartbeat_down: " << pc_heartbeat_down << std::endl;
    stop = true;
    EmergencyStop();
  }
  else if (stop) {
    debug.println("(MonitoringTimer) restart control timer");
    stop = false;
    ResumeControl();
  }

  // ハートビートを送信
  std::string heartbeat = std::to_string(millis());
  mqtt.publish("robot/heartbeat", heartbeat);
}

static void MqttLoop()
{
  static uint32_t last_time = 0;
  uint32_t current_time = millis();
  uint32_t dt = current_time - last_time;
  last_time = current_time;

  // std::cout << "past dt: " << dt << std::endl;


  mqtt.mqttLoop();
}

static void Control()
{
  int x = 0; int y = 0; int slider=0; bool startStop=false;
  mqtt.getLastValue<int>("control/joystick/x", x);
  mqtt.getLastValue<int>("control/joystick/y", y);
  mqtt.getLastValue<int>("control/slider", slider);
  mqtt.getLastValue<bool>("control/startStop", startStop);

  std::cout << "x: " << x << " y: " << y << " slider: " << slider << " startStop: " << startStop << std::endl;

  servo1.setPosition(x);
  servo2.setPosition(y);

  if (startStop && !emergency_stop) {
    esc.setSpeed((float)slider);
  } else {
    esc.setSpeed(0);
  }
  esc.update();
}

static void Sensor()
{
  static uint32_t last_time = 0;
  static float flapping_freq = 0.0f;

  float deg_per_sec = as5600.getAngularSpeed();
  flapping_freq = 0.5 * deg_per_sec / 360.0f / GEAR_RATIO + 0.5 * flapping_freq;

  uint32_t current_time = millis();
  uint32_t dt = current_time - last_time;
  last_time = current_time;

  uint16_t angle = as5600.readAngle();

  static uint32_t last_show = 0;
  if (current_time - last_show > 1000)
  {
    std::cout << "flapping_freq: " << flapping_freq << std::endl;
    last_show = current_time;
  }
  //std::cout << "past dt: " << dt << " " << flapping_freq << " " << std::endl;
}

void initializeTimer()
{
  timer_monitoring.begin(MonitoringTimer, MONITORING_RATE_US);
  timer_mqtt_loop.begin(MqttLoop, MQTT_RATE_US);
  timer_control.begin(Control, CONTROL_RATE_US);
  // timer_sensor.begin(Sensor, SENSOR_RATE_US);
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
  esc.update();
  servo1.setPosition(0);
  servo2.setPosition(0);

  Wire.begin();
  as5600.begin(255);
  as5600.setDirection(AS5600_CLOCK_WISE);
  
  mqtt.registerTopic<uint64_t>("robot/heartbeat");
  mqtt.registerTopic<uint64_t>("watchdog/heartbeat");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<int>("control/slider");
  mqtt.registerTopic<int>("control/joystick/x");
  mqtt.registerTopic<int>("control/joystick/y");
  mqtt.init();
  initializeTimer();
  debug.println("(setup) end");
}

void loop() 
{
}