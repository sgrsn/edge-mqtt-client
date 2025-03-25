#include <Arduino.h>
#include "secret.h"
#include "i2cslave.hpp"
#include "i2c_register.h"
#include <PrintServer.h>
DumpServer debug;
#include "mqtt_client.hpp"

#include <arduino-timer.h>

HardwareSerial serial_(0);
MqttClient mqtt(serial_, APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);
I2CSlave i2cSlave;
int teensy41_status = 0;

auto watchdog_timer = timer_create_default();
auto mqtt_timer =     timer_create_default();
auto notify_timer =   timer_create_default();

// 頻度
const uint16_t WATCHDOG_INTERVAL = 800;
const uint16_t WATCHDOG_TIMEOUT = 2000;
const uint16_t MQTT_INTERVAL = 10;
const uint16_t GNSS_NOTIFY_INTERVAL = 5000;

EdgeStatus edgeStatus;

bool monitoring(void *)
{
  uint64_t heartbeat;
  mqtt.getLastValue<uint64_t>("robot/heartbeat", heartbeat);
  unsigned long error_time = millis() - heartbeat;
  debug.println("MonitoringTimer", error_time);
  mqtt.publish("robot/heartbeat", std::to_string(millis()));

  bool status = error_time < WATCHDOG_TIMEOUT;
  /*
  0: Error
  1: OK
  */

  i2cSlave.setRegister(MODEM_HEARTBEAT_STATUS_REG, status);

  // teensyのステータスを定期的にリセット
  edgeStatus = 0;

  static uint32_t last = 0;
  debug.println("monitoring past: ", millis() - last);
  last = millis();

  return true;
}

bool mqttLoop(void *) 
{
  mqtt.mqttLoop();

  int x = 0;
  int y = 0;
  int slider = 0;
  bool startStop = false;
  bool logging = false;

  mqtt.getLastValue<int>("control/joystick/x", x);
  mqtt.getLastValue<int>("control/joystick/y", y);
  mqtt.getLastValue<int>("control/slider", slider);
  mqtt.getLastValue<bool>("control/startStop", startStop);
  mqtt.getLastValue<bool>("logging/startStop", logging);

  i2cSlave.setRegister(MODEM_JOYSTICK_X_REG,  int(x));
  i2cSlave.setRegister(MODEM_JOYSTICK_Y_REG,  int(y));
  i2cSlave.setRegister(MODEM_SLIDER_REG,      int(slider));
  i2cSlave.setRegister(MODEM_START_STOP_REG,  startStop);
  i2cSlave.setRegister(LOGGING_STATUS_REG,    logging);

  edgeStatus = i2cSlave.getRegister(TEENSY41_STATUS_REG);

  return true;
}


void setup() {
  serial_.begin(115200, SERIAL_8N1, RX, TX);
  WiFi.softAP("MyESP32", "12345678");
  debug.begin();
  debug.println("(setup)", "start");
  bool modem_status = false;
  while (!modem_status)
  {
    modem_status = mqtt.init();
    i2cSlave.setRegister(MODEM_STATUS_REG,    modem_status);
    delay(1000);
  }
  mqtt.registerTopic<uint64_t>("robot/heartbeat");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<int>("control/slider");
  mqtt.registerTopic<int>("control/joystick/x");
  mqtt.registerTopic<int>("control/joystick/y");
  mqtt.registerTopic<bool>("logging/startStop");
  mqtt.registerTopic<int>("teensy4.1/status");

  i2cSlave.setRegister(MODEM_STATUS_REG,    modem_status);
  i2cSlave.setRegister(ESP32_IS_READY_REG,  1);
  i2cSlave.init(ESP32_I2C_ADDR);

  watchdog_timer.every(WATCHDOG_INTERVAL, monitoring);
  mqtt_timer.every(MQTT_INTERVAL, mqttLoop);
  notify_timer.every(GNSS_NOTIFY_INTERVAL, [](void *) {
    mqtt.publish("teensy4.1/status", std::to_string(edgeStatus));
    return true;
  });

  debug.println("(setup)", "end");
}


void loop()
{
  watchdog_timer.tick();
  mqtt_timer.tick();
  notify_timer.tick();
}