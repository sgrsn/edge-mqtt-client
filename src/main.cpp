#include <Arduino.h>
#include "secret.h"
#include "i2cslave.hpp"
#include "i2c_register.h"
#include <PrintServer.h>
DumpServer debug;
#include "mqtt_client.hpp"

HardwareSerial serial_(0);
MqttClient mqtt(serial_, APN, GPRS_USER, GPRS_PASS, BROKER, PORT, CLIENT_ID, USERNAME, PASSWORD);
I2CSlave i2cSlave;

// 頻度
const uint16_t WATCHDOG_INTERVAL = 800;
const uint16_t WATCHDOG_TIMEOUT = 2000;

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

  i2cSlave.setRegister(MODEM_STATUS_REG,    modem_status);
  i2cSlave.setRegister(ESP32_IS_READY_REG,  1);
  i2cSlave.init(ESP32_I2C_ADDR);

  debug.println("(setup)", "end");
}

void monitoring()
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
}

void loop() 
{
  static uint32_t last_monitoring = 0;

  uint32_t now = millis();

  mqtt.mqttLoop();

  int x, y, slider;
  bool startStop;

  mqtt.getLastValue<int>("control/joystick/x", x);
  mqtt.getLastValue<int>("control/joystick/y", y);
  mqtt.getLastValue<int>("control/slider", slider);
  mqtt.getLastValue<bool>("control/startStop", startStop);

  i2cSlave.setRegister(MODEM_JOYSTICK_X_REG,  int(x));
  i2cSlave.setRegister(MODEM_JOYSTICK_Y_REG,  int(y));
  i2cSlave.setRegister(MODEM_SLIDER_REG,      int(slider));
  i2cSlave.setRegister(MODEM_START_STOP_REG,  startStop);

  if (millis() - last_monitoring > WATCHDOG_INTERVAL)
  {
    monitoring();
    last_monitoring = millis();
  }

  debug.println("Loop take", millis() - now);
}