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

void setup() {
  serial_.begin(115200, SERIAL_8N1, RX, TX);
  WiFi.softAP("MyESP32", "12345678");
  debug.begin();
  debug.println("(setup)", "start");
  mqtt.init();
  mqtt.registerTopic<std::string>("esp/watchdog/heartbeat");
  mqtt.registerTopic<bool>("control/startStop");
  mqtt.registerTopic<std::string>("control/slider");
  mqtt.registerTopic<double>("control/joystick/x");
  mqtt.registerTopic<double>("control/joystick/y");
  debug.println("(setup)", "end");

  i2cSlave.init(ESP32_I2C_ADDR);
}

void monitoring()
{
  std::string heartbeat;
  mqtt.getLastValue("esp/watchdog/heartbeat", heartbeat);
  unsigned long last_heartbeat = 0;
  if (!heartbeat.empty())
    last_heartbeat = std::stoi(heartbeat);
  unsigned long error_time = millis() - last_heartbeat;
  debug.println("MonitoringTimer", error_time);
  mqtt.publish("esp/watchdog/heartbeat", std::to_string(millis()));
}

void loop() 
{
  static uint32_t last_monitoring = 0;

  uint32_t now = millis();

  mqtt.mqttLoop();

  double x, y;
  mqtt.getLastValue("control/joystick/x", x);
  mqtt.getLastValue("control/joystick/y", y);
  i2cSlave.setRegister(MODEM_JOYSTICK_X_REG, int(x));
  i2cSlave.setRegister(MODEM_JOYSTICK_Y_REG, int(y));
  debug.println("X: ",  int(x), " Y: ", int(y));

  if (millis() - last_monitoring > 1000)
  {
    monitoring();
    last_monitoring = millis();
  }

  debug.println("Loop take", millis() - now);
}