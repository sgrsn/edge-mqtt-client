#pragma once

#include <cstring>

/*I2C slave address*/
#define ESP32_I2C_ADDR 0x08

/*I2C slave register*/
#define MODEM_STATUS_REG            0
#define ESP32_IS_READY_REG          1
#define MODEM_JOYSTICK_X_REG        2
#define MODEM_JOYSTICK_Y_REG        3
#define MODEM_SLIDER_REG            4
#define MODEM_START_STOP_REG        5
#define MODEM_HEARTBEAT_STATUS_REG  6
#define GNSS_STATUS_REG             7
#define LOGGING_STATUS_REG          8
#define TEENSY41_STATUS_REG         9

struct EdgeStatus
{
  uint8_t vectornav  : 2 = 0;
  uint8_t sdcard     : 2 = 0;
  uint8_t gnss_fix   : 2 = 0;
  operator int() const
  {
    uint8_t result;
    std::memcpy(&result, this, sizeof(EdgeStatus));
    return result;
  }
  EdgeStatus& operator=(const uint8_t& other)
  {
    std::memcpy(this, &other, sizeof(EdgeStatus));
    return *this;
  }
};

enum ErrorCode
{
  NO_ERROR  = 0,
  SUCCESS   = 1,
  FAILED    = 2,
  TIMEOUT   = 3,
};