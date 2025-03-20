#pragma once

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