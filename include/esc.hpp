#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_timer.h"

class EscController {
private:
  const int servoPin;
  const int channel;
  const int frequency = 50;  // 50Hz for most servos
  const int resolution = 10; // 10-bit resolution (0-1023)
  
  // Microseconds range
  int minUs;
  int maxUs;
  
  // Duty cycle range
  int minDuty;
  int maxDuty;

  double speed_ = 0;

public:
  EscController(int pin, int ch = 1, int min_us = 800, int max_us = 2200) 
    : servoPin(pin), channel(ch), minUs(min_us), maxUs(max_us){
    initialize();
  }
  
  void initialize() {
    pinMode(servoPin, OUTPUT);
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(servoPin, channel);
    
    // Calculate duty cycle ranges
    minDuty = map(minUs, 0, 20000, 0, 1023);
    maxDuty = map(maxUs, 0, 20000, 0, 1023);
    
    // stop motor
    stop();
  }

  void initializeTimer()
  {
    esp_timer_handle_t testTimer;
    esp_timer_create_args_t timerConfig;
    timerConfig.arg = this;
    timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(__callback__);
    timerConfig.dispatch_method = ESP_TIMER_TASK;
    timerConfig.name = "EscControllerTimer";
    esp_timer_create(&timerConfig, &testTimer);
    esp_timer_start_periodic(testTimer, 100000);
  }

  static void __callback__(void *arg) {
    EscController *obj = (EscController *)arg;
    obj->callback();
  }
    
  void callback(void) {
    setSpeed(speed_);
    // debug.println("callback: ", millis(), ", ", speed_);
  }
  
  // stop motor
  void stop() {
    setSpeed(0);
  }

  // Set speed from 0 to 100
  void setSpeed(double speed) {
    // Constrain input
    speed = constrain(speed, 0.0, 100.0);
    speed_ = speed;
    
    // Map speed to duty cycle
    int duty = map(speed, 0, 100, minDuty, maxDuty);
    ledcWrite(channel, duty);
  }

  void linearAccelSpeed(double speed, double accel=5, double bias=0) {
    if (speed_ < bias) {
      speed_ += bias;
    }
    if (speed_ < speed) {
      speed_ += accel;
      if (speed_ > speed) speed_ = speed;
    } else if (speed_ > speed) {
      speed_ -= accel;
      if (speed_ < speed) speed_ = speed;
    }
  }

  double speed() {
    return speed_;
  }
  
  // Calibration methods
  void setMinUs(int us) { minUs = us; initialize(); }
  void setMaxUs(int us) { maxUs = us; initialize(); }
};