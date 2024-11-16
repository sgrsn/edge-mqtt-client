#include <Arduino.h>
#include <ESP32Servo.h>

class ServoController {
private:
  const int servoPin;
  const int channel;
  const int frequency = 50;  // 50Hz for most servos
  const int resolution = 10; // 10-bit resolution (0-1023)
  
  // Microseconds range
  int minUs;
  int maxUs;
  int neutralUs;
  
  // Duty cycle range
  int minDuty;
  int maxDuty;
  
  // Current position
  double currentPosition = 0;

public:
  ServoController(int pin, int ch = 1, int min_us = 1000, int max_us = 2000, int neutral_us = 1500) 
    : servoPin(pin), channel(ch), minUs(min_us), maxUs(max_us), neutralUs(neutral_us) {
    initialize();
  }
  
  void initialize() {
    pinMode(servoPin, OUTPUT);
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(servoPin, channel);
    
    // Calculate duty cycle ranges
    minDuty = map(minUs, 0, 20000, 0, 1023);
    maxDuty = map(maxUs, 0, 20000, 0, 1023);
    
    // Set to neutral position
    setToNeutral();
  }
  
  // Set position from -100 to 100
  void setPosition(double position) {
    // Constrain input
    position = constrain(position, -100.0, 100.0);
    currentPosition = position;
    
    // Map position to duty cycle
    int duty = map(position, -100, 100, minDuty, maxDuty);
    ledcWrite(channel, duty);
  }
  
  // Set to neutral position
  void setToNeutral() {
    int neutralDuty = map(neutralUs, 0, 20000, 0, 1023);
    ledcWrite(channel, neutralDuty);
    currentPosition = 0;
  }
  
  // Get current position (-100 to 100)
  double getCurrentPosition() {
    return currentPosition;
  }
  
  // Calibration methods
  void setMinUs(int us) { minUs = us; initialize(); }
  void setMaxUs(int us) { maxUs = us; initialize(); }
  void setNeutralUs(int us) { neutralUs = us; initialize(); }
};