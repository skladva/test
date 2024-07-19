#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int plus, int minus, int en_a, int en_b) {
  pinMode(plus, OUTPUT);
  pinMode(minus, OUTPUT);
  pinMode(en_a, INPUT_PULLUP);
  pinMode(en_b, INPUT_PULLUP);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::en_a = en_a;
  Motor::en_b = en_b;
  
}

void Motor::rotate(int value) {
  if (value >= 0) {
    int out = map(value, 0, 100, 0, 255);
    ledcWrite(plus, out);  // Use ledcWrite for ESP32 PWM control
    ledcWrite(minus, 0);
  } else {
    int out = map(value, 0, -100, 0, 255);
    ledcWrite(plus, 0);
    ledcWrite(minus, out);
  }
}
