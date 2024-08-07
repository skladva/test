/*
  Motor.h - Library for working with the Double BTS7960 43A H-Bridge Motor Driver.
  Updated by ChatGPT, July 16, 2024.
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    // Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
    Motor(int plus, int minus, int en_a, int en_b);
    // Spin the motor with a percentage value
    void rotate(int value);
    // Motor Outputs - plus is one direction and minus is the other
    int plus;
    int minus;
    // Encoder Inputs
    int en_a;
    int en_b;
};

#endif
