//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <util/atomic.h>
#include "PIDController.h" //https://github.com/DonnyCraft1/PIDArduino


class Motor {
public:
    Motor(int encCountRev,
          byte pwmPin,
          byte firstBridgePin,
          byte secondBridgePin,
          byte encoder1Pin,
          byte encoder2Pin);

    void initialize();

    double calculateAngularVelocity();

    double getAngle() const;

    byte getEncoderPin() const;

    void move(double velocity);

    void movePWM(int pwm);

    void interruptCallback();

private:
    int encCountRev;

    byte pwmPin;
    byte firstBridgePin;
    byte secondBridgePin;
    byte encoder1Pin;
    byte encoder2Pin;

    volatile double posi = 0;
    double precision;

    // Half-second interval for measurements
    const int interval = 500;
    long lastUpdated = 0;

    volatile float velocity = 0;
    volatile long prevTime = 0;
    float velocityFiltered = 0;
    float velocityPrev = 0;

    const double RPM_TO_RADIANS = 0.10471975512;

    PIDController velocityPID;

    const double Kp = 1.0;
    const double Ki = 0;
    const double Kd = 0;

    void setDirectionForward();
};

#endif
