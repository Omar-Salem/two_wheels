//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <util/atomic.h>
#include "PIDController.h" //https://github.com/DonnyCraft1/PIDArduino
#include "MotorConfig.h"


class Motor {
public:
    Motor(MotorConfig motorConfig,
          byte pwmPin,
          byte firstBridgePin,
          byte secondBridgePin,
          byte velocityEncoder,
          byte directionEncoder);

    void initialize();

    double calculateAngularVelocity();

    double getPosition() const;

    byte getVelocityEncoder() const;

    void move(double velocity);

    void movePWM(int pwm);

    void interruptCallback(bool isLeft);

    void tunePID(double p, double i, double d);

private:
    int encCountRev;
    byte pwmPin;
    byte firstBridgePin;
    byte secondBridgePin;
    byte velocityEncoder;
    byte directionEncoder;

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

    void setDirectionForward();

    void setDirectionBackward();
};

#endif
