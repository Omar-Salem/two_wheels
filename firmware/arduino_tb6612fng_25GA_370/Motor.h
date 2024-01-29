//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <util/atomic.h>
#include <PID_v1_bc.h> // https://github.com/drf5n/Arduino-PID-Library


class Motor {
public:
    Motor(int encCountRev, int pwmPin, int firstBridgePin, int secondBridgePin,
          int encoder1Pin,
          int encoder2Pin);

    void initialize();

    double getAngularVelocity();

    double getAngle();

    int getEncoderPin();

    void move(double velocity);

    void movePWM(int pwm);

    void interruptCallback();

private:
    int encCountRev;

    int pwmPin;
    int firstBridgePin;
    int secondBridgePin;
    int encoder1Pin;
    int encoder2Pin;

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

    const double Kp = 190;
    const double Ki = 3.7;
    const double Kd = 0.1;

    PID *velocityPID;
    double Setpoint, Input, Output;

    void setDirectionForward();
};

#endif
