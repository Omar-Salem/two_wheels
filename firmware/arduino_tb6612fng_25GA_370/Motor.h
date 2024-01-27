//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <util/atomic.h>


class Motor {
public:
    Motor(int encCountRev, double wheelRadiusMeters, int pwmPin, int firstBridgePin, int secondBridgePin,
          int encoderPin);

    void initialize();

    double getLinearVelocity();

    double getAngularVelocity();

    int getEncoderPin();

    int getRPM();

    void move(double velocity);

    void movePWM(int pwm);

    void interruptCallback();

    void odom();

private:
    int encCountRev;
    double wheelRadiusMeters;

    int pwmPin;
    int firstBridgePin;
    int secondBridgePin;
    int encoderPin;

    double rpm;
    volatile float velocity_i = 0;
    volatile long prevT_i = 0;

    double angVelocity;
    double eintegral = 0;
    long prevT = 0;
    float v2Filt = 0;
    float v2Prev = 0;

    const double RPM_TO_RADIANS = 0.10471975512;

    const double Kp = 2000;
    const double Ki = 500;

    void setDirectionForward();
};

#endif
