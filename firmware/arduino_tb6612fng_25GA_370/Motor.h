//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <util/atomic.h>
#include <PIDController.h>


class Motor {
public:
    Motor(int encCountRev, int pwmPin, int firstBridgePin, int secondBridgePin,
          int encoderPin);

    void initialize();

    double getAngularVelocity();

    int getEncoderPin();

    int getRPM();

    void move(double velocity);

    void movePWM(int pwm);

    void interruptCallback();

    void odom();

private:
    int encCountRev;

    int pwmPin;
    int firstBridgePin;
    int secondBridgePin;
    int encoderPin;

    double rpm;
    volatile float velocity_i = 0;
    volatile long prevT_i = 0;

    double angVelocity;
    float v2Filt = 0;
    float v2Prev = 0;

    const double RPM_TO_RADIANS = 0.10471975512;

    const double Kp = 800;
    const double Ki = 0;
    const double Kd = 0;

    PIDController pid;

    void setDirectionForward();
};

#endif
