//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <util/atomic.h>
#include <PIDController.h> //https://github.com/DonnyCraft1/PIDArduino


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

    double Kp = 600;
    double Ki = 6.5;
    double Kd = 0.4;

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

    PIDController pid;

    void setDirectionForward();

    void odom();
};

#endif
