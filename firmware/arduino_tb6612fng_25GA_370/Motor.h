//
// Created by omar on 1/24/24.
//
#ifndef MOTOR_h
#define MOTOR_h

#include <Arduino.h>
#include <PIDController.h>  //https://github.com/DonnyCraft1/PIDArduino


class Motor {
public:
    Motor(int encCountRev, double wheelRadiusMeters, int pwmPin, int firstBridgePin, int secondBridgePin,
          int encoderPin);

    void initialize();

    void odom();

    double getLinearVelocity();

    int getEncoderPin();

    void move(double velocity);

    void interruptCallback();

private:
    int encCountRev;
    double wheelRadiusMeters;

    int pwmPin;
    int firstBridgePin;
    int secondBridgePin;
    int encoderPin;
    volatile long pulseCount;
    double rpm;
    float angVelocity;
    const int INTERVAL_MILLI_SEC = 1000;
    const double RPM_TO_RADIANS = 0.10471975512;

    long previousMillis = 0;
    long currentMillis = 0;
    PIDController pid;
};

#endif
