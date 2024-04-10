//
// Created by omar on 1/24/24.
//
#ifndef DCMOTOR_h
#define DCMOTOR_h

#include <Arduino.h>
#include <Motor.h>
#include "PIDController.h" //https://github.com/DonnyCraft1/PIDArduino
#include "DCMotorConfig.h"


class DCMotor : public Motor{
public:
    DCMotor(DCMotorConfig motorConfig,
          byte pwmPin,
          byte firstBridgePin,
          byte secondBridgePin,
          byte velocityEncoder,
          byte directionEncoder,
          bool isLeft);

    double getAngularVelocity();

    double getPosition() const;

    void move(double velocity);

    void initialize(void (*interruptCallback)());

    void interruptCallback();

    void movePWM(int pwm);

    byte getVelocityEncoderPin() const;

    void tunePID(double p, double i, double d);

private:
    int encCountRev_;
    byte pwmPin;
    byte firstBridgePin_;
    byte secondBridgePin_;
    byte velocityEncoder_;
    byte directionEncoder_;

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
