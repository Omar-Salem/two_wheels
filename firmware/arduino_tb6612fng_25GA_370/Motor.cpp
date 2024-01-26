//
// Created by omar on 1/24/24.
//
#include "Motor.h"

Motor::Motor(int encCountRev, double wheelRadiusMeters, int pwmPin, int firstBridgePin, int secondBridgePin,
             int encoderPin)
        : encCountRev(encCountRev),
          wheelRadiusMeters(wheelRadiusMeters),
          pwmPin(pwmPin),
          firstBridgePin(firstBridgePin),
          secondBridgePin(secondBridgePin),
          encoderPin(encoderPin),
          pulseCount(0),
          rpm(0),
          angVelocity(0),
          previousMillis(0),
          currentMillis(0) {}

void Motor::initialize() {
    pinMode(pwmPin, OUTPUT);
    pinMode(firstBridgePin, OUTPUT);
    pinMode(secondBridgePin, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);
    pid.begin();
    pid.tune(Kp, Ki, Kd);    // Tune the PID, arguments: kP, kI, kD
    pid.limit(0, 255);
    setDirectionForward();
}

void Motor::odom() {
    rpm = pulseCount * 60 / encCountRev;
    angVelocity = rpm * RPM_TO_RADIANS;
    pulseCount = 0;  //  reset counter to zero
    // Enable the timer
}

double Motor::getLinearVelocity() {
    return wheelRadiusMeters * angVelocity;
}

void Motor::tune(double p, double i, double d) {
    pid.tune(p, i, d);
}

void Motor::move(double velocity) {
    pid.setpoint(velocity);
    auto pwm = pid.compute(this->getLinearVelocity());
    analogWrite(pwmPin, pwm);
}

void Motor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
}

void Motor::interruptCallback() {
    pulseCount++;
}

void Motor::setDirectionForward() {
    digitalWrite(firstBridgePin, LOW);
    digitalWrite(secondBridgePin, HIGH);
}

int Motor::getEncoderPin() {
    return encoderPin;
}
