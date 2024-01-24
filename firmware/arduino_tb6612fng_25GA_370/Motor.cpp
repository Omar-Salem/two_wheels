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
    pid.tune(4096, 0, 0);    // Tune the PID, arguments: kP, kI, kD
    pid.limit(0, 255);
}

void Motor::odom() {
    // Record the time
    currentMillis = millis();

    // If one second has passed, print the number of pulses
    if (currentMillis - previousMillis > INTERVAL_MILLI_SEC) {

        previousMillis = currentMillis;

        // Calculate revolutions per minute
        rpm = (float) (pulseCount * 60 / encCountRev);
        angVelocity = rpm * RPM_TO_RADIANS;
        pulseCount = 0;
    }
}

double Motor::getLinearVelocity() {
    return wheelRadiusMeters * angVelocity;
}

void Motor::move(double velocity) {
    digitalWrite(firstBridgePin, LOW);
    digitalWrite(secondBridgePin, HIGH);
    pid.setpoint(velocity);
    auto output = pid.compute(this->getLinearVelocity());
    analogWrite(pwmPin, output);
}

void Motor::interruptCallback() {
    pulseCount++;
}

int Motor::getEncoderPin() {
    return encoderPin;
}
