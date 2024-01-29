//
// Created by omar on 1/24/24.
//
#include "Motor.h"

Motor::Motor(int encCountRev, int pwmPin, int firstBridgePin, int secondBridgePin,
             int encoder1Pin,
             int encoder2Pin)
        : encCountRev(encCountRev),
          pwmPin(pwmPin),
          firstBridgePin(firstBridgePin),
          secondBridgePin(secondBridgePin),
          encoder1Pin(encoder1Pin),
          encoder2Pin(encoder2Pin) {
    precision = (2 * PI) / encCountRev;
}

void Motor::initialize() {
    pinMode(pwmPin, OUTPUT);
    pinMode(firstBridgePin, OUTPUT);
    pinMode(secondBridgePin, OUTPUT);
    pinMode(encoder1Pin, INPUT_PULLUP);
    pinMode(encoder2Pin, INPUT);

    velocityPID.begin();
    velocityPID.limit(0, 255);
    velocityPID.tune(Kp, Ki, Kd);
}

double Motor::getAngularVelocity() {
    float velocity2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        velocity2 = velocity;
    }

    if (millis() - lastUpdated > interval) {
        //reset velocity (measured in interruptCallback), otherwise will be stuck
        // on last reading when motor stops moving
        velocity = 0;
    }
    // Low-pass filter (25 Hz cutoff)
    velocityFiltered = 0.854 * velocityFiltered + 0.0728 * velocity2 + 0.0728 * velocityPrev;
    velocityPrev = velocity2;

    double rpm = velocityFiltered * 60 / encCountRev;
    return rpm * RPM_TO_RADIANS;
}

double Motor::getAngle() {
    double pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pos = posi;
    }
    return pos * precision; //https://qr.ae/pKE7DV
}

void Motor::move(double targetVelocity) {
    velocityPID.setpoint(targetVelocity);
    int pwm = velocityPID.compute(getAngularVelocity());
    movePWM(pwm);
}

void Motor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
    setDirectionForward();
}

void Motor::interruptCallback() {
    lastUpdated = millis();
    long currT = micros();
    float deltaT = ((float) (currT - prevTime)) / 1.0e6;
    velocity = 1 / deltaT;
    prevTime = currT;

    int direction = digitalRead(encoder2Pin);
    direction > 0 ? posi++ : posi--;
}

void Motor::setDirectionForward() {
    digitalWrite(firstBridgePin, HIGH);
    digitalWrite(secondBridgePin, LOW);
}

int Motor::getEncoderPin() {
    return encoder1Pin;
}
