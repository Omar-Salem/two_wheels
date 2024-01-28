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

    pid.begin();
    pid.limit(0, 255);
    pid.tune(Kp, Ki, Kd);
}

double Motor::getAngularVelocity() {
    float velocity2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        velocity2 = velocity_i;
    }
    //    // Low-pass filter (25 Hz cutoff)
    v2Filt = 0.854 * v2Filt + 0.0728 * velocity2 + 0.0728 * v2Prev;
    v2Prev = velocity2;

    double rpm = v2Filt * 60 / encCountRev;
    return rpm * RPM_TO_RADIANS;
}

int Motor::getAngle() {
    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pos = posi;
    }
    return pos * precision; //https://qr.ae/pKE7DV
}

void Motor::move(double targetVelocity) {
    pid.setpoint(targetVelocity);
    int pwm = pid.compute(getAngularVelocity());
    movePWM(pwm);
}

void Motor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
    setDirectionForward();
}

void Motor::interruptCallback() {
    long currT = micros();
    float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
    velocity_i = 1 / deltaT;
    prevT_i = currT;

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
