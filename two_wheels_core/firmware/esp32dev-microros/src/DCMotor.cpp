//
// Created by omar on 1/24/24.
//
#include "DCMotor.h"

DCMotor::DCMotor(DCMotorConfig motorConfig,
                 byte pwmPin,
                 byte firstBridgePin,
                 byte secondBridgePin,
                 byte velocityEncoder,
                 byte directionEncoder,
                 bool isLeft) : Motor(isLeft) {
        
    encCountRev_ = motorConfig.getEncCountRev();
    this->pwmPin = pwmPin;
    this->firstBridgePin_ = firstBridgePin;
    this->secondBridgePin_ = secondBridgePin;
    this->velocityEncoder_ = velocityEncoder;
    this->directionEncoder_ = directionEncoder;
    precision = (2 * PI) / motorConfig.getEncCountRev();
    velocityPID.tune(motorConfig.getKp(),
                     motorConfig.getKi(),
                     motorConfig.getKd());
}

void DCMotor::initialize(void (*interruptCallback)()) {
    pinMode(pwmPin, OUTPUT);
    pinMode(firstBridgePin_, OUTPUT);
    pinMode(secondBridgePin_, OUTPUT);
    pinMode(velocityEncoder_, INPUT_PULLUP);
    pinMode(directionEncoder_, INPUT);
    attachInterrupt(digitalPinToInterrupt(getVelocityEncoderPin()), interruptCallback, RISING);
}

double DCMotor::getAngularVelocity() {
    float velocity2 = 0;
    // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //TODO critical section
    // {
    velocity2 = velocity;
    // }

    if (millis() - lastUpdated > interval) {
        //reset velocity (measured in interruptCallback), otherwise will be stuck
        // on last reading when motor stops moving
        velocity = 0;
    }
    // Low-pass filter (25 Hz cutoff)
    velocityFiltered = 0.854 * velocityFiltered + 0.0728 * velocity2 + 0.0728 * velocityPrev;
    velocityPrev = velocity2;

    double rpm = velocityFiltered * 60 / encCountRev_;
    return rpm * RPM_TO_RADIANS;
}

double DCMotor::getPosition() const {
    double pos = 0;
    // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) //TODO critical section
    // {
    pos = posi;
    // }
    return pos * precision; //https://qr.ae/pKE7DV
}

void DCMotor::move(double targetVelocity) {
    if (targetVelocity > 0) {
        setDirectionForward();
    } else {
        setDirectionBackward();
        targetVelocity *= -1;
    }
    double currentV = getAngularVelocity();
    int pwm = velocityPID.compute(currentV, targetVelocity);
    movePWM(pwm);
}

void DCMotor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
}

void DCMotor::interruptCallback() {
    lastUpdated = millis();
    long currT = micros();
    float deltaT = ((float) (currT - prevTime)) / 1.0e6;
    velocity = 1 / deltaT;
    prevTime = currT;

    bool direction = digitalRead(directionEncoder_);
    if (this->isLeft_) {
        direction > 0 ? posi++ : posi--;
    } else {
        direction > 0 ? posi-- : posi++;
    }
}

void DCMotor::setDirectionForward() {
    digitalWrite(firstBridgePin_, HIGH);
    digitalWrite(secondBridgePin_, LOW);
}

void DCMotor::setDirectionBackward() {
    digitalWrite(firstBridgePin_, LOW);
    digitalWrite(secondBridgePin_, HIGH);
}

byte DCMotor::getVelocityEncoderPin() const {
    return velocityEncoder_;
}

void DCMotor::tunePID(double p, double i, double d) {
    velocityPID.tune(p,
                     i,
                     d);
}
