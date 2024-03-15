//
// Created by omar on 1/24/24.
//
#include "Motor.h"

Motor::Motor(MotorConfig motorConfig,
             byte pwmPin,
             byte firstBridgePin,
             byte secondBridgePin,
             byte velocityEncoder,
             byte directionEncoder)
        : encCountRev(motorConfig.getEncCountRev()),
          pwmPin(pwmPin),
          firstBridgePin(firstBridgePin),
          secondBridgePin(secondBridgePin),
          velocityEncoder(velocityEncoder),
          directionEncoder(directionEncoder) {
    precision = (2 * PI) / motorConfig.getEncCountRev();
    velocityPID.tune(motorConfig.getKp(),
                     motorConfig.getKi(),
                     motorConfig.getKd());
}

void Motor::initialize() {
    pinMode(pwmPin, OUTPUT);
    pinMode(firstBridgePin, OUTPUT);
    pinMode(secondBridgePin, OUTPUT);
    pinMode(velocityEncoder, INPUT_PULLUP);
    pinMode(directionEncoder, INPUT);
}

double Motor::calculateAngularVelocity() {
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

double Motor::getPosition() const {
    double pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pos = posi;
    }
    return pos * precision; //https://qr.ae/pKE7DV
}

void Motor::move(double targetVelocity) {
    if (targetVelocity > 0) {
        setDirectionForward();
    } else {
        setDirectionBackward();
        targetVelocity *= -1;
    }
    double currentV = calculateAngularVelocity();
    int pwm = velocityPID.compute(currentV, targetVelocity);
    movePWM(pwm);
}

void Motor::movePWM(int pwm) {
    analogWrite(pwmPin, pwm);
}

void Motor::interruptCallback(bool isLeft) {
    lastUpdated = millis();
    long currT = micros();
    float deltaT = ((float) (currT - prevTime)) / 1.0e6;
    velocity = 1 / deltaT;
    prevTime = currT;

    bool direction = digitalRead(directionEncoder);
    if (isLeft) {
        direction > 0 ? posi++ : posi--;
    } else {
        direction > 0 ? posi-- : posi++;
    }
}

void Motor::setDirectionForward() {
    digitalWrite(firstBridgePin, HIGH);
    digitalWrite(secondBridgePin, LOW);
}

void Motor::setDirectionBackward() {
    digitalWrite(firstBridgePin, LOW);
    digitalWrite(secondBridgePin, HIGH);
}

byte Motor::getVelocityEncoder() const {
    return velocityEncoder;
}

void Motor::tunePID(double p, double i, double d) {
    velocityPID.tune(p,
                     i,
                     d);
}
