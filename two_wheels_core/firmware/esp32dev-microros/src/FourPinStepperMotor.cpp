//
// Created by omar on 4/2/24.
//

#include "FourPinStepperMotor.h"

FourPinStepperMotor::FourPinStepperMotor(byte firstPin,
                                         byte secondPin,
                                         byte thirdPin,
                                         byte fourthPin,
                                         bool isLeft) {
    directionMultiplier_ = isLeft ? 1 : -1;
    commandLastPing_ = millis();
    accelStepper = new AccelStepper(FULLSTEP, firstPin, thirdPin, secondPin, fourthPin);

    accelStepper->setMaxSpeed(MAX_STEPS_PER_SECOND);
}

double FourPinStepperMotor::getAngularVelocity() {
    return accelStepper->speed() * CONVERT_DEG_TO_RAD;
}

double FourPinStepperMotor::getPosition() const {
    int currentPosition = accelStepper->currentPosition();
    currentPosition *= ANGLES_PER_STEP;
    return currentPosition * CONVERT_DEG_TO_RAD * directionMultiplier_;
}

void FourPinStepperMotor::setVelocity(double angularVelocity) {
    commandLastPing_ = millis();
    const double angularVelocityDegrees = angularVelocity * CONVERT_RAD_TO_DEG;
    const double stepsPerSecond = angularVelocityDegrees / ANGLES_PER_STEP;
    accelStepper->setSpeed(stepsPerSecond * directionMultiplier_);
}

void FourPinStepperMotor::move() {
    // if (millis() - commandLastPing_ >= DEAD_MAN_SWITCH_TIMEOUT_MILLI_SEC) {
    //     accelStepper->setSpeed(0);
    // }
    accelStepper->runSpeed();
}
