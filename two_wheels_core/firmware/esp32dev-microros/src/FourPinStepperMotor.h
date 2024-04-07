//
// Created by omar on 4/2/24.
//

#ifndef TWO_WHEELS_STEPPERMOTOR_H
#define TWO_WHEELS_STEPPERMOTOR_H

#include <AccelStepper.h>

class FourPinStepperMotor {
private:
    AccelStepper *accelStepper;
    const int FULLSTEP = 4;
    const int STEPS_PER_REVOLUTION = 2048;
    const int MAX_STEPS_PER_SECOND = STEPS_PER_REVOLUTION / FULLSTEP;
    const double ANGLES_PER_STEP = 0.17578125;
    const double CONVERT_DEG_TO_RAD = 0.0174533;
    const double MAX_ANGULAR_VELOCITY = MAX_STEPS_PER_SECOND*ANGLES_PER_STEP * CONVERT_DEG_TO_RAD;//1.570797 r/s
    const double CONVERT_RAD_TO_DEG = 57.29578;
    int directionMultiplier_;
    long commandLastPing_ ;
    const long DEAD_MAN_SWITCH_TIMEOUT_MILLI_SEC=1000;

public:
    FourPinStepperMotor(byte firstPin,
                        byte secondPin,
                        byte thirdPin,
                        byte fourthPin,
                        bool isLeft);

    double getAngularVelocity();

    double getPosition() const;

    void setVelocity(double angularVelocity);

    void move();
};


#endif //TWO_WHEELS_STEPPERMOTOR_H
