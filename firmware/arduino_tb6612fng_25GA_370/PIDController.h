#ifndef PIDControllerLib
#define PIDControllerLib

#include <Arduino.h>

class PIDController {
public:

    void tune(double Kp, double Ki, double Kd);

    double compute(double current, double target);


private:
    unsigned long lastTime;
    unsigned long SampleTime = 100;
    double lastTarget;
    double outputSum;
    double Kp_ = 1;
    double Ki_ = 1;
    double Kd_ = 1;

    const double outMax = 255;
    const double outMin = 0;
};

#endif
