#ifndef PIDControllerLib
#define PIDControllerLib

#include <PID_v1_bc.h> // https://github.com/drf5n/Arduino-PID-Library

class PIDController {
public:

    void init(double Kp, double Ki, double Kd, double min, double max);

    double compute(double current, double target);


private:
    double target_, current_, output_;
    PID *pid_;
};

#endif
