//
// Created by omar on 2/8/24.
//
#ifndef TWO_WHEELS_FIRMWARE_H
#define TWO_WHEELS_FIRMWARE_H


#include "MotorsOdom.h"

class Firmware {
public:

    virtual void connect() = 0;

    virtual void disconnect() = 0;

    virtual void ping() = 0;

    virtual void setMotorsVelocity(double m1, double m2) = 0;

    virtual MotorsOdom getMotorsOdom() = 0;
};

#endif //TWO_WHEELS_FIRMWARE_H
