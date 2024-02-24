//
// Created by omar on 2/8/24.
//
#ifndef TWO_WHEELS_FIRMWARE_H
#define TWO_WHEELS_FIRMWARE_H


class Firmware {
public:

    virtual void connect() = 0;

    virtual void disconnect() = 0;

    virtual void ping() = 0;

    virtual double getFirstMotorPosition() = 0;

    virtual double getFirstMotorVelocity() = 0;

    virtual void setFirstMotorVelocity(double v) = 0;

    virtual double getSecondMotorPosition() = 0;

    virtual double getSecondMotorVelocity() = 0;

    virtual void setSecondMotorVelocity(double v) = 0;
};

#endif //TWO_WHEELS_FIRMWARE_H
