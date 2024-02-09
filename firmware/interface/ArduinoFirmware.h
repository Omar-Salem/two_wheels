//
// Created by omar on 2/8/24.
//

#ifndef TWO_WHEELS_ARDUINOFIRMWARE_H
#define TWO_WHEELS_ARDUINOFIRMWARE_H


#include "Firmware.h"
#include "serialib.h"
#include <regex>
#include <nlohmann/json.hpp>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD 115200

#define MOVE_MOTOR_1 1
#define MOVE_MOTOR_2 2
#define GET_MOTOR_1_VELOCITY 3
#define GET_MOTOR_2_VELOCITY 4
#define GET_MOTOR_1_POSITION 5
#define GET_MOTOR_2_POSITION 6

using json = nlohmann::json;
using namespace std;

class ArduinoFirmware : public Firmware {
public:
    void configure();

    double getFirstMotorPosition();

    double getFirstMotorVelocity();

    void setFirstMotorVelocity(double v);

private:
    serialib serial;

    void sendCommand(const string &command);

    double readCommand();

    const string READ_COMMAND_TEMPLATE = R"({"command":#command})";
    const string WRITE_COMMAND_TEMPLATE = R"({"command":#command,"params":{"velocity":#velocity}})";

    double readCommandUtil(int commandNumber);
};

#endif //TWO_WHEELS_ARDUINOFIRMWARE_H