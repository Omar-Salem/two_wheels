//
// Created by omar on 2/8/24.
//

#ifndef TWO_WHEELS_ARDUINOFIRMWARE_H
#define TWO_WHEELS_ARDUINOFIRMWARE_H


#include "Firmware.h"
#include "serialib.h"
#include <regex>
#include <chrono>
#include <thread>
#include "MotorsOdom.h"

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD 115200

#define PING 0
#define MOVE_MOTORS 1
#define GET_MOTORS_ODOM 2

using namespace std;

class ArduinoFirmware : public Firmware {
public:
    void connect() override;

    void disconnect() override;

    void ping() override;

    void setMotorsVelocity(double m1, double m2) override;

    MotorsOdom getMotorsOdom() override;

private:
    serialib serial;

    string readOutput();

    void writeQueryCommand(int commandNumber);

    vector<string> split(const string &str);

    const string QUERY_TEMPLATE = "#command\n";
    const string COMMAND_TEMPLATE = "#m1,#m2\n";
};

#endif //TWO_WHEELS_ARDUINOFIRMWARE_H