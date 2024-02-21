//
// Created by omar on 2/8/24.
//


#include <iostream>
#include "ArduinoFirmware.h"
#include<algorithm>

constexpr int MAX_PING_RETRY = 5;

void ArduinoFirmware::configure() {
    serial = make_unique<SerialOps>(SERIAL_PORT, BAUD);
}

void ArduinoFirmware::ping() {
    for (int i = 0; i < MAX_PING_RETRY; ++i) {
        const auto value = readString(PING);
        if (value == "PONG") {
            return;
        }
        std::this_thread::sleep_for(500ms);
    }
    throw runtime_error("************************ SerialConnectionError");
}

double ArduinoFirmware::getFirstMotorPosition() {
    const auto value = readString(GET_MOTOR_1_POSITION);
    return std::stod(value);
}

double ArduinoFirmware::getFirstMotorVelocity() {
    const auto value = readString(GET_MOTOR_1_VELOCITY);
    return std::stod(value);
}

void ArduinoFirmware::setFirstMotorVelocity(double v) {
    string command = std::regex_replace(WRITE_COMMAND_TEMPLATE,
                                        std::regex("#command"),
                                        to_string(MOVE_MOTOR_1));
    command = std::regex_replace(command,
                                 std::regex("#velocity"),
                                 to_string(v));
    serial->write(command);
}

string ArduinoFirmware::readString(int commandNumber) {
    const string command = regex_replace(READ_COMMAND_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));

    return serial->read(command);
}
