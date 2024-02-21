//
// Created by omar on 2/8/24.
//

#include <iostream>
#include "ArduinoFirmware.h"
#include <algorithm>

constexpr int MAX_PING_RETRY = 5;

void ArduinoFirmware::configure() {
    char errorOpening = serial.openDevice(SERIAL_PORT, BAUD);
    if (errorOpening != 1) {
        throw runtime_error("************************ SerialConfigurationError");
    }
}

void ArduinoFirmware::ping() {
    for (int i = 0; i < MAX_PING_RETRY; ++i) {
        writeCommand(PING);
        const auto value = readOutput();
        if (value == "PONG") {
            return;
        }
        std::this_thread::sleep_for(500ms);
    }
    throw runtime_error("************************ SerialConnectionError");
}

double ArduinoFirmware::getFirstMotorPosition() {
    writeCommand(GET_MOTOR_1_POSITION);
    const auto value = readOutput();
    return std::stod(value);
}

double ArduinoFirmware::getFirstMotorVelocity() {
    writeCommand(GET_MOTOR_1_VELOCITY);
    const auto value = readOutput();
    return std::stod(value);
}

void ArduinoFirmware::setFirstMotorVelocity(double v) {
    string command = std::regex_replace(MOVE_MOTOR_1_COMMAND,
                                 std::regex("#velocity"),
                                 to_string(v));
    serial.writeString(command.c_str());
}

string ArduinoFirmware::readOutput() {
    char buffer[15];
    serial.readString(buffer, '\n', 14, 10);
    std::string value(buffer);
    value.erase(remove(value.begin(), value.end(), '\r'), value.end());
    value.erase(remove(value.begin(), value.end(), '\n'), value.end());
    return value;
}

void ArduinoFirmware::writeCommand(int commandNumber) {
    const string command = regex_replace(QUERY_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));

    serial.writeString(command.c_str());
}
