//
// Created by omar on 2/8/24.
//

#include <iostream>
#include "ArduinoFirmware.h"
#include <algorithm>
#include <regex>

constexpr int MAX_PING_RETRY = 5;

void ArduinoFirmware::connect() {
    char errorOpening = serial.openDevice(SERIAL_PORT, BAUD);
    if (errorOpening != 1) {
        throw runtime_error("************************ SerialConfigurationError");
    }
}

void ArduinoFirmware::disconnect() {
    serial.closeDevice();
}

void ArduinoFirmware::ping() {
    for (int i = 0; i < MAX_PING_RETRY; ++i) {
        writeQueryCommand(PING);
        const auto value = readOutput();
        if (value == "PONG") {
            return;
        }
        std::this_thread::sleep_for(1000ms);
    }
    throw runtime_error("************************ SerialConnectionError");
}

double ArduinoFirmware::getFirstMotorPosition() {
    writeQueryCommand(GET_MOTOR_1_POSITION);
    return readDouble();
}

double ArduinoFirmware::getFirstMotorVelocity() {
    writeQueryCommand(GET_MOTOR_1_VELOCITY);
    return readDouble();
}

void ArduinoFirmware::setMotorsVelocity(double m1, double m2) {
    string command = std::regex_replace(COMMAND_TEMPLATE,
                                        std::regex("#m1"),
                                        to_string(m1));
    command = std::regex_replace(command,
                                 std::regex("#m2"),
                                 to_string(m2));
    int res = serial.writeString(command.c_str());
    if (res != 1) {
        throw runtime_error("************************ Could not write command");
    }
}


double ArduinoFirmware::getSecondMotorPosition() {
    writeQueryCommand(GET_MOTOR_2_POSITION);
    return readDouble();
}

double ArduinoFirmware::getSecondMotorVelocity() {
    writeQueryCommand(GET_MOTOR_2_VELOCITY);
    return readDouble();
}

string ArduinoFirmware::readOutput() {
    char buffer[15];
    serial.readString(buffer, '\n', 14, 125);
    std::string value(buffer);
    value.erase(remove(value.begin(), value.end(), '\r'), value.end());
    value.erase(remove(value.begin(), value.end(), '\n'), value.end());
    return value;
}

void ArduinoFirmware::writeQueryCommand(int commandNumber) {
    const string command = regex_replace(QUERY_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));
    int res = serial.writeString(command.c_str());
    if (res != 1) {
        throw runtime_error("************************ Could not write command");
    }
}

double ArduinoFirmware::readDouble() {
    const auto value = readOutput();
    if (!isNumber(value)) {
        return 0; //probably still reading "PONG"
//        throw invalid_argument("Wrong value:" + value);
    }
    return std::stod(value);
}

bool ArduinoFirmware::isNumber(const string &s) {
    try {
        std::stod(s);
        return true;
    } catch (...) {
        return false;
    }
}

