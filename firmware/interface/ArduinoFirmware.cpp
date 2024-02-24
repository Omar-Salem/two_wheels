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
        writeCommand(PING);
        const auto value = readOutput();
        cout << "PING result:" << value << "**********";
        if (value == "PONG") {
            return;
        }
        std::this_thread::sleep_for(1000ms);
    }
    throw runtime_error("************************ SerialConnectionError");
}

double ArduinoFirmware::getFirstMotorPosition() {
    writeCommand(GET_MOTOR_1_POSITION);
    return readDouble();
}

double ArduinoFirmware::getFirstMotorVelocity() {
    writeCommand(GET_MOTOR_1_VELOCITY);
    return readDouble();
}

void ArduinoFirmware::setFirstMotorVelocity(double v) {
    const string command = std::regex_replace(MOVE_MOTOR_1_COMMAND,
                                        std::regex("#velocity"),
                                        to_string(v));
    serial.writeString(command.c_str());
}


double ArduinoFirmware::getSecondMotorPosition() {
    writeCommand(GET_MOTOR_2_POSITION);
    return readDouble();
}

double ArduinoFirmware::getSecondMotorVelocity() {
    writeCommand(GET_MOTOR_2_VELOCITY);
    return readDouble();
}

void ArduinoFirmware::setSecondMotorVelocity(double v) {
    const string command = std::regex_replace(MOVE_MOTOR_2_COMMAND,
                                        std::regex("#velocity"),
                                        to_string(v));
    serial.writeString(command.c_str());
}

string ArduinoFirmware::readOutput() {
    char buffer[15];
    serial.readString(buffer, '\n', 14, 125);
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

double ArduinoFirmware::readDouble() {
    const auto value = readOutput();
    if (!isNumber(value)) {
        return 0; //probably still reading "PONG"
//        throw invalid_argument("Wrong value:" + value);
    }
    return std::stod(value);
}

bool ArduinoFirmware::isNumber(string s) {
    try {
        std::stod(s);
        return true;
    } catch (...) {
        return false;
    }
}

