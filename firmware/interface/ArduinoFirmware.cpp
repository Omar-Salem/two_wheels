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

MotorsOdom ArduinoFirmware::getMotorsOdom() {
    writeQueryCommand(GET_MOTORS_ODOM);
    const auto value = readOutput();
    //v1,p1,v2,p2
    vector<string> values = split(value);
    return MotorsOdom(stod(values.at(0)),
                      stod(values.at(1)),
                      stod(values.at(2)),
                      stod(values.at(3)));
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

vector<string> ArduinoFirmware::split(const string &str) {
    vector<string> strings;
    char separator = ',';
    int startIndex = 0, endIndex = 0;
    for (int i = 0; i <= str.size(); i++) {

        // If we reached the end of the word or the end of the input.
        if (str[i] == separator || i == str.size()) {
            endIndex = i;
            string temp;
            temp.append(str, startIndex, endIndex - startIndex);
            strings.push_back(temp);
            startIndex = endIndex + 1;
        }
    }
    return strings;
}

