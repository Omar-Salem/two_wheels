//
// Created by omar on 2/8/24.
//


#include <iostream>
#include "ArduinoFirmware.h"

void ArduinoFirmware::configure() {
    serial = make_unique<SerialOps>(SERIAL_PORT, BAUD);
}

void ArduinoFirmware::ping() {
    const auto value = readJsonValue(PING);
    if (value != "PONG") {
        throw runtime_error("************************ SerialConnectionError");
    }
}

double ArduinoFirmware::getFirstMotorPosition() {
    const auto value = readJsonValue(GET_MOTOR_1_POSITION);
    return std::stod(value);
}

double ArduinoFirmware::getFirstMotorVelocity() {
    const auto value = readJsonValue(GET_MOTOR_1_VELOCITY);
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

string ArduinoFirmware::readJsonValue(int commandNumber) {
    const string command = regex_replace(READ_COMMAND_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));

    const string buffer = serial->read(command);
    cout << "*************************** buffer" << buffer << endl;
    if (buffer.empty()) {
        throw runtime_error("************************ SerialBufferEmpty");
    }
    const auto data = json::parse(buffer);
    if (data.is_null()) {
        throw runtime_error("************************ SerialReadNull");
    }
    const auto &value = data["value"];
    if (value.is_null()) {
        throw runtime_error("************************ SerialNullValue");
    }
    return value.get<nlohmann::json::string_t>();
}
