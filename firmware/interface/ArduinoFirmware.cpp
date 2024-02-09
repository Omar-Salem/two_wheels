//
// Created by omar on 2/8/24.
//


#include "ArduinoFirmware.h"

void ArduinoFirmware::configure() {
    serial = new SerialOps(SERIAL_PORT, BAUD);
}

double ArduinoFirmware::getFirstMotorPosition() {
    return readCommandUtil(GET_MOTOR_1_POSITION);
}

double ArduinoFirmware::getFirstMotorVelocity() {
    return readCommandUtil(GET_MOTOR_1_VELOCITY);
}

void ArduinoFirmware::setFirstMotorVelocity(double v) {
    string command = std::regex_replace(WRITE_COMMAND_TEMPLATE,
                                        std::regex("#command"),
                                        to_string(MOVE_MOTOR_1));
    command = std::regex_replace(command,
                                 std::regex("#velocity"),
                                 to_string(v));
    sendCommand(command);
}


void ArduinoFirmware::sendCommand(const string &command) {
    serial->write(command);
}

double ArduinoFirmware::readCommand() {
    const string buffer = serial->read();
    if (buffer.empty()) {
        throw invalid_argument("SerialBufferEmpty");
    }
    json data = json::parse(buffer);
    const auto value = data["value"];
    return value.get<nlohmann::json::number_float_t>();
}

double ArduinoFirmware::readCommandUtil(int commandNumber) {
    const string command = regex_replace(READ_COMMAND_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));
    sendCommand(command);
    return readCommand();
}
