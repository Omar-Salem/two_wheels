//
// Created by omar on 2/8/24.
//


#include "ArduinoFirmware.h"

void ArduinoFirmware::configure() {
    auto errorOpening = serial.openDevice(SERIAL_PORT, BAUD);
    if (errorOpening != 1) {
        throw invalid_argument("SerialDeviceException: Could not open device");
    }
    bool serialReady = false;
    unsigned int retryCount = 5;
    while (!serialReady && retryCount > 0) {
        try {
            getFirstMotorPosition();
            serialReady = true;
        } catch (...) {
            --retryCount;
            std::this_thread::sleep_for(500ms);
        }
    }
    if (!serialReady) {
        throw invalid_argument("Could not configure");
    }
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

    printf("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC: %s\n", command.c_str());
    int res = serial.writeString(command.c_str());
    if (res != 1) {
        throw invalid_argument("Could not write to serial");
    }
}

double ArduinoFirmware::readCommand() {
    char buffer[1000];
    int result = serial.readString(buffer, '\n', 2000, 500);
    if (result <= 0) {
        string errorMsg = "ReadException:";
        switch (result) {
            case 0:
                errorMsg += "timeout reached";
                break;
            case -1:
                errorMsg += "error while setting the Timeout";
                break;
            case -2:
                errorMsg += "error while reading the character";
                break;
            case -3:
                errorMsg += "MaxNbBytes is reached";
                break;
            default:
                errorMsg += to_string(result);
                break;
        }
        throw invalid_argument(errorMsg);
    }
    try {
        json data = json::parse(buffer);
    } catch (...) {
        printf("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO String read: %s\n", buffer);
    }
    // {\"value\":5.9}
//    const auto value = data["value"];
//    printf("JSON read: %s\n", value.dump().c_str());
//    value.
//    return value.get<nlohmann::json::number_float_t>();
    return 0;
}

double ArduinoFirmware::readCommandUtil(int commandNumber) {
    const string command = regex_replace(READ_COMMAND_TEMPLATE,
                                         regex("#command"),
                                         to_string(commandNumber));
    sendCommand(command);
    return readCommand();
}
