#pragma once

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <stdint.h>
#include <iostream>
#include <thread>
#include <chrono>

#define SERIAL_PORT "/dev/ttyS0"
#define BAUD_RATE LibSerial::BaudRate::BAUD_115200
#define CHAR_SIZE LibSerial::CharacterSize::CHAR_SIZE_8
#define STOP_BITS LibSerial::StopBits::STOP_BITS_1
#define PARITY LibSerial::Parity::PARITY_NONE

using namespace LibSerial;
using namespace nlohmann;

class SerialDevice
{
public:
    static SerialDevice& getInstance()
    {
        static SerialDevice instance;
        return instance;
    }

    void send_data(const std::string &data);
    std::string read_data();

private:
    SerialDevice();
    SerialStream serialStream; // Using SerialStream

    SerialDevice(const SerialDevice&) = delete;
    SerialDevice& operator=(const SerialDevice&) = delete;
};
