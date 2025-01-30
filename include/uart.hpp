#pragma once

#include <libserial/SerialPort.h>
#include <nlohmann/json.hpp>
#include <string>

#define UART_PORT "/dev/ttyS0"
#define BAUD_RATE LibSerial::BaudRate::BAUD_115200
#define CHAR_SIZE LibSerial::CharacterSize::CHAR_SIZE_8
#define STOP_BITS LibSerial::StopBits::STOP_BITS_1
#define PARITY LibSerial::Parity::PARITY_NONE

class UART
{
public:
    UART();
    void send_data(const std::string &data);
    std::string read_data();

private:
    LibSerial::SerialPort serial_port;
};
