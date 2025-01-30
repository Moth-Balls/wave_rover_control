#include "uart.hpp"
#include <iostream>

UART::UART()
{
    try
    {
        serial_port.Open(UART_PORT);
        serial_port.SetBaudRate(BAUD_RATE);
        serial_port.SetCharacterSize(CHAR_SIZE);
        serial_port.SetStopBits(STOP_BITS);
        serial_port.SetParity(PARITY); 

    }
    catch(const LibSerial::OpenFailed& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void UART::send_data(const std::string &data)
{
    if(serial_port.IsOpen())
    {
        serial_port.Write(data);
    } else {
        std::cerr << "UART is not open" << '\n';
    }  
}

std::string UART::read_data()
{
    std::string data;
    if(serial_port.IsOpen())
    {
        serial_port.Read(data);
        return data;
    }
    else
    {
        std::cerr << "UART is not open" << '\n';
        return "";
    }
}