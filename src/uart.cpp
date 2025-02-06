#include "uart.hpp"

SerialDevice::SerialDevice() : serialStream(SERIAL_PORT)
{
    // SerialStream constructor opens the port.
    // Serial stream parameters are set here.
    serialStream.SetBaudRate(BAUD_RATE);
    serialStream.SetCharacterSize(CHAR_SIZE);
    serialStream.SetStopBits(STOP_BITS);
    serialStream.SetParity(PARITY);
}

void SerialDevice::send_data(const std::string &data)
{
    serialStream.write(data.c_str(), data.size());
    serialStream.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

std::string SerialDevice::read_data()
{
    std::string response;
    while (true) {
        std::getline(serialStream, response);
        if (!response.empty() && response.back() == '\r') {
            response.pop_back();
        }
        
        json imu_data = json::parse(response);
        if (imu_data.contains("T") && imu_data["T"] == 1002) {
            std::cout << response << std::endl;
            return response;
        }
    }
}
