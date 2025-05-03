#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <memory>
#include <string>
#include <serial/serial.h>

class SerialPort
{
public:
    SerialPort(const std::string &port_name, int baudrate, bool verbose = false);

    ~SerialPort();

    // Initialize and open the serial port
    bool init();

    bool open();       // ✅ 加上這個
    bool isOpen();     // ✅ 加上這個
    // Close the serial port
    void close();

    // Write a raw string (e.g., formatted command) to serial
    bool writeRaw(const std::string &message);

    // Read a line from the serial port
    std::string readRaw();

    // Optional debug output
    void setVerbose(bool verbose);

private:
    std::string port_name_;
    int baud_rate_;
    bool verbose_;

    std::unique_ptr<serial::Serial> serial_;
};

#endif // SERIAL_PORT_HPP
