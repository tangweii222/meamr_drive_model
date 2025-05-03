#include "meamr_drive_model/serial_port.hpp"
#include <iostream>

SerialPort::SerialPort(const std::string &port_name, int baud_rate, bool verbose)
    : port_name_(port_name), baud_rate_(baud_rate)
{
    serial_ = std::make_unique<serial::Serial>();
}


SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::init()
{
    port_name_ = "/dev/ttyUSB0";
    baud_rate_ = 115200;
    verbose_ = true;
    try
    {
        serial_ = std::make_unique<serial::Serial>(
            port_name_,       // e.g. "/dev/ttyUSB0"
            baud_rate_,       // e.g. 115200
            serial::Timeout::simpleTimeout(1000) // timeout in ms
        );

        if (serial_->isOpen())
        {
            if (verbose_)
                std::cout << " Serial port opened: " << port_name_ << " @ " << baud_rate_ << " baud" << std::endl;
            return true;
        }
        else
        {
            std::cerr << " Failed to open serial port: " << port_name_ << std::endl;
            return false;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << " Serial exception: " << e.what() << std::endl;
        return false;
    }
}

bool SerialPort::open()
{
    try {
        serial_->setPort(port_name_);
        serial_->setBaudrate(baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_->setTimeout(timeout);
        serial_->open();

    } catch (serial::IOException &e) {
        std::cerr << "SerialPort::open() failed: " << e.what() << std::endl;
        return false;
    }

    return serial_ && serial_->isOpen();
}


bool SerialPort::isOpen()
{
    return serial_ && serial_->isOpen();
}

void SerialPort::close()
{
    if (serial_ && serial_->isOpen())
    {
        serial_->close();
        if (verbose_)
            std::cout << "🔌 Serial port closed." << std::endl;
    }
}

bool SerialPort::writeRaw(const std::string &message)
{
    // Test mode
    {
        std::cout << "[Mock Serial Write] " << message << std::endl;
        return true;
    }
    // Real mode
    // if (!serial_ || !serial_->isOpen())
    //     return false;

    // size_t bytes_written = serial_->write(message);
    // return bytes_written == message.size();
}

std::string SerialPort::readRaw()
{
    if (!serial_ || !serial_->isOpen())
        return "";

    return serial_->readline(512, "\n");
}

void SerialPort::setVerbose(bool verbose)
{
    verbose_ = verbose;
}
