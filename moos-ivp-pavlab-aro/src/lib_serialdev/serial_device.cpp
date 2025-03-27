/*  ----------------------------------------------------------------------------
    Author: Raymond Turrisi
    Circa: August 2023
    Origin: MIT, Cambridge MA
    File: serial_device.cpp
    ---------------------------------------------------------------------------- 
*/  
#include "serial_device.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <cerrno>
#include <cstring>
#include <termios.h>

using namespace std;

SerialDevice::SerialDevice()
{
    init();
}

void SerialDevice::config(const std::string &port, size_t baudRate = B9600, size_t read_buffer_size = 64)
{
    m_port = port;
    if (!m_standard_bauds.count(baudRate))
        fprintf(stderr, "<serial_device.cpp> Warning: Non-standard baud rate selected\n");
    m_baud_rate = baudRate;
    m_read_buffer_size = read_buffer_size;
    if(m_read_buffer != nullptr) 
        delete[] m_read_buffer;
    m_read_buffer = new char[read_buffer_size];
}

SerialDevice::SerialDevice(const std::string &port, size_t baudRate = B9600, size_t read_buffer_size = 64)
{
    config(port, baudRate);
}

void SerialDevice::init()
{
    m_serial_fd = -2;
    m_baud_rate = B9600;
    m_read_buffer_size = 64;
    m_read_buffer = nullptr;
    m_flush = false;

    m_standard_bauds.insert(B9600);
    m_standard_bauds.insert(B19200);
    m_standard_bauds.insert(B38400);
    m_standard_bauds.insert(B57600);
    m_standard_bauds.insert(B115200);

    m_reconnect_conditions.insert(EBUSY);
    m_reconnect_conditions.insert(EINTR);
    m_reconnect_conditions.insert(ETIMEDOUT);
    m_reconnect_conditions.insert(EAGAIN);
    m_reconnect_conditions.insert(EWOULDBLOCK);
}

SerialDevice::~SerialDevice()
{
    closeSerial();
    delete[] m_read_buffer;
}

void SerialDevice::flushAfterRead() { m_flush = true; }

int SerialDevice::openSerial()
{
    m_serial_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_serial_fd == -1)
    {
        fprintf(stderr, "<serial_device.cpp> ERR: Error opening device: [%s]\n", strerror(errno));
        for (int i = 1; i < 6; i++)
        {
            if (m_reconnect_conditions.count(errno))
            {
                fprintf(stderr, "\t [%d] Trying to reconnect\n", i);
                // Wait for 200 milliseconds to try to reconnect, maybe waste a maximum of 1 second
                usleep(200000);
                m_serial_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            }
            else
            {
                break;
            }
        }
        return m_serial_fd;
    }

    struct termios options;

    tcgetattr(m_serial_fd, &options); // Get the current options for the port

    cfsetispeed(&options, m_baud_rate);
    cfsetospeed(&options, m_baud_rate);

    options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    return m_serial_fd;
}

int SerialDevice::readSerial(std::string &contents)
{
    int n = 0;
    int c_count = 0;
    string msg_queue;
    contents.clear();
    memset(m_read_buffer, '\0', m_read_buffer_size);
    n = read(m_serial_fd, m_read_buffer, m_read_buffer_size);
    if (n > 0)
    {
        contents = string(m_read_buffer);
    }
    if (m_flush)
        tcflush(m_serial_fd, TCIOFLUSH);

    // Returns an error if there was one, or the total character count from what was in the buffer
    return n;
}

std::string SerialDevice::readSerialAndReturnErr(std::string &contents)
{
    if (readSerial(contents) < 0)
    {
        fprintf(stderr, "<serial_device.cpp> ERR: Error reading from serial\n");
        return std::string(strerror(errno));
    }
    else
    {
        return "";
    }
}

int SerialDevice::writeSerial(std::string data)
{
    int n = write(m_serial_fd, data.c_str(), data.size());
    return n;
}

std::string SerialDevice::writeSerialAndReturnErr(std::string data)
{
    if (writeSerial(data) < 0)
    {
        return std::string(strerror(errno));
    }
    else
    {
        return "";
    }
}

int SerialDevice::closeSerial()
{
    int condition = close(m_serial_fd);
    if(condition) {
        switch (errno)
        {
        case EINTR:
         {
            fprintf(stderr, "<serial_device.cpp> ERR: [1] Closing interrupted - trying again\n");
            usleep(200000);
            for(int i = 2; i < 6; i++) {
                condition = close(m_serial_fd);
                if(condition) break;
                fprintf(stderr, "<serial_device.cpp> ERR: [%d] Closing interrupted - trying again\n", i);
                usleep(200000);
            }
            break;
         }
        case EBADF: 
        {
            fprintf(stderr, "<serial_device.cpp> ERR: Unable to close serial port - bad file descriptor\n");
            break;
        }
        case EIO: 
        {
            fprintf(stderr, "<serial_device.cpp> ERR: Unable to close serial port [%s]\n", strerror(errno));
            break;
        }
        default: {
            fprintf(stderr, "<serial_device.cpp> ERR: [%s]\n", strerror(errno));
            break;
        }
            
        }
    }
    return condition;
}
