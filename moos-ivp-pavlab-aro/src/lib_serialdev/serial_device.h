/*  ----------------------------------------------------------------------------
    Author: Raymond Turrisi
    Circa: August 2023
    Origin: MIT, Cambridge MA
    File: serial_device.h
    Desc: A general purpose serial library which reduces repeated
            setup across devices
    Status: To be tested with within lib_seatrac and iHydroLink
    ---------------------------------------------------------------------------- 
*/  
#pragma once

#include <string>
#include <set>
#include <termios.h>

class SerialDevice {
    public:
        /**
         * @brief Default configuration for a SerialDevice with no configuration
         * 
         */
        SerialDevice();

        /**
         * @brief Proper construction of a SerialDevice object
         * 
         * @param port A string which provides a path to a serial port i.e. /dev/ttyACM0
         * @param baudRate Default 9600 bits/s
         * @param read_buffer_size Default 64 bytes
         */
        SerialDevice(const std::string &port, size_t baudRate, size_t read_buffer_size);
        
        /**
         * @brief Close the serial port and destroy the object
         * 
         */
        ~SerialDevice();

        /**
         * @brief If not setup with the configuration constructor, can set configuration settings before opening the port
         * 
         * @param port A string which provides a path to a serial port i.e. /dev/ttyACM0
         * @param baudRate Default 9600 bits/s
         * @param read_buffer_size Default 64 bytes
         */
        void config(const std::string &port, size_t baudRate, size_t read_buffer_size);

        /**
         * @brief Whether or not the buffer should be flushed after each read
         * 
         */
        void flushAfterRead();

        /**
         * @brief Opens the serial port if the device has been configured
         * 
         * @return int 0 if ok -1 if there was an error in opening the device
         */
        int openSerial();

        /**
         * @brief Reads from the serial port
         * 
         * @param contents A string buffer which the contents will be placed
         * @return int Returns the number of bytes read, or -1 if there was an error
         */
        int readSerial(std::string &contents);

        /**
         * @brief Reads from the serial port, and returns the system level error message. 
         * Can be used like readSerial, where a read is successful the size is 0
         * 
         * 
         * @param contents A string buffer which the contents will be placed
         * @return std::string An error message from strerror(errno)
         */
        std::string readSerialAndReturnErr(std::string &contents);

        /**
         * @brief Writes the string as-is to the serial buffer
         * 
         * @param data Data to be written to the serial device
         * @return int 0 if ok, -1 if there was an error
         */
        int writeSerial(std::string data);

        /**
         * @brief Writes the string as-is to the serial buffer, and returns the system
         * level error message. Can be used like writeSerial, where if a write is successful
         * the length of the message is of size 0
         * 
         * @param data Data to be written to the serial device
         * @return std::string An error message from strerror(errno)
         */
        std::string writeSerialAndReturnErr(std::string data);

        /**
         * @brief Closes the serial port
         * 
         * @return int Returns the error code if there was one
         */
        int closeSerial();

    private:
        void init();
        int m_serial_fd;
        bool m_flush;
        size_t m_baud_rate;
        size_t m_read_buffer_size;
        std::string m_port;
        std::set<int> m_standard_bauds;
        std::set<int> m_reconnect_conditions;
        char *m_read_buffer;
}   ;