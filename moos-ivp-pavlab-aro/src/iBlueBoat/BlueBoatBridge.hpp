#pragma once
#include "SockNinja.h"

class BlueBoatBridge
{
public:
    BlueBoatBridge() {

    };

    ~BlueBoatBridge() {

    };

    SockNinja m_sockNinja;

    // setter methods

    bool setIPAddr(std::string ipAddress)
    {
        return m_sockNinja.setIPAddr(ipAddress);
    }

    bool setPortNumber(int port)
    {
        return m_sockNinja.setPortNumber(port);
    }

    bool setCommsType(std::string commType)
    {
        return m_sockNinja.setCommsType(commType);
    }

    bool setIgnoreCheckSum(bool ignore)
    {
        return m_sockNinja.setIgnoreCheckSum(ignore);
    }

    std::string getState()
    {
        return m_sockNinja.getState();
    }

    bool sendSockMessage(std::string message)
    {
        return m_sockNinja.sendSockMessage(message);
    }

    void setMsgFormatVerbatim()
    {
        m_sockNinja.setMsgFormatVerbatim();
    }

    bool setDesiredMessages(std::list<std::string> desiredMessages)
    {
    std:
        std::string msg = "BBDMS,";
        for (std::list<std::string>::const_iterator it = desiredMessages.begin(); it != desiredMessages.end(); ++it)
        {
            msg += *it + ",";
        }
        msg.pop_back();
        msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";
        return m_sockNinja.sendSockMessage(msg);
    }

    // getter methods

    bool getIgnoreCheckSum()
    {
        return m_sockNinja.getIgnoreCheckSum();
    }

    std::list<std::string> getSockMessages()
    {
        return m_sockNinja.getSockMessages();
    }

    std::list<std::string> getEvents()
    {
        return m_sockNinja.getEvents();
    }

    std::list<std::string> getWarnings()
    {
        return m_sockNinja.getWarnings();
    }

    std::list<std::string> getRetractions()
    {
        return m_sockNinja.getRetractions();
    }

    std::list<std::string> getSummary()
    {
        return m_sockNinja.getSummary();
    }

    // Destructor

    void closeSockFDs()
    {
        m_sockNinja.closeSockFDs();
    }

    // set up connection
    void setupConnection()
    {
        m_sockNinja.setupConnection();
    }

    // Send formatted message
    void sendFormattedMessage(int leftThrust, int rightThrust, int auxServo)
    {
        std::string msg = "BBTMS,";
        msg += std::to_string(leftThrust);
        msg += ",";
        msg += std::to_string(rightThrust);
        msg += ",";
        msg += std::to_string(auxServo);
        msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

        m_sockNinja.sendSockMessage(msg);
    }

    // Disable Functions

    bool disableWarningBadNMEALen()
    {
        m_sockNinja.disableWarningBadNMEALen();
        return true;
    }

    bool disableWarningBadNMEANend()
    {
        m_sockNinja.disableWarningBadNMEANend();
        return true;
    }

    bool disableWarningBadNMEARend()
    {
        m_sockNinja.disableWarningBadNMEARend();
        return true;
    }

    bool disableWarningBadNMEAForm()
    {
        m_sockNinja.disableWarningBadNMEAForm();
        return true;
    }

    bool disableWarningBadNMEAChks()
    {
        m_sockNinja.disableWarningBadNMEAChks();
        return true;
    }

    bool disableWarningBadNMEAKey()
    {
        m_sockNinja.disableWarningBadNMEAKey();
        return true;
    }
};