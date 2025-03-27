#include <iostream>
#include "MBUtils.h"
// #include "SockNinja.h"
#include <unistd.h>
#include "BlueBoatBridge.hpp"

using namespace std;

void showExampleAndExit()
{
    cout << "Example usage: " << endl;
    cout << "    BlueBoatBridge -i 192.168.1.5 -p 29217 -t client" << endl;
    exit(0);
}

void showHelpAndExit()
{
    cout << "Usage: " << endl;
    cout << "    BlueBoatBridge [-e | --example] [-h | --help] [-i | --ip] [-p | --port] [-t | --type]" << endl;
    cout << "Options: " << endl;
    cout << "    -e, --example    Show example usage" << endl;
    cout << "    -h, --help       Show help" << endl;
    cout << "    -i, --ip         Set IP address" << endl;
    cout << "    -p, --port       Set port number" << endl;
    cout << "    -t, --type       Set communication type (client or server)" << endl;
    exit(0);
}

int main(int argc, char *argv[])
{
    // Set default values
    BlueBoatBridge BlueBoatBridge;
    BlueBoatBridge.setCommsType("client");
    BlueBoatBridge.setIPAddr("192.168.1.5");
    BlueBoatBridge.setPortNumber(29217);

    // BlueBoatBridge.setMsgFormatVerbatim();

    // Parse user arguments
    // parse arguments and set setters from BlueBoatBridge
    for (int i = 1; i < argc; i++)
    {
        string argi = argv[i];
        if ((argi == "-e") || (argi == "--example") || (argi == "-example"))
            showExampleAndExit();
        else if ((argi == "-h") || (argi == "--help") || (argi == "-help"))
            showHelpAndExit();
        else if ((argi == "-i") || (argi == "--ip"))
        {
            i++;
            BlueBoatBridge.setIPAddr(argv[i]);
        }
        else if ((argi == "-p") || (argi == "--port"))
        {
            i++;
            BlueBoatBridge.setPortNumber(stoi(argv[i]));
        }
        else if ((argi == "-t") || (argi == "--type"))
        {
            i++;
            BlueBoatBridge.setCommsType(argv[i]);
        }
    }

    // Initialize socket
    BlueBoatBridge.setupConnection();

    // Check connection?
    if (BlueBoatBridge.getState() == "connected")
    {
        cout << "Connection established" << endl;
    }
    else
    {
        cout << "Connection failed" << endl;
    }

    list<string> desired_messages;
    desired_messages.push_back("RC_CHANNELS");
    desired_messages.push_back("HEARTBEAT");
    desired_messages.push_back("GLOBAL_POSITION_INT");
    desired_messages.push_back("BATTERY_STATUS");
    desired_messages.push_back("ATTITUDE");

    BlueBoatBridge.setDesiredMessages(desired_messages);

    // Some test case for some period of time
    list<string> messages = BlueBoatBridge.getSockMessages();
    for (std::list<std::string>::const_iterator it = messages.begin(); it != messages.end(); ++it)
    {
        cout << "Message: ";
        cout << *it << endl;
    }
    std::cout << std::endl;

    int counter = 0;
    int operand = 0;
    int base_pwm = 1500;
    int limit = 300;
    int step = 10;

    // Enter pseudo control loop and test cases
    while (true)
    {
        if (counter <= limit && operand == 0)
        {
            base_pwm += counter;
            counter += step;

            if (counter >= limit)
            {
                operand = 1;
                counter = 0;
                base_pwm = 1500;
            }
        }
        else if (counter <= limit && operand == 1)
        {
            base_pwm -= counter;
            counter += step;

            if (counter >= limit)
            {
                operand = 0;
                counter = 0;
                base_pwm = 1500;
            }
        }

        BlueBoatBridge.sendFormattedMessage(1500, 1500, base_pwm);

        messages = BlueBoatBridge.getSockMessages();
        for (std::list<std::string>::const_iterator it = messages.begin(); it != messages.end(); ++it)
        {
            cout << "Message: ";
            cout << *it << endl;
        }

        list<string> warnings = BlueBoatBridge.getWarnings();
        for (std::list<std::string>::const_iterator it = warnings.begin(); it != warnings.end(); ++it)
        {
            cout << "Warning: ";
            cout << *it << endl;
        }
        sleep(1);
    }

    // Close connection
    BlueBoatBridge.closeSockFDs();
    cout << "Connection closed" << endl;

    // Monitor state information and print it out

    // On shutdown, verify that the bridge is closed properly

    // if (argc != 3)
    //     return (1);
    // string ninja_type = argv[1];
    // string user_greet = argv[2];
    // SockNinja ninja(ninja_type, 29500);
    // ninja.setMsgFormatVerbatim();
    // while (!ninja.isConnected())
    // {
    //     usleep(5000);
    //     cout << "Waiting to connect..." << endl;
    //     ninja.setupConnection();
    //     ninja.sendSockMessage(user_greet);
    //     cout << "Reply:" << ninja.getSockMessage() << endl;
    // }
}
