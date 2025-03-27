#include <iostream>
#include "MBUtils.h"
// #include "SockNinja.h"
#include <unistd.h>
#include "BlueBoatBridge.hpp"

using namespace std;

int main(int argc, char *argv[])
{

    if (argc != 3)
        return (1);
    string ninja_type = argv[1];
    string user_greet = argv[2];
    SockNinja ninja(ninja_type, 29500);
    ninja.setMsgFormatVerbatim();
    while (!ninja.isConnected())
    {
        usleep(5000);
        cout << "Waiting to connect..." << endl;
        ninja.setupConnection();
        ninja.sendSockMessage(user_greet);
        cout << "Reply:" << ninja.getSockMessage() << endl;
    }
}