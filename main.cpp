#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include "SerialCan.hpp"

int main(int argc, char **argv)
{
    std::string path = "/dev/ttyUSB";
    SerialCan *comObj = NULL;

    if (argc == 2)
    {
        comObj = createSerialCom(path + argv[1]);
    }
    else
    {
        comObj = createSerialCom(path + '0');
    }
    if (comObj)
    {
        comObj->startReadThd();
    }
    while (1)
    {
        comObj->sendCanMsg(1234, false, true, 0, 0);
        sleep(1);
    }
}
