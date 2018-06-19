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
        char data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        comObj->sendCanMsg(1234, true, false, 8, (uint8_t *)data);
        sleep(1);
    }
}
