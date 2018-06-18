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

SerialCan *createSerialCom(std::string path,
                           speed_t baudRate)
{
    int fd;

    fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n",
               path.c_str(),
               strerror(errno));
        return NULL;
    }
    else
    {
        struct termios tty;

        if (tcgetattr(fd, &tty) < 0)
        {
            printf("Error from tcgetattr: %s\n", strerror(errno));
            return NULL;
        }

        cfsetospeed(&tty, baudRate);
        cfsetispeed(&tty, baudRate);

        tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      /* 8-bit characters */
        tty.c_cflag &= ~PARENB;  /* no parity bit */
        tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

        /* setup for non-canonical mode */
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        //blocking read until at least one byte availables
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            printf("Error from tcsetattr: %s\n", strerror(errno));
            return NULL;
        }

        return new SerialCan(fd);
    }
};

SerialCan::SerialCan(int fd)
    : fd(fd),
      startPos(0),
      endPos(0),
      shouldRead(false)
{
    memset(buf, 0, sizeof(buf));
};

bool SerialCan::startReadThd()
{
    if (shouldRead)
    {
        return false;
    }
    else
    {
        shouldRead = true;
        readThd = new std::thread(&SerialCan::readThdFunc, this);
        return true;
    }
};

bool SerialCan::stopReadThd()
{
    if (shouldRead)
    {
        shouldRead = false;
        readThd->join();
        delete readThd;
        readThd = NULL;
        return true;
    }
    else
    {
        return false;
    }
};

bool SerialCan::sendCanMsg(uint32_t id,
                           bool extended,
                           bool remote,
                           uint8_t numBytes,
                           const uint8_t data[])
{
    //AT [4byte id + type] [1 byte DLC] [up to 8 byte data] \r\n

    unsigned char msg[18] = {0};

    msg[0] = 'A';
    msg[1] = 'T';

    if (extended)
    {
        //very strange arrangement here for the module, little endian by bytes, but send MSByte first
        //29 bit eid
        msg[2] = id >> 21;
        msg[3] = id >> 13;
        msg[4] = id >> 5;
        msg[5] = id << 3;
        msg[5] |= EXTENTED_MASK;
    }
    else
    {
        //11 bit sid
        msg[2] = id >> 3;
        msg[3] = id << 5;
    }

    size_t msgSize;

    if (remote)
    {
        msg[5] |= REMOTE_MASK;
        msg[6] = 0;
        msg[7] = '\r';
        msg[8] = '\n';
        msgSize = 9;
    }
    else
    {
        msg[6] = numBytes;
        memcpy(&msg[7], data, numBytes);
        msg[7 + numBytes] = '\r';
        msg[7 + numBytes + 1] = '\n';
        msgSize = 9 + numBytes;
    }

    int wlen = write(fd, msg, msgSize);
    if (wlen != msgSize)
    {
        printf("Error from write: %d, %d\n", wlen, errno);
        return false;
    }
    else
        return true;
};

SerialCan::~SerialCan()
{
    stopReadThd();
    delete readThd;
    close(fd);
};

SerialCan::SerialCan(int fd)
    : fd(fd),
      readThd(NULL)
{
    memset(buf, 0, sizeof(buf));
};

void SerialCan::receiveCB(uint32_t id,
                          bool extended,
                          bool remote,
                          uint8_t numBytes,
                          const uint8_t data[]){

};

void SerialCan::readThdFunc()
{
    while (shouldRead)
    {
        int readSize;
        //always reserve on byte between in writePos to know the three states:
        //  startPos=writePos => no data
        //  writePos 1 byte behine startPos => buffer full
        //  else => some data in the buffer
        if (startPos == 0)
            readSize = read(fd, &buf[endPos], (BUFFER_SIZE - endPos - 1));
        if (startPos <= endPos)
            readSize = read(fd, &buf[endPos], (BUFFER_SIZE - endPos));
        else
            readSize = read(fd, &buf[endPos], (startPos - endPos - 1));

        endPos += readSize;
        if (endPos == BUFFER_SIZE)
        {
            endPos = 0;
        }

        if ((endPos == startPos - 1) ||
            (startPos == 0 && endPos == BUFFER_SIZE - 1))
        {
            printf("BUFFER FULL! poping something");
        }

        bool popedMsg;

        do
        {
            popedMsg = false;
            if (!foundAT)
            {
                //AT not found, serach for it
                while (ATseekPos != endPos)
                {
                    if (buf[ATseekPos] == 'A')
                    {
                        if (buf[(ATseekPos + 1) % BUFFER_SIZE] == 'T')
                        {
                            //found AT
                            foundAT = true;
                            EOLseekPos = (ATseekPos + 2) % BUFFER_SIZE;
                        }
                        else if ((ATseekPos + 1) % BUFFER_SIZE == endPos)
                        {
                            //things after 'A' not read, check again next time
                            break;
                        }
                    }

                    //"AT" not here
                    ATseekPos = (ATseekPos + 1) % BUFFER_SIZE;
                }
            }

            if (foundAT)
            {
                //search for \r \n and perform receive callback if data valid
                while (EOLseekPos != endPos)
                {
                    if (buf[EOLseekPos] == '\n' ||
                        buf[EOLseekPos] == '\r')
                    {
                        //seems received something
                        //TODO:
                        foundAT = false;
                        startPos = ATseekPos = EOLseekPos;
                        popedMsg = true;
                    }

                    //end of line not here
                    EOLseekPos = (EOLseekPos + 1) % BUFFER_SIZE;
                }
            }
        } while (popedMsg);
    }
};