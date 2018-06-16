#include <iostream>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
using namespace std;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

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

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void sendData(char *send, uint32_t ID, bool isRemote, int numOfData, uint8_t data[])
{

    uint32_t temp;
    int remote_mode = 6;
    int data_mode = 4;

    send[0] = 'A';
    send[1] = 'T';

    if (isRemote)
        temp = ((ID) << 3 | remote_mode);
    else
        temp = ((ID) << 3 | data_mode);
    //    memcpy(&send[2],&temp,4);

    char a = (temp);
    char b = (temp >> 8);
    char c = (temp >> 16);
    char d = (temp >> 24);
    send[2] = d;
    send[3] = c;
    send[4] = b;
    send[5] = a;

    char num = (char)(numOfData);
    send[6] = num;

    int index = 7;
    for (int i = 0; i < numOfData; i++)
    {
        send[index] = data[i];
        index++;
    }
    send[index] = '\r';
    send[index + 1] = '\n';
}

void send_thread(int fd)
{
    do
    {
        char buf[100];

        int rdlen;
        uint32_t testID = 0b01010101010101010101010101010;
        uint8_t test_data[] = {0b1, 0b110, 0b111};
        char send_str[12];

        sendData(send_str, testID, false, 3, test_data);

        int numOfChar = sizeof(send_str);
        int wlen = write(fd, send_str, numOfChar);
        if (wlen != numOfChar)
        {
            printf("Error from write: %d, %d\n", wlen, errno);
        }
        usleep(500000);//millisecond
    } while (1);
}

void receive_thread(int fd)
{
    do
    {
        char buf[100];
        int rdlen = read(fd, buf, sizeof(buf) - 1); //\0
        if (rdlen > 0)
        {
#ifdef DISPLAY_STRING
            buf[rdlen] = 0;
            printf("Read %d: %s", rdlen, buf);
#else /* display hex */
            char *p;
            printf("Read %d:", rdlen);
            for (p = buf; rdlen-- > 0; p++)
                printf(" 0x%x", *p);
            printf("\n");
#endif
        }
        else if (rdlen < 0)
        {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
        usleep(500000);
    } while (1);
    /* repeat read to get full message */
}

int main()
{
    const char *portname = "/dev/ttyUSB1";
    int fd;
    int wlen;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);

    std::thread task1(send_thread, fd);
    std::thread task2(receive_thread, fd);
    
    task1.join();
    task2.join();
}