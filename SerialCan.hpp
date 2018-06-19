#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

#define BUFFER_SIZE 128

//masks for the 6th byte
#define EXTENTED_MASK 0b100
#define REMOTE_MASK 0b10

class SerialCan
{
public:
  friend SerialCan *createSerialCom(std::string path,
                                    speed_t baudRate);
  bool startReadThd();
  bool stopReadThd();
  bool sendCanMsg(uint32_t id,
                  bool extended,
                  bool remote,
                  uint8_t numBytes,
                  const uint8_t data[]);

  ~SerialCan();

private:
  SerialCan(int fd);

  void receiveCB(uint32_t id,
                 bool extended,
                 bool remote,
                 uint8_t numBytes,
                 const uint8_t data[]);

  void readThdFunc();
  const int fd;

  //queue related
  unsigned char buf[BUFFER_SIZE];
  unsigned int startPos = 0;
  unsigned int endPos = 0;

  bool foundAT = false;
  unsigned int ATseekPos = 0;
  unsigned int EOLseekPos = 0;

  std::thread *readThd;

  volatile bool shouldRead = false;
};

SerialCan *createSerialCom(std::string path,
                           speed_t baudRate = B115200);
