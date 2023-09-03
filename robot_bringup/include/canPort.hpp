#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <net/if.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "utilty.h"

class canPort
{
private:
    int fd;
public:
    canPort();
    //~canPort();
    CANTxMsg_t rxmsg;
        
    int openPort(const char *dev);
    int writeBuffer(unsigned int id, uint8_t *data, unsigned int len);
    int readBuffer(void);
};