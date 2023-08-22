#pragma once
#ifndef __USBCAN_BASE_H__
#define __USBCAN_BASE_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <queue>

typedef struct can_frame CanFrame_t;
typedef std::queue<CanFrame_t> CanFrameList_t;

typedef struct canfd_frame CanfdFrame_t;
typedef std::queue<CanfdFrame_t> CanfdFrameList_t;

class UsbCanBase
{
private:

public:
    UsbCanBase(const char* model)
    {
        fprintf(stderr, "usbcan model: %s\n", model);
    }
    ~UsbCanBase(){}

    virtual bool init_usbcan() = 0;
    virtual bool open_usbcan(unsigned int chn, int can_type=0) = 0;
    virtual void close_usbcan() = 0;
    virtual int send_frame(const CanFrame_t& frame, unsigned int chn) = 0;
    virtual int recv_frame(CanFrameList_t& frames, unsigned int chn) = 0;
    virtual int send_frame(const CanfdFrame_t& frame, unsigned int chn) = 0;
    virtual int recv_frame(CanfdFrameList_t& frames, unsigned int chn) = 0;

};

#endif