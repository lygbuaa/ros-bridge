#pragma once
#ifndef __USBCAN_WRAPPER_H__
#define __USBCAN_WRAPPER_H__

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
#include "usbcan/controlcan/controlcan.h"
#include "logging_utils.h"

typedef struct can_frame CanFrameClassical_t;
typedef std::queue<CanFrameClassical_t> FrameList_t;

/* chuang-xin-ke-ji CANalyst-II */
class UsbCanClassical
{
private:
    constexpr static unsigned int dev_idx_ = 0;
    constexpr static unsigned int BUF_MAX_ = 1000;
    unsigned int chn;
    VCI_CAN_OBJ* buffer_;

public:
    UsbCanClassical()
    {
        if(!init_usbcan()){
            LOGPF("failed to init usbcan!");
            abort();
        }
    }

    ~UsbCanClassical() 
    {
        close_usbcan();
    }

    bool init_usbcan()
    {
        /* suppose we have at most 5 device */
        VCI_BOARD_INFO infos[5];
        int num = VCI_FindUsbDevice2(infos);
        LOGPF("find %d usbcan device", num);
        /* we only accept 1 device */
        assert(num == 1);

        /* open device 0 */
        VCI_BOARD_INFO info;
        if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1 &&
           VCI_ReadBoardInfo(VCI_USBCAN2, 0, &info) == 1)
        {
            std::string sn(info.str_Serial_Num, 20);
            std::string hw(info.str_hw_Type, 10);
            LOGPF("open device %d success, sn: %s, hw: %s, firmware: %d", dev_idx_, sn.c_str(), hw.c_str(), info.fw_Version);
        }else{
            LOGPF("open device %d error!", dev_idx_);
            return false;
        }
        return true;
    }

    /* open usbcan channel with 500Kbps */
    bool open_usbcan(unsigned int chn)
    {
        VCI_INIT_CONFIG config;
        /* filter none */
        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;
        /* baudrate = 500Kbps */
        config.Timing0 = 0x00;
        config.Timing1 = 0x1C;
        config.Mode = 0;

        if(VCI_InitCAN(VCI_USBCAN2, dev_idx_, chn, &config) != 1){
            LOGPF("init can device %d channel %d error!", dev_idx_, chn);
            VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        }

        if(VCI_StartCAN(VCI_USBCAN2, dev_idx_, chn) != 1){
            LOGPF("start can device %d channel %d error!", dev_idx_, chn);
            VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        }

        LOGPF("start can device %d channel %d, baudrate: 500Kbps", dev_idx_, chn);
        buffer_ = new VCI_CAN_OBJ[BUF_MAX_];

        return true;
    }

    void close_usbcan()
    {
        VCI_ResetCAN(VCI_USBCAN2, dev_idx_, 0);
        VCI_ResetCAN(VCI_USBCAN2, dev_idx_, 1);
        VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        delete[] buffer_;
    }

    int send_frame(const CanFrameClassical_t& frame, unsigned int chn)
    {
        VCI_CAN_OBJ send[1];
        send[0].ID = frame.can_id;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        /* standard frame */
        send[0].ExternFlag = 0;
        send[0].DataLen = frame.can_dlc;

        memcpy(send[0].Data, frame.data, frame.can_dlc);
        // for(int i=0; i<frame.can_dlc; i++){
        //     send[0].Data[i] = frame.data[i];
        // }

        if(VCI_Transmit(VCI_USBCAN2, dev_idx_, chn, send, 1) == 1){
            LOGPF("send frame %x success", frame.can_id);
            return frame.can_dlc;
        }else{
            LOGPF("send frame %x fail", frame.can_id);
        }

        return -1;
    }

    int recv_frame(FrameList_t& frames, unsigned int chn)
    {
        /* WaitTime = 100ms? */
        int reclen = VCI_Receive(VCI_USBCAN2, dev_idx_, chn, buffer_, BUF_MAX_, 100);
        for(int i=0; i<reclen; i++){
            CanFrameClassical_t frame;
            frame.can_id = buffer_[i].ID;
            frame.can_dlc = buffer_[i].DataLen;
            memcpy(frame.data, buffer_[i].Data, frame.can_dlc);
            frames.push(frame);
            // LOGPF("recv canid: 0x%03X, len: %d", frame.can_id, frame.can_dlc);
        }

        return reclen;
    }

};

#endif