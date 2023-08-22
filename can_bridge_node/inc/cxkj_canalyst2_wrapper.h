#pragma once
#ifndef __CXKJ_CANALYST2_H__
#define __CXKJ_CANALYST2_H__

#include "usbcan_base.h"
#include "usbcan/controlcan/controlcan.h"
#include "logging_utils.h"


/* chuang-xin-ke-ji CANalyst-II */
class UsbCanClassicCxkj : public UsbCanBase
{
private:
    constexpr static unsigned int dev_idx_ = 0;
    constexpr static unsigned int BUF_MAX_ = 10;
    unsigned int chn;
    /* one buffer for each channel */
    VCI_CAN_OBJ* buffer_[2];

public:
    UsbCanClassicCxkj() : UsbCanBase("UsbCanClassicCxkj")
    {
        // if(!init_usbcan()){
        //     LOGPF("failed to init usbcan!");
        //     abort();
        // }
    }

    ~UsbCanClassicCxkj() 
    {
        // close_usbcan();
    }

    virtual bool init_usbcan() override
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
            return true;
        }else{
            LOGPF("open device %d error!", dev_idx_);
            return false;
        }
    }

    /* open usbcan channel with 500Kbps */
    virtual bool open_usbcan(unsigned int chn, int can_type=0) override
    {
        if(chn > 1){
            LOGPF("channel %d overflow!", chn);
            return false;
        }
        VCI_INIT_CONFIG config;
        /* filter none */
        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;
        /** 
         * baudrate = 500Kbps 
         * calc by USB_CAN TOOL
         */
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
        buffer_[chn] = new VCI_CAN_OBJ[BUF_MAX_];

        return true;
    }

    virtual void close_usbcan() override
    {
        VCI_ResetCAN(VCI_USBCAN2, dev_idx_, 0);
        VCI_ResetCAN(VCI_USBCAN2, dev_idx_, 1);
        VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        delete[] buffer_[0];
        delete[] buffer_[1];
    }

    virtual int send_frame(const CanFrame_t& frame, unsigned int chn) override
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

        if(VCI_Transmit(VCI_USBCAN2, dev_idx_, chn, send, 1) == 1)
        {
            LOGPF("send frame %x success", frame.can_id);
            return frame.can_dlc;
        }
        else
        {
            LOGPF("send frame %x fail", frame.can_id);
        }

        return -1;
    }

    virtual int recv_frame(CanFrameList_t& frames, unsigned int chn) override
    {
        /* WaitTime = 100ms? */
        int reclen = VCI_Receive(VCI_USBCAN2, dev_idx_, chn, buffer_[chn], 1, 20);
        for(int i=0; i<reclen; i++){
            CanFrame_t frame;
            frame.can_id = buffer_[chn][i].ID;
            frame.can_dlc = buffer_[chn][i].DataLen;
            memcpy(frame.data, buffer_[chn][i].Data, frame.can_dlc);
            frames.push(frame);
            // LOGPF("recv canid: 0x%03X, len: %d", frame.can_id, frame.can_dlc);
        }

        return reclen;
    }

    virtual int send_frame(const CanfdFrame_t& frame, unsigned int chn) override
    {
        return -1;
    }

    virtual int recv_frame(CanfdFrameList_t& frames, unsigned int chn) override
    {
        return -1;
    }

};

#endif