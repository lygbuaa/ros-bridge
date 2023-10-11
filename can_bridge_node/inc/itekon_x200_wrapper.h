#pragma once
#ifndef __ITEKON_CANFD_H__
#define __ITEKON_CANFD_H__

#include "logging_utils.h"
#include "usbcan_base.h"
#include "usbcan/itekon_canfd_linux/lib/iTekCANFD.h"

/* itekon-canfd-x200 */
class UsbCanFDItekon : public UsbCanBase
{
private:
    /** arbitration baudrate */
    static constexpr uint32_t Arbitrat_Rate5K = 0x01F31302;
    static constexpr uint32_t Arbitrat_Rate10K = 0x00F91302;
    static constexpr uint32_t Arbitrat_Rate20K = 0x007C1302;
    static constexpr uint32_t Arbitrat_Rate40K = 0x00630A02;
    static constexpr uint32_t Arbitrat_Rate50K = 0x00311302;
    static constexpr uint32_t Arbitrat_Rate80K = 0x001D1204;
    static constexpr uint32_t Arbitrat_Rate100K = 0x00181302;
    static constexpr uint32_t Arbitrat_Rate125K = 0x00131302;
    static constexpr uint32_t Arbitrat_Rate200K = 0x00130A02;
    static constexpr uint32_t Arbitrat_Rate250K = 0x00091302;
    static constexpr uint32_t Arbitrat_Rate400K = 0x00090A02;
    static constexpr uint32_t Arbitrat_Rate500K = 0x00070A02;
    static constexpr uint32_t Arbitrat_Rate800K = 0x00021006;
    static constexpr uint32_t Arbitrat_Rate1000K = 0x00040702;

    /** data baudrate */
    static constexpr uint32_t Data_Rate100K = 0x001D0E30;
    static constexpr uint32_t Data_Rate125K = 0x001F0A20;
    static constexpr uint32_t Data_Rate200K = 0x00130A20;
    static constexpr uint32_t Data_Rate250K = 0x000F0A20;
    static constexpr uint32_t Data_Rate400K = 0x00090A20;
    static constexpr uint32_t Data_Rate500K = 0x00090A20;
    static constexpr uint32_t Data_Rate800K = 0x00040A20;
    static constexpr uint32_t Data_Rate1M = 0x00040720;
    static constexpr uint32_t Data_Rate2M = 0x00010A20;
    static constexpr uint32_t Data_Rate3M = 0x00000D40;
    static constexpr uint32_t Data_Rate4M = 0x00000A20;
    static constexpr uint32_t Data_Rate5M = 0x00000720;
    static constexpr uint32_t Data_Rate6M = 0x00010200;

    static constexpr unsigned int dev_idx_ = 0;
    static constexpr unsigned int BUF_MAX_ = 10;

    DEVICE_HANDLE dev_handle_ = nullptr;
    CHANNEL_HANDLE chn_handle_[2] = {nullptr};
    iTek_CANFD_DEVICE_INFO dev_info_;
    iTek_CANFD_Receive_Data* buffer_[2];
    // iTek_CANFD_CHANNEL_INIT_CONFIG chn_config_;

public:
    UsbCanFDItekon() : UsbCanBase("UsbCanFDItekon")
    {
    }
    ~UsbCanFDItekon() {}


    virtual bool init_usbcan() override
    {
        if (!iTek_UsbInit()) {
            LOGPF("iTek_UsbInit failed!");
            close_usbcan();
            return false;
        }
        /**
         * Device_Type: 1-x100, 2-x200
         * device_index: 0
         */
        dev_handle_ = iTek_OpenDevice(2, dev_idx_, 0);
        if (nullptr == dev_handle_) {
            LOGPF("iTek_OpenDevice failed!");
            close_usbcan();
            return false;
        }

        iTek_GetDeviceInfo(dev_handle_, &dev_info_);
        LOGPF("iTek_GetDeviceInfo success:");
        LOGPF("hardware version: V%d.%d.%d", dev_info_.hw_Version[0], dev_info_.hw_Version[1], dev_info_.hw_Version[2]);
        LOGPF("firmware version: V%d.%d.%d", dev_info_.fw_Version[0], dev_info_.fw_Version[1], dev_info_.fw_Version[2]);
        LOGPF("channel count: %d", dev_info_.can_Num);
        LOGPF("serial number: %s", dev_info_.str_Serial_Num);
        LOGPF("descriptor: %s", dev_info_.str_hw_Type);
        return true;
    }

    virtual bool open_usbcan(unsigned int chn, int can_type=0) override
    {
        if(chn > 1){
            LOGPF("channel %d overflow!", chn);
            return false;
        }
        iTek_CANFD_CHANNEL_INIT_CONFIG chn_config;
        memset(&chn_config, 0, sizeof(iTek_CANFD_CHANNEL_INIT_CONFIG));

        /** 0:can classic, 1:canfd */
        chn_config.can_type = can_type;
        chn_config.workMode = 0;

        if(can_type == 0)
        {
            LOGPF("*** channel[%d] using CAN Classic mode! ***", chn);
            /** set baudrate 500K*/
            chn_config.abit_timing = Arbitrat_Rate500K;
            chn_config.dbit_timing = Arbitrat_Rate500K;
        }
        else if(can_type == 1)
        {
            LOGPF("*** channel[%d] using CAN FD mode! ***", chn);
            /** set baudrate 500K + 2M*/
            chn_config.abit_timing = Arbitrat_Rate500K;
            chn_config.dbit_timing = Data_Rate2M;

            chn_config.CANFDStandard = 0;
            /** 1:enable speed up */
            chn_config.CANFDSpeedup = 1;
        }
        else
        {
            LOGPF("*** channel[%d] invalid can type! ***", chn);
            return false;
        }

        /** set filter to recv ids between 0x0~0xfff */
        chn_config.Standard.num = 1;
        chn_config.Standard.filterDataStandard->ID1 = 0x0;
        chn_config.Standard.filterDataStandard->ID2 = 0xfff;
        chn_config.Standard.filterDataStandard->frameType = 0;
        chn_config.Standard.filterDataStandard->filterType = 0;
        /** set filter to recv ids between 0x0~0x1FFFFFFF */
        chn_config.Extend.num = 1;
        chn_config.Extend.filterDataExtend->ID1 = 0x0;
        chn_config.Extend.filterDataExtend->ID2 = 0x1FFFFFFF;
        chn_config.Extend.filterDataExtend->frameType = 1;
        chn_config.Extend.filterDataExtend->filterType = 0;

        chn_handle_[chn] = iTek_InitCan(dev_handle_, chn, &chn_config);

        if (chn_handle_[chn] == nullptr) {
            LOGPF("iTek_InitCan chn[%d] failed!", chn);
            close_usbcan();
            return false;
        }

        if (!iTek_StartCAN(chn_handle_[chn])) {
            LOGPF("iTek_StartCAN chn[%d] failed!", chn);
            close_usbcan();
            return false;
        }

        buffer_[chn] = new iTek_CANFD_Receive_Data[BUF_MAX_];
        LOGPF("iTek_StartCAN chn[%d] is open, ready to send/recv.", chn);

        return true;
    }

    virtual void close_usbcan() override
    {
        for(int i=0; i<2; i++)
        {
            if(chn_handle_[i] != nullptr)
            {
                iTek_RestCAN(chn_handle_[i]);
            }
            if(buffer_[i] != nullptr)
            {
                delete[] buffer_[i];
            }
        }
        if(dev_handle_ != nullptr)
        {
            iTek_CloseDevice(dev_handle_);
        }
        iTek_UsbExit();
    }

    virtual int send_frame(const CanFrame_t& frame, unsigned int chn) override
    {
        iTek_CANFD_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(iTek_CANFD_Transmit_Data));

        transmit_data.send_type = 0;     /** normal mode */
        transmit_data.frame.can_id = frame.can_id;
        transmit_data.frame.cantype = 0; /** 0:CAN 2:CANFD 3:CANFD-speedup */
        transmit_data.frame.len = frame.can_dlc;
        memcpy(transmit_data.frame.data, frame.data, frame.can_dlc);

        int ret = iTek_Transmit(chn_handle_[chn], &transmit_data, 1);
        if(ret == 1)
        {
            LOGPF("send frame 0x%x success", frame.can_id);
            return frame.can_dlc;
        }
        else
        {
            LOGPF("send frame 0x%x fail", frame.can_id);
        }

        return -1;
    }

    virtual int recv_frame(CanFrameList_t& frames, unsigned int chn) override
    {
        /** read 1 frame each time, timeout=20ms */
        int reclen = iTek_Receive(chn_handle_[chn], buffer_[chn], 1, 20);
        for(int i=0; i<reclen; i++){
            CanFrame_t frame;
            frame.can_id = buffer_[chn][i].frame.can_id;
            frame.can_dlc = buffer_[chn][i].frame.len;
            memcpy(frame.data, buffer_[chn][i].frame.data, frame.can_dlc);
            frames.push(frame);
            LOGPF("recv frame[%d] timestamp: %ld, canid: 0x%x, cantype: %d, datalen: %d", i, buffer_[chn][i].timestamp, buffer_[chn][i].frame.can_id, buffer_[chn][i].frame.cantype, buffer_[chn][i].frame.len);
        }

        return reclen;
    }


    virtual int send_frame(const CanfdFrame_t& frame, unsigned int chn) override
    {
        iTek_CANFD_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(iTek_CANFD_Transmit_Data));

        transmit_data.send_type = 0;     /** normal mode */
        transmit_data.frame.can_id = frame.can_id;
        transmit_data.frame.cantype = 3; /** 0:CAN 2:CANFD 3:CANFD-speedup */
        transmit_data.frame.len = frame.len;
        memcpy(transmit_data.frame.data, frame.data, frame.len);

        int ret = iTek_Transmit(chn_handle_[chn], &transmit_data, 1);
        if(ret == 1)
        {
            LOGPF("send frame 0x%x success", frame.can_id);
            return frame.len;
        }
        else
        {
            LOGPF("send frame 0x%x fail", frame.can_id);
        }

        return -1;
    }

    virtual int recv_frame(CanfdFrameList_t& frames, unsigned int chn) override
    {
        /** read 1 frame each time, timeout=20ms */
        int reclen = iTek_Receive(chn_handle_[chn], buffer_[chn], 1, 20);
        for(int i=0; i<reclen; i++){
            CanfdFrame_t frame;
            frame.can_id = buffer_[chn][i].frame.can_id;
            frame.len = buffer_[chn][i].frame.len;
            memcpy(frame.data, buffer_[chn][i].frame.data, frame.len);
            frames.push(frame);
            LOGPF("recv frame[%d] timestamp: %ld, canid: 0x%x, cantype: %d, datalen: %d", i, buffer_[chn][i].timestamp, buffer_[chn][i].frame.can_id, buffer_[chn][i].frame.cantype, buffer_[chn][i].frame.len);
        }

        return reclen;
    }

    void test_can_classic()
    {
        init_usbcan();
        /** send on chn-0, recv on chn-1 */
        open_usbcan(0, 0);
        open_usbcan(1, 0);
        CanFrameList_t frames;
        for(int i=0x100; i<0x110; i++)
        {
            CanFrame_t frame;
            frame.can_id = i;
            frame.can_dlc = 8;
            send_frame(frame, 0);
            recv_frame(frames, 1);
            usleep(100*1000);
        }
        close_usbcan();
    }

    void test_canfd()
    {
        init_usbcan();
        /** send on chn-0, recv on chn-1 */
        open_usbcan(0, 1);
        open_usbcan(1, 1);
        CanfdFrameList_t frames;
        for(int i=0x200; i<0x210; i++)
        {
            CanfdFrame_t frame;
            frame.len = 64;
            frame.can_id = i;
            send_frame(frame, 0);
            recv_frame(frames, 1);
            usleep(100*1000);
        }
        close_usbcan();
    }

};

#endif