#pragma once
#ifndef __AUMO_DEVICE_WRAPPER_H__
#define __AUMO_DEVICE_WRAPPER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <queue>
#include "logging_utils.h"

typedef struct
{
    int ch = 0;
    int width = 640;
    int height = 360;
    int stride = 1280;
    int h_total = 640;
    int v_total = 360;
    int fps = 20;
    int video_type = FB_YUYV8_422;
    int gmsl_speed = GMSL_SPEED_3G;
    const virtual_i2c_device_t *vI2cDev0 = nullptr;
    const virtual_i2c_device_t *vI2cDev1 = nullptr;
    const virtual_i2c_device_t *vI2cDev2 = nullptr;
    char* pdata = nullptr;
    std::vector<uint8_t> yuyvImage;
    cv::Mat yuyvImageMat;
    unsigned long long time_sec = 0;
    unsigned int time_nsec = 0;
    unsigned int frame_period_nsec = 0;
    unsigned int img_count = 0;
} aumo_channel_info_t;
constexpr static int AUMO_CH_NUM_ = 8; 

class AumoDevice
{
private:
    void *pcie_card_addr_ = nullptr;

public:
    AumoDevice() {}
    ~AumoDevice() 
    {
        // destroy_dev();
    }

    bool init_dev(std::vector<aumo_channel_info_t>& ch_infos)
    {
        std::vector<std::string> device_paths = get_device_paths();
        if(device_paths.empty()){
            LOGPF("found no pcie device!");
            return false;
        }

        for(int i=0; i<device_paths.size(); i++)
        {
            LOGPF("pcie_device_paths[%d]: %s", i, device_paths[i].c_str());
        }

        /** open the first device */
        const int index = 0;
        pcie_card_addr_ = open_device(device_paths.at(index));

        if (pcie_card_addr_ == nullptr)
        {
            LOGPF("open aumo device error");
            return false;
        }

        /* check channel number */
        const int ch_num = get_channel_num(pcie_card_addr_);
        const char* cardname = get_boardcard_name(pcie_card_addr_);
        LOGPF("card: %s, get_channel_num: %d", cardname, ch_num);
        if (ch_num != AUMO_CH_NUM_)
        {
            LOGPF("aumo device must have %d channel!", AUMO_CH_NUM_);
            return false;
        }
        else
        {
            unsigned int ver = get_version(pcie_card_addr_);
            LOGPF("aumo firmware version: %d.%d.%02d", (ver >> 16) & 0xff, (ver >> 8) & 0xff, (ver >> 0) & 0xff);

            unsigned int time_nsec;
            unsigned long long time_sec;
            char tmp_str[128] = {0};
            struct tm *sttm;
            time_sec = get_release_time(pcie_card_addr_);
            sttm = localtime((time_t *)&time_sec);
            LOGPF("aumo release time:%04u-%02u-%02u %02u:%02u:%02u",
                        sttm->tm_year + 1900,
                        sttm->tm_mon + 1,
                        sttm->tm_mday,
                        sttm->tm_hour,
                        sttm->tm_min,
                        sttm->tm_sec);

            get_ptp_time(pcie_card_addr_, &time_sec, &time_nsec);
            sttm = localtime((time_t *)&time_sec);
            sprintf(tmp_str, "%04u-%02u-%02u %02u:%02u:%02u %d",
                    sttm->tm_year + 1900,
                    sttm->tm_mon + 1,
                    sttm->tm_mday,
                    sttm->tm_hour,
                    sttm->tm_min,
                    sttm->tm_sec,
                    time_nsec);
            LOGPF("aumo get_ptp_time: %s", tmp_str);

            /** init video channels */
            for (int i = 0; i < AUMO_CH_NUM_; i++)
            {
                aumo_channel_info_t& info = ch_infos[i];
                LOGPF("aumo create ch: %d", info.ch);
                int ret = set_channel_img(pcie_card_addr_,
                                info.ch,
                                info.video_type,
                                info.width,
                                info.height,
                                info.stride,
                                info.h_total, 
                                info.v_total,
                                info.fps);
                LOGPF("ch[%d] set_channel_img: %d", info.ch, ret);
                ret = set_channel_gmsl(pcie_card_addr_,
                                    info.ch,
                                    info.gmsl_speed);
                LOGPF("ch[%d] set_channel_gmsl: %d", info.ch, ret);
                ret = set_channel_trigger(pcie_card_addr_,
                                    info.ch,
                                    PLAY_TIMESTAMP,
                                    2);
                LOGPF("ch[%d] set_channel_trigger: %d", info.ch, ret);
                ret = channel_create(pcie_card_addr_, info.ch);
                LOGPF("ch[%d] channel_create: %d", info.ch, ret);
                ret = channel_create_I2cDev(pcie_card_addr_, info.ch,
                                      info.vI2cDev0,
                                      info.vI2cDev1,
                                      info.vI2cDev2);
                LOGPF("ch[%d] channel_create_I2cDev: %d", info.ch, ret);
                ret = channel_start(pcie_card_addr_, info.ch);
                LOGPF("ch[%d] channel_start: %d", info.ch, ret);
            }
            return true;
        }
    }

    bool inject_image(aumo_channel_info_t& info)
    {
        if(info.pdata != nullptr)
        {
            int ret = input_img(pcie_card_addr_, info.ch, info.pdata, info.time_sec, info.time_nsec);
            info.time_nsec += info.frame_period_nsec;
            if (info.time_nsec >= 1e9)
            {
                info.time_nsec -= 1e9;
                info.time_sec += 1;
            }
            info.img_count++;
            info.pdata = nullptr;
            LOGPF("ch[%d] inject image count: %d, ret: %d", info.ch, info.img_count, ret);
            return (ret == 0); 
        }
        else
        {
            return false;
        }

    }

    void destroy_dev()
    {
        if(pcie_card_addr_)
        {
            close_device(pcie_card_addr_);
        }
    }

};

#endif