
#include "aumo_device.h"

//* log with fprintf *//
#ifndef LOGPF
#define LOGPF(format, ...) fprintf(stderr ,"[%s:%d] " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif

int main(int argc, char **argv)
{
    for(int i = 0; i < argc; i++){
        LOGPF("argv[%d] = %s\n", i, argv[i]);
    }

    if(argc < 8)
    {
        LOGPF("missing args: test_video_inject [card_id] [img_path] [format] [img_w] [img_h] [sig_w] [sig_h]");
        return 0;
    }

    /** input args */
    const int card_id = atoi(argv[1]);
    const std::string img_path(argv[2]);

    int format = FB_YUYV8_422;
    if(strncmp(argv[3], "uyvy", 4) == 0)
    {
        format = FB_YUYV8_422;
    }
    else if(strncmp(argv[3], "prcc", 4) == 0)
    {
        format = FB_RAW12;
    }
    else
    {
        LOGPF("wrong format!");
        return 0;
    }

    const int img_w = atoi(argv[4]);
    const int img_h = atoi(argv[5]);
    const int sig_w = atoi(argv[6]);
    const int sig_h = atoi(argv[7]);

    std::vector<aumo_video_channel_info_t> aumo_video_infos_;
    std::vector<aumo_canfd_channel_info_t> aumo_canfd_infos_;
    size_t img_len = 0;

    for(int i=0; i<AUMO_VIDEO_CH_NUM_; i++)
    {
        aumo_video_channel_info_t info;
        info.ch = i;
        info.width = img_w;
        info.height = img_h;
        info.h_total = sig_w;
        info.v_total = sig_h;
        info.fps = 10;
        info.frame_period_nsec = 1e9 / info.fps;
        info.video_type = format;     //FB_YUYV8_422
        info.gmsl_speed = GMSL_SPEED_3G;
        info.trigger_mode = PLAY_NODELAY;

        float stride = (info.video_type==FB_RAW12) ? 1.5f : 2.0f;
        info.stride = (int)round(info.width * stride);
        img_len = info.stride * info.height;

        aumo_video_infos_.emplace_back(info);
    }

    std::shared_ptr<AumoDevice> aumo_device_ = std::make_shared<AumoDevice>();
    if(!aumo_device_->init_dev(card_id, aumo_video_infos_, aumo_canfd_infos_))
    {
        LOGPF("pcie card [%d] init error, quit!", card_id);
        return -1;
    }

    char* buffer = (char*)malloc(img_len);
    memset(buffer, 0x0, img_len);

    while(true)
    {
        for(int i=0; i<AUMO_VIDEO_CH_NUM_; i++)
        {
            aumo_video_channel_info_t& info = aumo_video_infos_[i];
            info.pdata = buffer;
            aumo_device_ -> inject_image(info);
        }
    }

    free(buffer);

}