#ifndef CCONTROLPLAY_H
#define CCONTROLPLAY_H

#include <string>
#include <vector>

#define FB_YUYV8_422        12
#define FB_RAW12            20

#define GMSL_SPEED_1_5G     1500
#define GMSL_SPEED_3G       3000
#define GMSL_SPEED_6G       6000

#define PLAY_TIMESTAMP      0
#define PLAY_TRIGGER_GPIO   1

#define I2C_ADDR_END        1

typedef struct
{
    //i2c设备地址，例如max9295a,
    unsigned char  i2c_addr[16];
    //设备编号,板卡内唯一(0~63)
    unsigned char  virtual_sn;
    //设备读响应数据配置
    unsigned char  key_num;
    unsigned short key_addr[64];
    unsigned char  key_value[64];
}virtual_i2c_device_t;

std::vector<std::string> get_device_paths();
void *open_device(std::string path);
int close_device(void *dev);
int set_channel_img(void *dev, int ch, int video_type, int img_w, int img_h, int img_stride, int signal_w, int signal_h, int fps);
int set_channel_gmsl(void *dev, int ch, int gmsl_speed);
int set_channel_trigger(void *dev, int ch, int mode, int gpio);
//跳过部分开始时候的触发信号,不是必须调用
int set_trigger_ignore_count(void *dev, int ch, int trigger_ignore_count);
int channel_create(void *dev, int ch);
int channel_create_I2cDev(void *dev, int ch, const virtual_i2c_device_t *virtual_i2c_dev0, const virtual_i2c_device_t *virtual_i2c_dev1, const virtual_i2c_device_t *virtual_i2c_dev2);
int channel_start(void *dev, int ch);

int input_img(void *dev, int ch, char *img, unsigned long long time_sec, unsigned int time_nsec);
int get_channel_num(void *dev);
unsigned int get_version(void *dev);
unsigned long long get_release_time(void *dev);
const char *get_boardcard_name(void *dev);
int get_ptp_time(void *dev, unsigned long long *sec, unsigned int *nsec);

#endif // CCONTROLPLAY_H
