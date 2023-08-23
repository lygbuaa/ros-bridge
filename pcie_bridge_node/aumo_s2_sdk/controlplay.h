#ifndef CCONTROLPLAY_H
#define CCONTROLPLAY_H


/***********************************************************************
                                版本修订说明
日期:2023/06/25
    sdk版本:1.5.6
    固件版本:1.5.6
    修改内容:
        1.优化PLAY_NODELAY模式注入延迟(fpga发送模块变化较大)
        2.添加函数set_trigger_delay_ns，用于设置收到trigger信号后，延迟发送图像的时间
        3.添加函数gmsl_reset_manage，用于管理gmsl链路的初始化，用于解决接收端可能接收不到数据的问题
        4.添加函数set_channel_ByteSwitch,

日期:2023/06/11
    sdk版本:1.4.7
    修改内容:
        1.解决get_device_paths返回值出错的bug
        2.set_channel_max9295a参数count等于-2时，sdk将不再重置gmsl链路
        3.降低dphy数据发送速率
        4.修复有通道注入图像数据不稳定bug

日期:2023/06/05
    sdk版本:1.4.6
    固件版本:1.4.6
    修改内容:
        1.添加sdk版本查询: getSdkRevision
        2.添加注入模式: PLAY_NODELAY
        3.添加MAX9295A用户配置接口: set_channel_max9295a
        4.添加独立8通道获取ptp时间模块,get_ptp_time添加通道参数
        5.添加板卡执行命令与获取返回结果函数(exe_script_comm,exe_script_read_result)
***********************************************************************/
#include <string>
#include <vector>

#define FB_YUYV8_422        12
#define FB_RAW12            20

#define GMSL_SPEED_1_5G     1500
#define GMSL_SPEED_3G       3000
#define GMSL_SPEED_6G       6000

#define PLAY_TIMESTAMP      0
#define PLAY_TRIGGER_GPIO   1
#define PLAY_NODELAY        2

#define I2C_ADDR_END        1

typedef struct
{
    //i2c设备地址，例如max9295a,
    unsigned char  i2c_addr[16];
    //设备编号,板卡内唯一(0~63)
    unsigned char  virtual_sn;
    //设备读响应数据配置
    unsigned short key_num;
    unsigned short key_addr[64];
    unsigned char  key_value[64];
    int A_num;
    unsigned short start_addr_A;
    const unsigned char *valueA;
    int B_num;
    unsigned short start_addr_B;
    const unsigned char *valueB;
    int C_num;
    unsigned short start_addr_C;
    const unsigned char *valueC;
}virtual_i2c_device_t;

std::vector<std::string> get_device_paths();
const char *getSdkRevision();
void *open_device(std::string path);
int channel_create(void *dev, int ch, int video_type, int img_w, int img_h, int img_stride, int signal_w, int signal_h, int fps);
int set_channel_gmsl(void *dev, int ch, int gmsl_speed);
int set_channel_trigger(void *dev, int ch, int mode, int gpio);
//不再使用默认配置，可以添加用户对max9295a的额外配置
int set_channel_max9295a(void *dev, int ch, const unsigned short *list, int count);
//跳过部分开始时候的触发信号,不是必须调用(仅用于触发模式)
int set_trigger_ignore_count(void *dev, int ch, int trigger_ignore_count);
//收到trigger信号后，延迟发送图像的时间
int set_trigger_delay_ns(void *dev, int ch, unsigned int delay);
//注入数据交换位置
    //mode=0 数据：0、1、2、3 ...  变换为：0、1、2、3 ...
    //mode=1 数据：0、1、2、3 ...  变换为：1、0、3、2 ...
int set_channel_ByteSwitch(void *dev, int ch, unsigned int mode);
//用于虚拟i2c设备
int set_channel_I2cDev(void *dev, int ch, const virtual_i2c_device_t *virtual_i2c_dev0, const virtual_i2c_device_t *virtual_i2c_dev1, const virtual_i2c_device_t *virtual_i2c_dev2);
int channel_start(void *dev, int ch);
void gmsl_reset_manage(void *dev);

int input_img(void *dev, int ch, char *img, unsigned long long time_sec, unsigned int time_nsec);

int get_channel_num(void *dev);
unsigned int get_version(void *dev);
unsigned long long get_release_time(void *dev);
const char *get_boardcard_name(void *dev);
int get_ptp_time(void *dev, unsigned long long *sec, unsigned int *nsec, int ch);
int exe_script(void *dev, const char *commbuf);
int exe_script_comm(void *dev, const char *commbuf);
const char *exe_script_read_result(void *dev);

void set_test_gpio(void *dev, int v);


int camfd_channel_create(void *dev, int ch, int bitRate_can, int bitRate_canfd);
int camfd_channel_start(void *dev, int ch);
void input_cam_pkg(void *dev, int ch, unsigned int id, unsigned int *pData, int len, unsigned long long sec, unsigned int nsec);

#endif // CCONTROLPLAY_H
