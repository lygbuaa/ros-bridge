#ifndef  ITEKCANFD_H
#define  ITEKCANFD_H

#include <stdint.h>
#include <stdbool.h>

#define CANFD_MAX_DLEN 64
typedef uint32_t canid_t;
typedef void * DEVICE_HANDLE;
typedef void * USBChannel_Handle;
typedef void * CHANNEL_HANDLE;

typedef struct tagiTek_CAN_DEVICE_INFO {
    uint8_t  hw_Version[3];             /* 硬件版本，如hw_Version={0x01, 0x00, 0x02}，代表V1.0.2 */
    uint8_t  fw_Version[3];             /* 固件版本，如fw_Version={0x01, 0x00, 0x03}，代表V1.0.3 */
    uint8_t  product_Version[3];        /* 暂不开放 */
    uint8_t  can_Num;                   /* 通道数量，如can_Num=2，代表设备集成2路CANFD接口 */
    uint8_t  str_Serial_Num[20];        /* 出厂序列号，以‘\0’结束，如“6120101001” */
    uint8_t  str_hw_Type[40];           /* 设备名称描述字符，以‘\0’结束，如“USBCANFD-X200” */
    uint16_t reserved[6];               /* 保留 */
}iTek_CANFD_DEVICE_INFO;

typedef struct tagFilterData {
    uint8_t     frameType;              /* 帧类型，用于说明该组过滤器组是针对标准帧还是扩展帧id有效；0=标准帧有效;1=扩展帧有效 */
    uint8_t     filterType;             /* 过滤器类型，用于说明本组过滤器组的过滤方式；0=范围id；1=明确id；2=掩码id；不同方式的过滤结果 */
    uint32_t    ID1;                    /* ID寄存器1 */
    uint32_t    ID2;                    /* ID寄存器2 */
}FilterData;

typedef struct tagFilterDataExtend{
    int num;                            /* 实际使用扩展帧过滤器组数量，取值范围0-63，比如：num=2，代表前2组有效 */
    FilterData filterDataExtend[64];    /* 扩展帧过滤器组寄存器结构体 */
}FilterDataExtend;

typedef struct tagFilterDataStandard{
    int num;                            /* 实际使用标准帧过滤器组数量，取值范围0-127，比如：num=5，代表前5组有效； */
    FilterData filterDataStandard[128]; /* 标准帧过滤器组寄存器结构体 */
}FilterDataStandard;

typedef struct tagiTek_CAN_CHANNEL_INIT_CONFIG {
    uint8_t    can_type;                /* CAN协议类型： 0=CAN协议；1=CANFD协议 */
    uint8_t    CANFDStandard;           /* CANFD标准：0=ISO标准；1=非ISO标准 */
    uint8_t    CANFDSpeedup;            /* CANFD是否加速： 0=不加速； 1=加速 */
    uint32_t   abit_timing;             /* 仲裁波特率 */
    uint32_t   dbit_timing;             /* 数据波特率 */
    uint8_t    workMode;                /* 工作模式， 0=正常工作模式；1=只听工作模式 */
    uint32_t   res;                     /* 保留位 */
    FilterDataExtend Extend;            /* 扩展帧过滤器组 */
    FilterDataStandard Standard;        /* 标准帧过滤器组 */
}iTek_CANFD_CHANNEL_INIT_CONFIG;

/** rename this struct to canfd_frame_itekon, to be different with socketcan definition */
typedef struct tag_canfd_frame {
    canid_t	can_id;                     /* 帧ID，高3位属于标志位，低29位ID有效位 */
    uint8_t    len;                     /* 数据长度，当前CAN(FD)帧实际数据长度 */
    uint8_t    flags;                   /* 错误帧标志位，0=正常数据帧；1=错误帧；当flags=1时，错误信息通过CAN帧数据位data0-data7表达 */
    uint8_t    res;                     /* 保留位 */
    uint8_t    cantype;                 /* CAN类型，0 = CAN；2 = CANFD；3= CANFD加速 */
    uint8_t    data[CANFD_MAX_DLEN];    /* 数据，CAN帧data<=8，CANFD帧<=64 */
}canfd_frame_itekon;

typedef struct tagiTek_CANFD_Data{
    canfd_frame_itekon     frame;              /* CAN(FD)报文信息 */
    uint64_t        timestamp;          /* CAN时间戳，从CAN控制器上电开始计时，长度64位，单位us（接收帧有效） */
}iTek_CANFD_Receive_Data;

typedef struct tagiTek_Transmit_CANFD_Data {
    canfd_frame_itekon           frame;        /*CAN(FD)报文信息*/
    uint16_t			  send_type;    /*发送方式：0 = 正常发送；1 = 自发自收*/
}iTek_CANFD_Transmit_Data;

#ifdef __cplusplus

extern "C"
{
#endif

int iTek_UsbInit ();
void iTek_UsbExit ();
DEVICE_HANDLE iTek_OpenDevice(uint32_t Device_Type, uint16_t device_index, uint32_t reserved);
void iTek_CloseDevice(DEVICE_HANDLE usbhandle);
bool iTek_GetDeviceInfo(DEVICE_HANDLE usbhandle, iTek_CANFD_DEVICE_INFO* pinfo);
bool iTek_RestCAN(CHANNEL_HANDLE channel_handle);
CHANNEL_HANDLE iTek_InitCan(DEVICE_HANDLE usbhandle, uint8_t channel, iTek_CANFD_CHANNEL_INIT_CONFIG* config);
bool iTek_StartCAN(CHANNEL_HANDLE channel_handle);

uint32_t iTek_Receive(CHANNEL_HANDLE channel_handle, iTek_CANFD_Receive_Data* pReceive, uint32_t len, int wait_time);
uint32_t iTek_Transmit(CHANNEL_HANDLE channel_handle, iTek_CANFD_Transmit_Data* data, uint32_t len);

void iTek_ClearBuffer(CHANNEL_HANDLE channel_handle);
uint32_t iTek_GetReceiveNum(CHANNEL_HANDLE channel_handle);
void *iTek_Authenticate(DEVICE_HANDLE usbhandle, uint16_t key_type,uint8_t *pStr, int len, unsigned char *dStr);
bool iTek_isConnected(DEVICE_HANDLE usbhandle);


#ifdef __cplusplus
}
#endif



#endif  /*ITEKCANFD_H*/
