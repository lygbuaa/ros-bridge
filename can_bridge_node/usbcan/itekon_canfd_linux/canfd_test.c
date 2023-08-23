
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include "iTekCANFD.h"

/*仲裁域定义*/
#define Arbitrat_Rate5K 0x01F31302
#define Arbitrat_Rate10K 0x00F91302
#define Arbitrat_Rate20K 0x007C1302
#define Arbitrat_Rate40K 0x00630A02
#define Arbitrat_Rate50K 0x00311302
#define Arbitrat_Rate80K 0x001D1204
#define Arbitrat_Rate100K 0x00181302
#define Arbitrat_Rate125K 0x00131302
#define Arbitrat_Rate200K 0x00130A02
#define Arbitrat_Rate250K 0x00091302
#define Arbitrat_Rate400K 0x00090A02
#define Arbitrat_Rate500K 0x00070A02
#define Arbitrat_Rate800K 0x00021006
#define Arbitrat_Rate1000K 0x00040702

/*数据域定义*/
#define Data_Rate100K 0x001D0E30
#define Data_Rate125K 0x001F0A20
#define Data_Rate200K 0x00130A20
#define Data_Rate250K 0x000F0A20
#define Data_Rate400K 0x00090A20
#define Data_Rate500K 0x00090A20
#define Data_Rate800K 0x00040A20
#define Data_Rate1M 0x00040720
#define Data_Rate2M 0x00010A20
#define Data_Rate3M 0x00000D40
#define Data_Rate4M 0x00000A20
#define Data_Rate5M 0x00000720
#define Data_Rate6M 0x00010200

DEVICE_HANDLE devhandle;
CHANNEL_HANDLE canhandl1, canhandl2;

int breakflag = 0;
void exit_can(int flag);
void *handle_send(void *arg)
{
    iTek_CANFD_Transmit_Data transmit_data[3];
    memset(transmit_data, 0, sizeof(iTek_CANFD_Transmit_Data) * 3);
    uint32_t ret = 0;
    uint8_t data[64] = {0};

    for (int i = 0; i < 64; i++) {
        data[i] = i + 1;
    }

    while (1) {
        if (breakflag) {
            breakflag = 2;
            return NULL;
        }

        sleep(1);
        data[7]++;
        transmit_data[0].send_type = 0;
        transmit_data[0].frame.can_id++;
        transmit_data[0].frame.cantype = 3; /*0:CAN 2:CANFD 3:CANFD加速*/
        transmit_data[0].frame.len = strlen(data);
        memcpy(transmit_data[0].frame.data, data, strlen(data));

        ret = iTek_Transmit(canhandl1, transmit_data, 1);
        printf("transdata:%d\n", ret);
    }
}

void exit_can(int flag)
{
    int time = 0;

    if (1 != flag) {
        printf("---------------------------------------ctrl+c\n");
        breakflag = 1;

        while (1) {
            if ((breakflag == 2) || (breakflag == 0)) {
                break;
            }

            sleep(1);
            time++;

            if (time > 5) {
                break;
            }
        }

        breakflag = 0;
    }

    iTek_RestCAN(canhandl1);
    iTek_RestCAN(canhandl2);
    iTek_CloseDevice(devhandle);

    iTek_UsbExit();

    exit(0);
}

void signal_handle()
{
    struct sigaction sig_handle;
    memset(&sig_handle, 0, sizeof(sig_handle));
    sigemptyset(&sig_handle.sa_mask);
    sig_handle.sa_flags = SA_SIGINFO;
    sig_handle.sa_sigaction = exit_can;
    sigaction(SIGINT, &sig_handle, NULL);
}

int main()
{
    pthread_t send_handle;
    iTek_CANFD_DEVICE_INFO devinfo;
    iTek_CANFD_CHANNEL_INIT_CONFIG can1, can2;

    memset(&devinfo, 0, sizeof(devinfo));

    if (!iTek_UsbInit()) {
        exit_can(1);
        return 0;
    }

    devhandle = iTek_OpenDevice(2, 0, 0);

    if (NULL == devhandle) {
        exit_can(1);
        return 0;
    }

    iTek_GetDeviceInfo(devhandle, &devinfo);
    printf("硬件版本:V%d.%d.%d\n", devinfo.hw_Version[0], devinfo.hw_Version[1], devinfo.hw_Version[2]);
    printf("固件版本:V%d.%d.%d\n", devinfo.fw_Version[0], devinfo.fw_Version[1], devinfo.fw_Version[2]);
    printf("通道数量:%d\n", devinfo.can_Num);
    printf("出厂序列号:%s\n", devinfo.str_Serial_Num);
    printf("设备名称描述字符:%s\n", devinfo.str_hw_Type);

    memset(&can1, 0, sizeof(can1));
    memset(&can2, 0, sizeof(can2));

    can1.can_type = 1;
    can1.CANFDStandard = 0;
    can1.CANFDSpeedup = 1;
    can1.workMode = 0;
    can1.abit_timing = Arbitrat_Rate250K;
    can1.dbit_timing = Data_Rate1M;
    can1.Standard.num = 1;
    can1.Standard.filterDataStandard->ID1 = 0x00;
    can1.Standard.filterDataStandard->ID2 = 0x7ff;
    can1.Standard.filterDataStandard->frameType = 0;
    can1.Standard.filterDataStandard->filterType = 0;
    can1.Extend.num = 1;
    can1.Extend.filterDataExtend->ID1 = 0x00;
    can1.Extend.filterDataExtend->ID2 = 0x1FFFFFFF;
    can1.Extend.filterDataExtend->frameType = 1;
    can1.Extend.filterDataExtend->filterType = 0;

    can2.can_type = 1;
    can2.CANFDStandard = 0;
    can2.CANFDSpeedup = 1; /*0不加速 1加速*/
    can2.workMode = 0;
    can2.abit_timing = Arbitrat_Rate250K;
    can2.dbit_timing = Data_Rate1M;
    can2.Standard.num = 1;
    can2.Standard.filterDataStandard->ID1 = 0x00;
    can2.Standard.filterDataStandard->ID2 = 0x7ff;
    can2.Standard.filterDataStandard->frameType = 0;
    can2.Standard.filterDataStandard->filterType = 0;
    can2.Extend.num = 1;
    can2.Extend.filterDataExtend->ID1 = 0x00;
    can2.Extend.filterDataExtend->ID2 = 0x1FFFFFFF;
    can2.Extend.filterDataExtend->frameType = 1;
    can2.Extend.filterDataExtend->filterType = 0;

    canhandl1 = iTek_InitCan(devhandle, 0, &can1);

    if (canhandl1 == NULL) {
        exit_can(1);
        return 0;
    }

    canhandl2 = iTek_InitCan(devhandle, 1, &can2);

    if (canhandl2 == NULL) {
        exit_can(1);
        return 0;
    }

    if (!iTek_StartCAN(canhandl1)) {
        exit_can(1);
        return 0;
    }

    if (!iTek_StartCAN(canhandl2)) {
        exit_can(1);
        return 0;
    }

    signal_handle();

    pthread_create(&send_handle, NULL, handle_send, NULL);

    iTek_CANFD_Receive_Data recvdata[100];
    int recvnum = 0;
    int totalnum = 0;

    while (1) {
        memset(recvdata, 0, sizeof(iTek_CANFD_Transmit_Data) * 100);
        recvnum = iTek_Receive(canhandl2, recvdata, 100, 1000);
        totalnum += recvnum;
        printf("totalnum:%d\n", totalnum);

        for (int j = 0; j < recvnum; j++) {
            printf("[%d]recvtime:[%llx] canid:[%x] cantype:[%d] datalen:[%d] data[", j, recvdata[j].timestamp, recvdata[j].frame.can_id & 0x1fffffff, recvdata[j].frame.cantype, recvdata[j].frame.len);

            for (int i = 0; i < recvdata[j].frame.len; i++) {
                printf("%02x ", recvdata[j].frame.data[i]);
            }

            printf("]\n");
        }
    }

    return 0;
}
