
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include "iTekCANFD.h"
#include "rbuf.h"

#define HTONL(v) (((v) << 24) | (((v) >> 24) & 255) | (((v) << 8) & 0xff0000) | (((v) >> 8) & 0xff00))
#define HTONS(v) ((((v) << 8) & 0xff00) | (((v) >> 8) & 255))
#define HTONLL(v) (((HTONL((int)((v << 32) >> 32))) << 32) | (unsigned int)HTONL((int)(v >> 32)))

libusb_context *contex = NULL;

static int usbinit_flag = 0;
int in_error = 0;

typedef struct usbinfo {
    struct libusb_device_handle *usb_p;
    rbuf_t *rbuf_handle[2];
    pthread_t recvthread_handle;
    int exit_flag;
    int error;
} USB_INFO_T;

typedef struct caninfo {
    USB_INFO_T *usbinfo;
    uint8_t canindex;
    iTek_CANFD_CHANNEL_INIT_CONFIG config;
} CAN_INFO_T;

// pthread_t recvthread_handle[16];

int self_usb_sendmsg_ed1(struct libusb_device_handle *usb_p, unsigned char *write_msg, int write_len)
{
    if (usb_p == NULL) {
        return -1;
    }

    int recv_len = 0;
    int ret = libusb_interrupt_transfer(usb_p, 0x1, write_msg, write_len, &recv_len, 1000);

    if (ret < 0) {
        printf("LINE:%d\tret %s recv %d\n", __LINE__, libusb_strerror(ret), recv_len);
        goto end;
    }

end:
    return ret;
}

int self_usb_recvmsg_ed1(struct libusb_device_handle *usb_p, unsigned char *recv_msg, int recv_max)
{
    if (usb_p == NULL) {
        return -1;
    }

    int recv_len = 0;
    int ret = libusb_interrupt_transfer(usb_p, 0x81, recv_msg, recv_max, &recv_len, 1000);

    if (ret < 0 && ret != LIBUSB_ERROR_OVERFLOW && ret != LIBUSB_ERROR_TIMEOUT) {
        goto end;
    }

    ret = recv_len;
end:
    return ret;
}

int self_usb_sendmsg_ed2(struct libusb_device_handle *usb_p, unsigned char *write_msg, int write_len)
{
    if (usb_p == NULL) {
        return -1;
    }

    int recv_len = 0;
    int ret = libusb_bulk_transfer(usb_p, 0x2, write_msg, write_len, &recv_len, 1000);

    if (ret < 0) {
        printf("LINE:%d\tret %s recv %d\n", __LINE__, libusb_strerror(ret), recv_len);
        goto end;
    }

end:
    return ret;
}

int self_usb_recvmsg_ed2(struct libusb_device_handle *usb_p, unsigned char *recv_msg, int recv_max)
{
    if (usb_p == NULL) {
        return -1;
    }

    int recv_len = 0;
    int ret = libusb_bulk_transfer(usb_p, 0x82, recv_msg, recv_max, &recv_len, 1000);

    if (ret < 0 && ret != LIBUSB_ERROR_OVERFLOW && ret != LIBUSB_ERROR_TIMEOUT) {
        goto end;
    }

    ret = recv_len;
end:
    return ret;
}

int self_usb_recvmsg_ed3(struct libusb_device_handle *usb_p, unsigned char *recv_msg, int recv_max)
{
    if (usb_p == NULL) {
        return -1;
    }

    int recv_len = 0;
    int ret = libusb_bulk_transfer(usb_p, 0x83, recv_msg, recv_max, &recv_len, 1000);

    if (ret < 0 && ret != LIBUSB_ERROR_OVERFLOW && ret != LIBUSB_ERROR_TIMEOUT) {
        goto end;
    }

    ret = recv_len;
end:
    return ret;
}

void *handle_recv(void *arg)
{
    USB_INFO_T *usbinfo_handle = (USB_INFO_T *)arg;

    uint8_t recv_data[483] = {0};
    uint8_t canframe_data[80] = {0};
    uint32_t canframe_len = 0;
    uint32_t payload_len = 0;
    int32_t recv_len = 0;
    int32_t data_num = 0;
    int32_t i;
    uint64_t *Changetmp64;
    uint32_t *Changetmp32;
    uint8_t canindex = 0;
    int32_t real_write = 0;

    iTek_CANFD_Receive_Data data;

    while (1) {
        if (usbinfo_handle->exit_flag == 1) {
            usbinfo_handle->exit_flag = 2;
            break;
        }

        memset(recv_data, 0, sizeof(recv_data));

        recv_len = self_usb_recvmsg_ed3(usbinfo_handle->usb_p, recv_data, sizeof(recv_data));
        data_num = recv_data[2];

        if (data_num == 0) {
            usleep(10 * 1000);
            continue;
        }

        if (recv_data[1] == 0) {
            /*CAN */
            canframe_len = 24;
            payload_len = 8;
        } else if (recv_data[1] == 1) {
            /*CANFD*/
            canframe_len = 80;
            payload_len = 64;
        }

        for (i = 0; i < data_num; i++) {
            memset(canframe_data, 0, sizeof(canframe_data));
            memcpy(canframe_data, recv_data + 3 + canframe_len * i, canframe_len);

            memset(&data, 0, sizeof(iTek_CANFD_Receive_Data));
            Changetmp64 = (uint64_t *)&canframe_data[0];
            data.timestamp = HTONLL(*Changetmp64);
            //      printf("[%02x%02x%02x%02x%02x%02x%02x%02x][%x]\n", canframe_data[0],canframe_data[1],canframe_data[2],canframe_data[3],canframe_data[4],canframe_data[5],canframe_data[6],canframe_data[7],data.timestamp);
            canindex = canframe_data[9] & 0x7;
            data.frame.flags = canframe_data[9] & 0x80;
            data.frame.cantype = canframe_data[10] & 0x3;
            data.frame.len = canframe_data[11];

            Changetmp32 = (uint32_t *)&canframe_data[12];
            data.frame.can_id = HTONL(*Changetmp32);
            memcpy(data.frame.data, canframe_data + 16, payload_len);

            real_write = rbuf_write(usbinfo_handle->rbuf_handle[canindex], (u_char *)&data, sizeof(iTek_CANFD_Receive_Data));

            if (real_write != sizeof(iTek_CANFD_Receive_Data)) {
                printf("LINE:%d\t 缓冲区已满\n", __LINE__);
            }

            memset(canframe_data, 0, sizeof(canframe_data));
        }
    }

    return NULL;
}

int iTek_UsbInit()
{
    if (usbinit_flag != 0) {
        return 0;
    }

    libusb_init(&contex);
    // libusb_set_option(contex, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
    usbinit_flag = 1;
    return 1;
}

void iTek_UsbExit()
{
    if (usbinit_flag == 1) {
        libusb_exit(contex);
        usbinit_flag = 0;
    }
}

DEVICE_HANDLE iTek_OpenDevice(uint32_t Device_Type, uint16_t device_index, uint32_t reserved)
{
    if (usbinit_flag != 1) {
        in_error = 2;
        return NULL;
    }

    USB_INFO_T *usbinfo_handle = NULL;
    uint16_t vendor_id, product_id;

    if (Device_Type == 1) {
    } else if (Device_Type == 2) {
        vendor_id = 0x1fc9;
        product_id = 0x0100;
    } else {
        return NULL;
    }

    ssize_t cnt;
    libusb_device *dev;
    libusb_device **devs;
    libusb_device *devlist[16];
    int dev_num = 0;
    int i = 0;

    cnt = libusb_get_device_list(contex, &devs);

    if (cnt < 0) {
        return NULL;
    }

    while ((dev = devs[i++]) != NULL) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);

        if (r < 0) {
            printf("打开设备错误\n");
            libusb_free_device_list(devs, 1);
            return NULL;
        }

        if ((desc.idVendor == vendor_id) && (desc.idProduct == product_id)) {
            if (dev_num >= 16) {
                continue;
            }

            devlist[dev_num] = dev;
            dev_num++;
        }
    }

    if (dev_num == 0) {
        printf("此设备不存在\n");
        libusb_free_device_list(devs, 1);
        return NULL;
    }

    if (device_index > dev_num) {
        printf("超出可用设备\n");
        libusb_free_device_list(devs, 1);
        return NULL;
    }

    dev = devlist[device_index];
    usbinfo_handle = (USB_INFO_T *)calloc(1, sizeof(USB_INFO_T));

    if (NULL == usbinfo_handle) {
        libusb_free_device_list(devs, 1);
        return NULL;
    }

    int ret = libusb_open(dev, &usbinfo_handle->usb_p);

    if (ret < 0) {
        free(usbinfo_handle);
        libusb_free_device_list(devs, 1);
        return NULL;
    } else {
        printf("success\n");
    }

    libusb_free_device_list(devs, 1);

    ret = libusb_claim_interface(usbinfo_handle->usb_p, 0);
    printf("LINE %d, ret %d\n", __LINE__, ret);

    usbinfo_handle->rbuf_handle[0] = rbuf_create(100000 * sizeof(iTek_CANFD_Receive_Data));
    usbinfo_handle->rbuf_handle[1] = rbuf_create(100000 * sizeof(iTek_CANFD_Receive_Data));

    ret = pthread_create(&usbinfo_handle->recvthread_handle, NULL, handle_recv, (void *)usbinfo_handle);

    return (DEVICE_HANDLE)usbinfo_handle;
}

void iTek_CloseDevice(DEVICE_HANDLE usbhandle)
{
    USB_INFO_T *selfhandle = (USB_INFO_T *)usbhandle;

    if (NULL == usbhandle) {
        return;
    }

    if (selfhandle->recvthread_handle != 0) {
        selfhandle->exit_flag = 1;
    }

    while (1) {
        sleep(1);

        if (selfhandle->exit_flag != 1) {
            break;
        }
    }

    libusb_release_interface(selfhandle->usb_p, 0);

    libusb_close(selfhandle->usb_p);
    selfhandle->usb_p = NULL;

    rbuf_destroy(selfhandle->rbuf_handle[0]);
    rbuf_destroy(selfhandle->rbuf_handle[1]);

    pthread_cancel(selfhandle->recvthread_handle);
    pthread_join(selfhandle->recvthread_handle, NULL);

    free(selfhandle);
    selfhandle = NULL;
    return;
}

bool iTek_GetDeviceInfo(DEVICE_HANDLE usbhandle, iTek_CANFD_DEVICE_INFO *pinfo)
{
    USB_INFO_T *selfhandle = (USB_INFO_T *)usbhandle;

    if (selfhandle == NULL) {
        return false;
    }

    unsigned char query_data[51] = {0};
    unsigned char recv_data[51] = {0};
    query_data[0] = 0x11;
    query_data[1] = 0x03;
    query_data[2] = 0x30;
    query_data[3] = 0x00;

    int recv_len = self_usb_sendmsg_ed1(selfhandle->usb_p, query_data, sizeof(query_data));

    if (recv_len < 0) {
        return false;
    }

    recv_len = self_usb_recvmsg_ed1(selfhandle->usb_p, recv_data, sizeof(recv_data));

    if (recv_len < 0) {
        return false;
    }

    if (recv_data[2] == 0xB0) {
        return false;
    }

    pinfo->can_Num = recv_data[3];
    memcpy(pinfo->hw_Version, recv_data + 4, 3);
    memcpy(pinfo->fw_Version, recv_data + 7, 3);
    memcpy(pinfo->str_hw_Type, recv_data + 10, 20);
    memcpy(pinfo->str_Serial_Num, recv_data + 30, 16);

    return true;
}

bool iTek_RestCAN(CHANNEL_HANDLE channel_handle)
{
    if (channel_handle == NULL) {
        return false;
    }

    CAN_INFO_T *self_canhandle = (CAN_INFO_T *)channel_handle;
    uint8_t restcan_data[51] = {0};
    uint8_t recv_data[51] = {0};
    restcan_data[0] = 0x11;
    restcan_data[1] = 0x07;
    restcan_data[2] = 0x30;
    restcan_data[3] = self_canhandle->canindex;

    int recv_len = self_usb_sendmsg_ed1(self_canhandle->usbinfo->usb_p, restcan_data, sizeof(restcan_data));

    if (recv_len < 0) {
        return false;
    }

    recv_len = self_usb_recvmsg_ed1(self_canhandle->usbinfo->usb_p, recv_data, sizeof(recv_data));

    if (recv_len < 0) {
        return false;
    }

    free(self_canhandle);
    self_canhandle = NULL;
    return true;
}

CHANNEL_HANDLE iTek_InitCan(DEVICE_HANDLE usbhandle, uint8_t channel, iTek_CANFD_CHANNEL_INIT_CONFIG *config)
{
    if (NULL == usbhandle) {
        return NULL;
    }

    if (channel < 0 || channel >= 2) {
        return NULL;
    }

    if (config->can_type < 0 || config->can_type >= 2) {
        return NULL;
    }

    if (config->CANFDStandard < 0 || config->CANFDStandard >= 2) {
        return NULL;
    }

    if (config->CANFDSpeedup < 0 || config->CANFDSpeedup >= 2) {
        return NULL;
    }

    if (config->workMode < 0 || config->workMode >= 2) {
        return NULL;
    }

    USB_INFO_T *self_usbhandle = (USB_INFO_T *)usbhandle;
    uint8_t initcan_data[52] = {0x11, 0x04, 0x30, 0x00};
    uint8_t recv_data[52] = {0};
    memset(initcan_data + 4, 0, sizeof(initcan_data) - 4);
    initcan_data[4] = channel;
    initcan_data[5] = config->can_type;
    initcan_data[6] = config->CANFDStandard;
    initcan_data[7] = config->CANFDSpeedup;
    initcan_data[8] = config->workMode;

    uint32_t *Changetmp = (uint32_t *)&initcan_data[9];
    *Changetmp = HTONL(config->abit_timing);

    Changetmp = (uint32_t *)&initcan_data[13];
    *Changetmp = HTONL(config->dbit_timing);

    int recv_len = self_usb_sendmsg_ed1(self_usbhandle->usb_p, initcan_data, sizeof(initcan_data));

    if (recv_len < 0) {
        return NULL;
    }

    recv_len = self_usb_recvmsg_ed1(self_usbhandle->usb_p, recv_data, sizeof(recv_data));

    if (recv_len < 0) {
        return NULL;
    }

    if (recv_data[2] == 0xB0) {
        return NULL;
    }

    CAN_INFO_T *canhandle = (CAN_INFO_T *)calloc(1, sizeof(CAN_INFO_T));

    if (NULL == canhandle) {
        return NULL;
    }

    uint8_t filter_data[51] = {0};
    uint8_t itmp = 0;

    filter_data[0] = 0x11;
    filter_data[1] = 0x05;
    filter_data[2] = 0x30;
    filter_data[3] = 0x00;
    filter_data[4] = channel;

    /*设置扩展帧过滤器*/
    for (itmp = 0; itmp < config->Extend.num; itmp++) {
        filter_data[5] = itmp;
        filter_data[6] = config->Extend.filterDataExtend[itmp].frameType;
        filter_data[7] = config->Extend.filterDataExtend[itmp].filterType;
        Changetmp = (uint32_t *)&filter_data[8];
        *Changetmp = HTONL(config->Extend.filterDataExtend[itmp].ID1);
        Changetmp = (uint32_t *)&filter_data[12];
        *Changetmp = HTONL(config->Extend.filterDataExtend[itmp].ID2);
        recv_len = self_usb_sendmsg_ed1(self_usbhandle->usb_p, filter_data, sizeof(filter_data));

        if (recv_len < 0) {
            printf("Extend filter [%d] setting faild\n", itmp);
            continue;
        }

        recv_len = self_usb_recvmsg_ed1(self_usbhandle->usb_p, recv_data, sizeof(recv_data));

        if (recv_len < 0) {
            continue;
        }
    }

    /*设置标准帧过滤器*/
    for (itmp = 0; itmp < config->Standard.num; itmp++) {
        filter_data[5] = itmp;
        filter_data[6] = config->Standard.filterDataStandard[itmp].frameType;
        filter_data[7] = config->Standard.filterDataStandard[itmp].filterType;
        Changetmp = (uint32_t *)&filter_data[8];
        *Changetmp = HTONL(config->Standard.filterDataStandard[itmp].ID1);
        Changetmp = (uint32_t *)&filter_data[12];
        *Changetmp = HTONL(config->Standard.filterDataStandard[itmp].ID2);
        recv_len = self_usb_sendmsg_ed1(self_usbhandle->usb_p, filter_data, sizeof(filter_data));

        if (recv_len < 0) {
            printf("Standard filter [%d] setting faild\n", itmp);
            continue;
        }

        recv_len = self_usb_recvmsg_ed1(self_usbhandle->usb_p, recv_data, sizeof(recv_data));

        if (recv_len < 0) {
            continue;
        }
    }

    canhandle->usbinfo = self_usbhandle;
    canhandle->canindex = channel;
    memcpy(&canhandle->config, config, sizeof(iTek_CANFD_CHANNEL_INIT_CONFIG));

    return (CHANNEL_HANDLE)canhandle;
}

bool iTek_StartCAN(CHANNEL_HANDLE channel_handle)
{
    CAN_INFO_T *self_canhandle = (CAN_INFO_T *)channel_handle;

    if (self_canhandle == NULL) {
        return false;
    }

    uint8_t startcan_data[51] = {0};
    uint8_t recv_data[51] = {0};
    startcan_data[0] = 0x11;
    startcan_data[1] = 0x06;
    startcan_data[2] = 0x30;
    startcan_data[3] = self_canhandle->canindex;

    int recv_len = self_usb_sendmsg_ed1(self_canhandle->usbinfo->usb_p, startcan_data, sizeof(startcan_data));

    if (recv_len < 0) {
        return false;
    }

    recv_len = self_usb_recvmsg_ed1(self_canhandle->usbinfo->usb_p, recv_data, sizeof(recv_data));

    if (recv_len < 0) {
        return false;
    }

    if (recv_data[2] == 0xB0) {
        printf("[%s]--[%d]-错误码[%u]\n", __func__, __LINE__, recv_data[3]);
        return false;
    }

    return true;
}

uint32_t iTek_Receive(CHANNEL_HANDLE channel_handle, iTek_CANFD_Receive_Data *pReceive, uint32_t len, int wait_time)
{
    if (NULL == channel_handle) {
        return 0;
    }

    iTek_CANFD_Receive_Data tmp_data;
    CAN_INFO_T *self_canhandle = (CAN_INFO_T *)channel_handle;
    struct timeval start, stop, result;
    uint32_t real_num = 0;
    int used_num = 0;

    gettimeofday(&start, NULL);
    gettimeofday(&stop, NULL);

    for (real_num = 0; real_num < len;) {
        used_num = rbuf_used(self_canhandle->usbinfo->rbuf_handle[self_canhandle->canindex]);

        if (used_num >= (int)sizeof(iTek_CANFD_Receive_Data)) {
            memset(&tmp_data, 0, sizeof(iTek_CANFD_Receive_Data));
            rbuf_read(self_canhandle->usbinfo->rbuf_handle[self_canhandle->canindex], (u_char *)&tmp_data, sizeof(iTek_CANFD_Receive_Data));
            memcpy(&pReceive[real_num], &tmp_data, sizeof(iTek_CANFD_Receive_Data));
            real_num++;
        } else {
            if (0 < wait_time) {
                timersub(&stop, &start, &result);

                if (result.tv_sec * 1000000 + result.tv_usec < wait_time * 1000) {
                    usleep(10000);
                } else {
                    /*已到超时时间，跳出循环*/
                    break;
                }
            } else {
                /*无超时时间，一直等待，间隔10ms*/
                usleep(100000);
            }
        }

        gettimeofday(&stop, NULL);
    }

    return real_num;
}

uint32_t iTek_Transmit(CHANNEL_HANDLE channel_handle, iTek_CANFD_Transmit_Data *data, uint32_t len)
{
    if (NULL == channel_handle) {
        return 0;
    }

    CAN_INFO_T *self_canhandle = (CAN_INFO_T *)channel_handle;
    uint8_t data_temple[80] = {0};
    uint8_t recv_data[4] = {0};
    uint32_t success_num = 0;
    int datalen_standard = 0;
    int transmitdata_len = 0;

    if (self_canhandle->config.can_type == 0) {
        /*CAN*/
        if (len > 20) {
            return 0;
        }

        datalen_standard = 24;
    } else if (self_canhandle->config.can_type == 1) {
        /*CANFD*/
        if (len > 6) {
            return 0;
        }

        datalen_standard = 80;
    }

    transmitdata_len = 3 + len * datalen_standard;
    uint8_t *transmit_data = (uint8_t *)calloc(transmitdata_len, sizeof(uint8_t));

    if (NULL == transmit_data) {
        return 0;
    }

    transmit_data[0] = 0x21;
    transmit_data[1] = self_canhandle->config.can_type;
    transmit_data[2] = len;

    uint32_t *Changetmp;
    uint32_t i;

    for (i = 0; i < len; i++) {
        /*0-7时间戳 8 位预留 都是0*/
        data_temple[9] = self_canhandle->canindex;
        data_temple[9] = data_temple[9] | (data[i].send_type << 4);

        data_temple[10] = data[i].frame.cantype;
        //       data_temple[10] = self_canhandle->config.CANFDSpeedup;
        //       data_temple[10] = data_temple[10] | (self_canhandle->config.can_type << 1);

        data_temple[11] = data[i].frame.len;

        Changetmp = (uint32_t *)&data_temple[12];
        *Changetmp = HTONL(data[i].frame.can_id);

        if (self_canhandle->config.can_type == 0) {
            memcpy(data_temple + 16, data[i].frame.data, 8);
        } else if (self_canhandle->config.can_type == 1) {
            memcpy(data_temple + 16, data[i].frame.data, 64);
        }

        memcpy(transmit_data + 3 + i * datalen_standard, data_temple, datalen_standard);
        memset(data_temple, 0, sizeof(data_temple));
    }

    int recv_len = self_usb_sendmsg_ed2(self_canhandle->usbinfo->usb_p, transmit_data, transmitdata_len);
    free(transmit_data);

    recv_len = self_usb_recvmsg_ed2(self_canhandle->usbinfo->usb_p, recv_data, sizeof(recv_data));
    success_num = (uint32_t)recv_data[3];

    return success_num;
}

uint32_t iTek_GetReceiveNum(CHANNEL_HANDLE channel_handle)
{
    CAN_INFO_T *self_canhandle = (CAN_INFO_T *)channel_handle;

    if (self_canhandle == NULL) {
        return 0;
    }

    uint32_t recvnum = 0;
    recvnum = rbuf_used(self_canhandle->usbinfo->rbuf_handle[self_canhandle->canindex]) / sizeof(iTek_CANFD_Receive_Data);
    return recvnum;
}

void iTek_ClearBuffer(CHANNEL_HANDLE channel_handle)
{
    CAN_INFO_T *self_canhandle = (CAN_INFO_T *)channel_handle;

    if (self_canhandle == NULL) {
        return;
    }

    rbuf_clear(self_canhandle->usbinfo->rbuf_handle[self_canhandle->canindex]);

    return;
}

void *iTek_Authenticate(DEVICE_HANDLE usbhandle, uint16_t key_type, uint8_t *pStr, int len, unsigned char *dStr)
{
    USB_INFO_T *self_usbhandle = (USB_INFO_T *)usbhandle;

    if (self_usbhandle == NULL) {
        return NULL;
    }

    if ((len != 16) || (key_type != 0)) {
        return NULL;
    }

    uint8_t auth_data[51];
    uint8_t recv_data[51] = {0};

    auth_data[0] = 0x11;
    auth_data[1] = 0xe0;
    auth_data[2] = 0x30;

    auth_data[3] = 0x00;
    auth_data[4] = 0x00;
    auth_data[5] = 0x00;
    auth_data[6] = 0x00;

    memcpy(auth_data + 7, pStr, len);
    int recv_len = self_usb_sendmsg_ed1(self_usbhandle->usb_p, auth_data, sizeof(auth_data));

    if (recv_len < 0) {
        return NULL;
    }

    recv_len = self_usb_recvmsg_ed1(self_usbhandle->usb_p, recv_data, sizeof(recv_data));

    if (recv_len < 0) {
        return false;
    }

    memcpy(dStr, recv_data + 7, len);

    return NULL;
}

bool iTek_isConnected(DEVICE_HANDLE usbhandle)
{
    USB_INFO_T *self_usbhandle = (USB_INFO_T *)usbhandle;

    if (NULL == self_usbhandle) {
        return false;
    }

    uint8_t check_data[51] = {0};
    uint8_t recv_data[51] = {0};

    check_data[0] = 0x11;
    check_data[1] = 0x08;
    check_data[2] = 0x30;
    check_data[3] = 0xaa;

    int recv_len = self_usb_sendmsg_ed1(self_usbhandle->usb_p, check_data, sizeof(check_data));

    if (recv_len < 0) {
        return false;
    }

    recv_len = self_usb_recvmsg_ed1(self_usbhandle->usb_p, recv_data, sizeof(recv_data));

    if (recv_len < 0) {
        return false;
    }

    return true;
}
