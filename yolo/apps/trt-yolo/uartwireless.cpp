#include "uartwireless.hpp"

unsigned char sendpack[2048];
unsigned char recvpack[2048];
int all_device[8] = {0};

Data globle;

using namespace std;

void updata(int num, vector<Datatample> &temp)
{
    int i;
    int temp_num;
    int loopnum;
    float temp_buf[MAX_DATALEN];
    if (num > MAX_OBJ_NUM)
    {
        temp_num = MAX_OBJ_NUM;
    }
    else
    {
        temp_num = num;
    }

    loopnum = temp_num * OBJ_DESCRIBE_ELEMENT;
    globle.data_num = temp_num;
    memcpy(&temp_buf[0], &temp[0], temp_num * sizeof(Datatample));
    for (i = 0; i < loopnum; i++)
    {
        globle.data_buf[i] = temp_buf[i];
    }
}

unsigned char crc_8(unsigned char *data, int length)
{
    unsigned char i;
    unsigned char crc = 0; // Initial value
    while (length--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc ^ 0x55;
}

int rdm_recv(pid_t fd, int *device_num)
{
    printf("-------------recv  begain\n");
    unsigned char buf[4];
    unsigned char head[3] = {0};
    unsigned char crc;
    unsigned char crc_recv;

    int length;
    int device_id;
    int fun_num;

    uart_recv(fd, &buf[0], 1, NULL);
    if (buf[0] == 0xfe)
    {
        uart_recv(fd, &buf[1], 1, NULL);
        if (buf[1] == 0xfd)
        {
            uart_recv(fd, &buf[2], 1, NULL);
            if (buf[2] != 0xfc)
            {
                bzero(&buf[0], sizeof(buf));
                return -1;
            }
        }
        else
        {
            bzero(&buf[0], sizeof(buf));
            return -1;
        }
    }
    else
    {
        bzero(&buf[0], sizeof(buf));

        return -1;
    }

    uart_recv(fd, &head[0], 3, NULL);

    fun_num = head[0] & 0x3f;
    length = ((head[0] >> 6) & 0x3) + ((head[1] & 0x3f) << 2);
    device_id = head[2] & 0xf;

    uart_recv(fd, &recvpack[0], length, NULL);
    unsigned char tempbuf[length + 3];
    memcpy(&tempbuf[0], &head[0], 3);
    memcpy(&tempbuf[3], &recvpack[0], length);
    crc = crc_8(&tempbuf[0], length + 3);

    uart_recv(fd, &crc_recv, 1, NULL);

    if (device_id > 4 || fun_num > 0x4)
    {
        bzero(&recvpack[0], sizeof(recvpack));

        return -1;
    }
    /*
    if (fun_num == 1 || fun_num == 3)
    {
        if (length != 0)
        {
            bzero(&recvpack[0], sizeof(recvpack));

            return -1;
        }
    }
    */

    //        printf("the fun :%#x",fun_num);
    //        printf("the len :%#x",length);
    //        printf("the device id:%#x",device_id);

#if 1
    if (crc != crc_recv)
    {
        bzero(&recvpack[0], sizeof(recvpack));
        return -1;
    }
#endif
    switch (fun_num)
    {
    case 1:
        *device_num = device_id;
        break;
    case 2:
        return 2;
        break;

    case 3:
        *device_num = device_id;
        break;
    case 4:
        return 4;
        break;
    }

    bzero(&recvpack[0], sizeof(recvpack));

    return fun_num;

    printf("----------------recv end\n");
}

void DataPack(int fun_num, int device_num, int data_len, float *data_buf) //#1: data of car #2:num of car #3: struct head
{
    printf("----------------begain pack\n");
    int ret;
    unsigned char comunicate_data;
    float comunicate_data_float;

    ret = sizeof(data_buf) / OBJ_DESCRIBE_ELEMENT;
    sendpack[0] = 0xfe;
    sendpack[1] = 0xfd;
    sendpack[2] = 0xfc;
    unsigned char crc;
    switch (fun_num)
    {
    case 1:
#if 0

            sendpack[3] = fun_num&0x3f;
            sendpack[4] = 0;
            sendpack[5] = device_num &0xf;
            sendpack[5] = sendpack[5] | (0x1<<4);
            sendpack[6] = 0;
#endif
        break;
    case 2:
#if 1

        sendpack[3] = fun_num & 0x3f;
        sendpack[4] = 0;
        sendpack[5] = device_num & 0xf;
        sendpack[5] = sendpack[5] | (0x1 << 4);
        crc = crc_8(&sendpack[3], 3);
        sendpack[6] = crc;
#endif
        break;
    case 3:
#if 0

            sendpack[3] = fun_num&0x3f;
            sendpack[4] = 0;
            sendpack[5] = device_num &0xf;
            sendpack[5] = sendpack[5] | (0x1<<4);
            sendpack[6] = 0;
#endif
    case 4:
#if 1

        sendpack[3] = fun_num & 0x3f;
        ret = (data_len & 0x3) << 6;
        sendpack[3] = (sendpack[3] | ret);

        sendpack[4] = data_len >> 2 & 0x3f;
        sendpack[5] = device_num & 0xf;
        sendpack[5] = sendpack[5] | (0x1 << 4);
        ret = (data_len - 1) / FRAME_DATA_LEN;

        sendpack[6] = ret;
        for (int i = 0; i < ret; i++)
        {
            comunicate_data = (int)data_buf[OBJ_DESCRIBE_ELEMENT * i];
            sendpack[7 + FRAME_DATA_LEN * i] = comunicate_data;
            comunicate_data = (int)data_buf[1 + OBJ_DESCRIBE_ELEMENT * i];
            sendpack[8 + FRAME_DATA_LEN * i] = comunicate_data;

            comunicate_data_float = data_buf[2 + OBJ_DESCRIBE_ELEMENT * i];
            memcpy(&sendpack[9 + FRAME_DATA_LEN * i], &comunicate_data_float, sizeof(comunicate_data_float));
            comunicate_data_float = data_buf[3 + OBJ_DESCRIBE_ELEMENT * i];
            memcpy(&sendpack[13 + FRAME_DATA_LEN * i], &comunicate_data_float, sizeof(comunicate_data_float));
            comunicate_data_float = data_buf[4 + OBJ_DESCRIBE_ELEMENT * i];

            memcpy(&sendpack[17 + FRAME_DATA_LEN * i], &comunicate_data_float, sizeof(comunicate_data_float));
            comunicate_data_float = data_buf[5 + OBJ_DESCRIBE_ELEMENT * i];

            memcpy(&sendpack[21 + FRAME_DATA_LEN * i], &comunicate_data_float, sizeof(comunicate_data_float));
            comunicate_data_float = data_buf[6 + OBJ_DESCRIBE_ELEMENT * i];

            memcpy(&sendpack[25 + FRAME_DATA_LEN * i], &comunicate_data_float, sizeof(comunicate_data_float));
            comunicate_data_float = data_buf[7 + OBJ_DESCRIBE_ELEMENT * i];

            memcpy(&sendpack[29 + FRAME_DATA_LEN * i], &comunicate_data_float, sizeof(comunicate_data_float));
        }

        crc = crc_8(&sendpack[3], data_len + 3);
        sendpack[6 + data_len] = crc;

#endif
        break;
    }
    printf("----------------end  pack\n");
}

void rdm_send(pid_t fd, int fun_num, int device_num, int data_num, float *data_buf)
{
    printf("-----------------begain send\n");

    int data_len = FRAME_DATA_LEN * data_num + 1;
    printf("%d\n", data_len);
    bzero(&sendpack, sizeof(sendpack));
    if (fun_num == 2)
    {
        DataPack(fun_num, device_num, 0, NULL);
    }
    if (fun_num == 4)
    {
        DataPack(fun_num, device_num, data_len, data_buf);
    }
    uart_send(fd, &sendpack[0], 8 + data_num * FRAME_DATA_LEN);
    bzero(&sendpack[0], sizeof(sendpack));
    printf("-----------------end  send\n");
}

void DataUnpack(int data_num, float *data_buf)
{
    float data_buf_temp[OBJ_DESCRIBE_ELEMENT];
    float real_data;
    printf("----------------begin unpack\n");

    data_num = recvpack[0];
    printf("data_num:%d\n", data_num);
    for (int i = 0; i < data_num; i++)
    {
        real_data = recvpack[1 + FRAME_DATA_LEN * i];
        data_buf_temp[0] = real_data;
        real_data = recvpack[2 + FRAME_DATA_LEN * i];
        data_buf_temp[1] = real_data / 10;
        real_data = recvpack[3 + FRAME_DATA_LEN * i];
        data_buf_temp[2] = real_data / 10;

        real_data = (recvpack[4 + FRAME_DATA_LEN * i] + ((recvpack[5 + FRAME_DATA_LEN * i] & 0xf) * 256));
        data_buf_temp[3] = real_data / 10 - 200;
        real_data = ((recvpack[6 + FRAME_DATA_LEN * i] << 4) + ((recvpack[5 + FRAME_DATA_LEN * i] & 0xf0) / 16));

        data_buf_temp[4] = real_data / 10 - 200;
        real_data = (recvpack[7 + FRAME_DATA_LEN * i] + ((recvpack[8 + FRAME_DATA_LEN * i] & 0x3) * 256));

        data_buf_temp[5] = real_data / 10 - 50;
        real_data = (recvpack[8 + FRAME_DATA_LEN * i] & 0xfc) / 2;
        real_data = (((recvpack[8 + FRAME_DATA_LEN * i] & 0xfc) / 4) + ((recvpack[9 + FRAME_DATA_LEN * i] & 0xf) * 64));

        data_buf_temp[6] = real_data / 10 - 50;
        real_data = (recvpack[9 + FRAME_DATA_LEN * i] & 0xf0) / 16;
        data_buf_temp[7] = real_data;

#if 1

        memcpy(&data_buf[OBJ_DESCRIBE_ELEMENT * i], &data_buf_temp[0], sizeof(data_buf_temp));
        bzero(&data_buf_temp[0], sizeof(data_buf_temp));
#endif
    }
    printf("----------------end unpack\n");
    return;
}

void init(pid_t fd)
{
    int speed = 115200;
    int flow_ctrl = 0;
    int databits = 8;
    int stopbits = 1;
    int parity = 'N';
    int errnum;
    errnum = uart_init(fd, speed, flow_ctrl, databits, stopbits, parity);
    if (errnum < 0)
    {
        perror("uart_init()\n");
        return;
    }
}
