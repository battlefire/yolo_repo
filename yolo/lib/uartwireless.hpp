#ifndef _H_HEAD_H
#define _H_HEAD_H
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include "uart.h"
#include <pthread.h>
#include <sys/time.h>
#include <vector>

#define THIS_DEVICE 0x00
#define MAX_DATALEN 256
#define OBJ_DESCRIBE_ELEMENT 8
#define MAX_OBJ_NUM 32
#define FRAME_DATA_LEN 26

using namespace std;
typedef struct
{
    float modul_id;
    float type;
    float x_loca;
    float y_loca;
    float len;
    float wide;
    float speed;
    float direction;

} Datatample;

void DataUnpack(int data_num, float *data_buf);
void updata(int num, vector<Datatample> &temp);
unsigned char crc_8(unsigned char *data, int length);
int rdm_recv(pid_t fd, int *device_num);
void DataPack(int fun_num, int device_num, int data_len, float *data_buf);
void rdm_send(pid_t fd, int fun_num, int device_num, int data_num, float *data_buf);
void init(pid_t fd);

typedef struct
{
    int data_num;
    float data_buf[MAX_DATALEN];

} Data;

#endif
