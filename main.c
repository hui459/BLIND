/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-05     RT-Thread    first version
 */
#include "main.h"

#include <rtthread.h>
#include <rthw.h>
#include <finsh.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <stdbool.h>
#include <rtdevice.h>
#include <sensor.h>
#include <board.h>
#include <stdio.h>
#include "icm20608.h"
#include <math.h>
#include "drv_soft_i2c.h"
#include <string.h>
#include <rtdef.h>


/* 定义 I2C 总线名称 */
#define I2C_BUS_NAME "i2c2"
#define ICM20608_ADDR 0x68 /* slave address, ad0 set 0 */
icm20608_device_t dev = RT_NULL;
rt_err_t result;

/* 定义 i2c_bus_init 函数 */
rt_err_t i2c_bus_init(const char *bus_name);


#define LED GET_PIN(E,11)
#define Trig GET_PIN(A,5)
#define Echo GET_PIN(A,6)
#define syn6288TX GET_PIN(B,10)
#define syn6288RX GET_PIN(B,11)
#define ICM20608_SDA_PIN   GET_PIN(F, 0)
#define ICM20608_SCL_PIN   GET_PIN(F, 1)

#include <rtthread.h>
#include <rthw.h>
#include <string.h>
//#include "usart.h" // 包含串口驱动头文件
void MX_USART3_UART_Init(void);

#define uchar unsigned char
#define uint unsigned int
#define _ICONV_H_
#define _ICONV_H_

/* 定义MQTT相关参数 */
#define MQTT_BROKER "mqtt://mqtt.tlink.io:1883" // Tlink MQTT服务器地址和端口
#define CLIENT_ID "HD3F10086M021HZ6"             // 客户端ID，需在Tlink平台注册设备获取
#define TOPIC_PUBLISH "HD3F10086M021HZ6"    // 发布主题
#define TOPIC_SUBSCRIBE "HD3F10086M021HZ6" // 订阅主题
UART_HandleTypeDef huart2;

/**************芯片设置命令*********************/
uchar SYN_StopCom[] = {0xFD, 0X00, 0X02, 0X02, 0XFD}; //停止合成
uchar SYN_SuspendCom[] = {0XFD, 0X00, 0X02, 0X03, 0XFC}; //暂停合成
uchar SYN_RecoverCom[] = {0XFD, 0X00, 0X02, 0X04, 0XFB}; //恢复合成
uchar SYN_ChackCom[] = {0XFD, 0X00, 0X02, 0X21, 0XDE}; //状态查询
uchar SYN_PowerDownCom[] = {0XFD, 0X00, 0X02, 0X88, 0X77}; //进入POWER DOWN 状态命令

// UART 句柄声明
extern UART_HandleTypeDef huart3;

// 初始化串口 USART3
void init_usart3(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;  // 设置波特率为 9600
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

// 串口发送一字节数据
void UART_SendByte(uchar DAT) {
    HAL_UART_Transmit(&huart3, &DAT, 1, 1000);
}

// 串口发送字符串数据
void UART_SendString(uchar *DAT, uchar len) {
    HAL_UART_Transmit(&huart3, DAT, len, 1000);
}

// 构造语音合成命令帧
void SYN_FrameInfo(uchar Music, uchar *HZdata) {
    uchar Frame_Info[50];
    uchar HZ_Length;
    uchar ecc = 0; // 定义校验字节
    uint i = 0;
    HZ_Length = strlen((char*)HZdata); // 需要发送文本的长度

    Frame_Info[0] = 0xFD; // 构造帧头FD
    Frame_Info[1] = 0x00; // 构造数据区长度的高字节
    Frame_Info[2] = HZ_Length + 3; // 构造数据区长度的低字节
    Frame_Info[3] = 0x01; // 构造命令字：合成播放命令
    Frame_Info[4] = 0x01 | Music << 4; // 构造命令参数：背景音乐设定

    for (i = 0; i < 5; i++) {
        ecc = ecc ^ Frame_Info[i]; // 对发送的字节进行异或校验
    }

    for (i = 0; i < HZ_Length; i++) {
        ecc = ecc ^ HZdata[i]; // 对发送的字节进行异或校验
    }

    memcpy(&Frame_Info[5], HZdata, HZ_Length);
    Frame_Info[5 + HZ_Length] = ecc;

    UART_SendString(Frame_Info, 5 + HZ_Length + 1);
}

// 发送语音合成命令
void YS_SYN_Set(uchar *Info_data) {
    uchar Com_Len = strlen((char*)Info_data);
    UART_SendString(Info_data, Com_Len);
}



//光照传感器
// 定义一个全局变量来存储光强值
int light_value;

bool led_switch; // 定义全局变量
// 定义一个函数来获取光强值

float get_light_value(void)
{
    return light_value;
}

// 定义一个线程来读取传感器数据并更新全局变量
static void bh1750_thread_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data data;
    rt_size_t res;

    /* 查找 bh1750 传感器 */
    dev = rt_device_find("li_bh1750");
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device: li_bh1750\n");
        return;
    }

    /* 以只读模式打开 bh1750 */
    if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open device failed!");
        return;
    }



    while (1)
    {
        /* 从传感器读取一个数据 */
        res = rt_device_read(dev, 0, &data, 1);

        if (1 != res)
        {
            rt_kprintf("read data failed! size is %d", res);
        }
        else
        {
            // 假设 data.data.light 的单位是 1 lux
            int light = data.data.light/10;

            // 打印光强值
            rt_kprintf("light: %d lux\n", light);


     //        更新全局变量
            light_value = light;
        }
        //光强传感器控制LED的开关
//        rt_pin_mode(LED,PIN_MODE_OUTPUT);
//        bool led_switch = (light_value < 100);
//         if(light_value < 100 ){
//         rt_pin_write(LED,PIN_HIGH);
//         }else{
//         rt_pin_write(LED,PIN_LOW);
//         }
         rt_pin_mode(LED, PIN_MODE_OUTPUT);
         // 定义一个布尔变量表示 LED 开关状态
      //   bool led_switch = (light_value < 100);
         if (light_value < 100) {
             led_switch = 1;
             rt_pin_write(LED, PIN_HIGH);
         } else {
             led_switch = 0;
             rt_pin_write(LED, PIN_LOW);
         }
         rt_kprintf("LED switch: %d\n", led_switch);
         rt_thread_mdelay(1000);

    }
}


//超声波

/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-05-09     shany       the first version
 */

#include <stdlib.h>
#include <rtthread.h>

#include "board.h"
#include "sensor.h"
#include "sensor_hc_sr04.h"

/* Modify this pin according to the actual wiring situation */
#define SR04_TRIG_PIN GET_PIN(A, 5)
#define SR04_ECHO_PIN GET_PIN(A, 6)

int sr04_read_distance_sample(void);
int rt_hw_sr04_port(void);

struct rt_sensor_data sensor_data; // 定义全局变量存储传感器数据
int distance = 0; // 定义全局变量存储距离值

static void sr04_read_distance_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
   // struct rt_sensor_data sensor_data;
    rt_size_t res;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL) {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        rt_kprintf("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);

    while (1) {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1) {
            rt_kprintf("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else {

            distance = sensor_data.data.proximity / 10; // 更新全局变量distance
            rt_kprintf("distance:%3d cm \n", distance);
            //rt_kprintf("distance:%3d.%dcm, timestamp:%5d\n", sensor_data.data.proximity / 10, sensor_data.data.proximity % 10, sensor_data.timestamp);
        }

        rt_thread_mdelay(3000);
    }
}

int sr04_read_distance_sample(void)
{
    rt_thread_t sr04_thread;

    sr04_thread = rt_thread_create("sr04",
                                   sr04_read_distance_entry,
                                   "pr_sr04",
                                   1024,
                                   RT_THREAD_PRIORITY_MAX / 2,
                                   20);
    if (sr04_thread != RT_NULL) {
        rt_thread_startup(sr04_thread);
    }

    return RT_EOK;
}

struct rt_sensor_config sensor_cfg;

int rt_hw_sr04_port(void)
{
    struct rt_sensor_config cfg;
    rt_base_t pins[2] = {SR04_TRIG_PIN, SR04_ECHO_PIN};

    cfg.intf.dev_name = "timer3";
    cfg.intf.user_data = (void *)pins;
    rt_hw_sr04_init("sr04", &cfg);

    return RT_EOK;
}
static void sr04_read_distance_entry(void *parameter);


/****************** 修改后的串口4相关代码开始 ******************/
int a,b;
#define UART4_DEVICE_NAME    "uart4"    // 串口4设备名称
#define MAX_RECV_LEN 64  // 最大接收长度，假设每次传输两次数据，每次32字节

// 串口4接收线程入口函数
static void uart4_thread_entry(void *parameter)
{
    rt_device_t serial = RT_NULL;
    char recv_buf[MAX_RECV_LEN];
    rt_size_t recv_len;

    // 查找串口设备
    serial = rt_device_find(UART4_DEVICE_NAME);
    if (!serial) {
        rt_kprintf("find %s failed!\n", UART4_DEVICE_NAME);
        return;
    }

    // 以中断接收及轮询发送模式打开串口设备
    if (rt_device_open(serial, RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
        rt_kprintf("open %s failed!\n", UART4_DEVICE_NAME);
        return;
    }

    while (1) {
        // 清空接收缓冲区
        rt_memset(recv_buf, 0, sizeof(recv_buf));

        // 从串口读取数据
        recv_len = rt_device_read(serial, 0, recv_buf, sizeof(recv_buf) - 1);

        if (recv_len > 0) {
            // 打印接收到的 ASCII 码值到 RTT 终端
            rt_kprintf("UART4 received ASCII values: ");
            for (rt_size_t i = 0; i < recv_len; i++) {
                rt_kprintf("%d \n", (unsigned char)recv_buf[i]); // 打印 ASCII 码值
            }
//            rt_kprintf("first data = %d",recv_buf[0]);
//            rt_kprintf("second data = %d",recv_buf[1]);
            a = recv_buf[0];
            b = recv_buf[1];
            rt_kprintf("\n");
        }

        rt_thread_mdelay(10);  // 适当延时，防止 CPU 占用过高
    }
}
// 初始化串口4
static int uart4_init(void)
{
    rt_thread_t thread = RT_NULL;

    // 创建串口4接收线程
    thread = rt_thread_create("uart4_rx",
                              uart4_thread_entry,
                              RT_NULL,
                              1024,
                              3,
                              10);

    if (thread != RT_NULL) {
        rt_thread_startup(thread);
        rt_kprintf("UART4 thread created successfully.\n");
        return RT_EOK;
    } else {
        rt_kprintf("Failed to create UART4 thread.\n");
        return -RT_ERROR;
    }
}



/****************** 修改后的串口4相关代码结束 ******************/



void voice_thread_entry(void *parameter)
{
    // 选择背景音乐2。(0：无背景音乐  1-15：背景音乐可选)
    // m[0~16]:0背景音乐为静音，16背景音乐音量最大
    // v[0~16]:0朗读音量为静音，16朗读音量最大
    // t[0~5]:0朗读语速最慢，5朗读语速最快
    // 其他不常用功能请参考数据手册
    // SYN_FrameInfo(0x00, (uchar*)"[v8][t7]欢迎使用智能导盲系统");

    // 初始化串口 USART3
    init_usart3();
    // 欢迎使用智能导盲系统
    uint8_t data[] = {0xFD, 0x00, 0x1D, 0x01, 0x01, 0xBB, 0xB6, 0xD3, 0xAD, 0xCA, 0xB9, 0xD3, 0xC3, 0xB6, 0xE0, 0xB3, 0xA1, 0xBE, 0xB0, 0xD6, 0xC7, 0xC4, 0xDC, 0xB5, 0xBC, 0xC3, 0xA4, 0xCF, 0xB5, 0xCD, 0xB3, 0xD9};
    // 发送数据
    UART_SendString(data, sizeof(data));
    rt_thread_mdelay(10000); // 延时 10 秒

    while (1) // 无限循环，持续检测距离并播报
    {
        if (distance > 50 && distance < 100)
        {
            // 前方障碍物距离为1米，请注意安全
            uint8_t FAR[] = {0xFD, 0x00, 0x21, 0x01, 0x01, 0xC7, 0xB0, 0xB7, 0xBD, 0xD2, 0xBB, 0xC3, 0xD7, 0xB4, 0xA6, 0xD3, 0xD0, 0xD5, 0xCF, 0xB0, 0xAD, 0xCE, 0xEF, 0xA3, 0xAC, 0xC7, 0xEB, 0xD7, 0xA2, 0xD2, 0xE2, 0xB0, 0xB2, 0xC8, 0xAB, 0xEC};
            UART_SendString(FAR, sizeof(FAR));
            rt_thread_mdelay(10000); // 延时 10 秒
            // 前方距离在0.5米以内，及时避让
        }
        else if (distance < 50)
        {
            uint8_t IN[] = {0xFD, 0x00, 0x20, 0x01, 0x01, 0xC7, 0xB0, 0xB7, 0xBD, 0xBE, 0xE0, 0xC0, 0xEB, 0xD4, 0xDA, 0x30, 0x2E, 0x35, 0xC3, 0xD7, 0xD2, 0xD4, 0xC4, 0xDA, 0xA3, 0xAC, 0xBC, 0xB0, 0xCA, 0xB1, 0xB1, 0xDC, 0xC8, 0xC3, 0xE2};
            UART_SendString(IN, sizeof(IN));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        rt_thread_mdelay(1000); // 添加适当的延时，避免高频循环
        if(a ==250 && b == 245){
            //正前方检测到红灯,请注意安全
            uint8_t red0[] = {0xFD,0x00,0x1F,0x01,0x01,0xD5,0xFD,0xC7,0xB0,0xB7,0xBD,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xBA,0xEC,0xB5,0xC6,0xA3,0xAC,0xC7,0xEB,0xD7,0xA2,0xD2,0xE2,0xB0,0xB2,0xC8,0xAB,0x9D};
            UART_SendString(red0, sizeof(red0));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a ==250 && b == 250){
            //左前方检测到红灯,请注意安全
            uint8_t red1[] ={0xFD,0x00,0x1F,0x01,0x01,0xD7,0xF3,0xC7,0xB0,0xB7,0xBD,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xBA,0xEC,0xB5,0xC6,0xA3,0xAC,0xC7,0xEB,0xD7,0xA2,0xD2,0xE2,0xB0,0xB2,0xC8,0xAB,0x91};
            UART_SendString(red1, sizeof(red1));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a ==250 && b == 255){
            //右前方检测到红灯,请注意安全
            uint8_t red2[]={0xFD,0x00,0x1F,0x01,0x01,0xD3,0xD2,0xC7,0xB0,0xB7,0xBD,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xBA,0xEC,0xB5,0xC6,0xA3,0xAC,0xC7,0xEB,0xD7,0xA2,0xD2,0xE2,0xB0,0xB2,0xC8,0xAB,0xB4};
            UART_SendString(red2, sizeof(red2));
             rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==249 && b == 245){
            //正前方检测到绿灯,请通行
            uint8_t green0[]={0xFD,0x00,0x1B,0x01,0x01,0xD5,0xFD,0xC7,0xB0,0xB7,0xBD,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xC2,0xCC,0xB5,0xC6,0xA3,0xAC,0xC7,0xEB,0xCD,0xA8,0xD0,0xD0,0x80};
            UART_SendString(green0, sizeof(green0));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==249 && b == 250){
            //左前方检测到绿灯,请通行
            uint8_t green1[] = {0xFD,0x00,0x1B,0x01,0x01,0xD7,0xF3,0xC7,0xB0,0xB7,0xBD,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xC2,0xCC,0xB5,0xC6,0xA3,0xAC,0xC7,0xEB,0xCD,0xA8,0xD0,0xD0,0x8C};
            UART_SendString(green1, sizeof(green1));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==249 && b == 255){
            //右前方检测到绿灯,请通行
            uint8_t green2[]={0xFD,0x00,0x1B,0x01,0x01,0xD3,0xD2,0xC7,0xB0,0xB7,0xBD,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xC2,0xCC,0xB5,0xC6,0xA3,0xAC,0xC7,0xEB,0xCD,0xA8,0xD0,0xD0,0xA9};
            UART_SendString(green2, sizeof(green2));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==213 && b == 245){
            //检测到正前方有斑马线
            uint8_t walk0[]={0xFD,0x00,0x17,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD5,0xFD,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xB0,0xDF,0xC2,0xED,0xCF,0xDF,0xE4};
            UART_SendString(walk0, sizeof(walk0));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==213 && b == 250){
            //检测到左前方有斑马线
            uint8_t walk1[]={ 0xFD,0x00,0x17,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD7,0xF3,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xB0,0xDF,0xC2,0xED,0xCF,0xDF,0xE8};
            UART_SendString(walk1, sizeof(walk1));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==213 && b == 255){
            //检测到右前方有斑马线
            uint8_t walk2[]= {0xFD,0x00,0x17,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD3,0xD2,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xB0,0xDF,0xC2,0xED,0xCF,0xDF,0xCD};
            UART_SendString(walk2, sizeof(walk2));
            rt_thread_mdelay(10000); // 延时 10 秒
        }
        if(a==192 && b == 245){
            //检测到正前方有盲道
            uint8_t blind0[] = {0xFD,0x00,0x15,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD5,0xFD,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xC3,0xA4,0xB5,0xC0,0xA4};
            UART_SendString(blind0, sizeof(blind0));
            rt_thread_mdelay(10000);
        }
        if(a==192 && b == 250){
            //检测到左前方有盲道
            uint8_t blind1[] ={0xFD,0x00,0x15,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD7,0xF3,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xC3,0xA4,0xB5,0xC0,0xA8};
            UART_SendString(blind1, sizeof(blind1));
            rt_thread_mdelay(10000);
        }
        if(a==192 && b == 255){
            //检测到右前方有盲道
            uint8_t blind2[] ={0xFD,0x00,0x15,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD3,0xD2,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xC3,0xA4,0xB5,0xC0,0x8D};
            UART_SendString(blind2, sizeof(blind2));
            rt_thread_mdelay(10000);
        }
    }
}


#define THRESHOLD_ACCEL 20000 // 加速度阈值，可以根据实际情况调整
#define THRESHOLD_GYRO 15000   // 角速度阈值，可以根据实际情况调整
// 定义线程入口函数
rt_int16_t accel_mag; // 加速度向量的大小
rt_int16_t gyro_mag;  // 角速度向量的大小
void fall_detection_thread_entry(void *parameter)
{
    icm20608_device_t dev = RT_NULL;
    const char *i2c_bus_name = "i2c2";
    int count = 0;
    rt_err_t result;
    rt_bool_t fall_detected = RT_FALSE; // 声明 fall_detected 变量

    /* 初始化 ICM20608 传感器 */
    dev = icm20608_init(i2c_bus_name);
    if (dev == RT_NULL)
    {
        LOG_E("The sensor initializes failure");
        return;
    }
    else
    {
        LOG_D("The sensor initializes success");
    }

    /* 对 ICM20608 进行零值校准 */
    result = icm20608_calib_level(dev, 10);
    if (result == RT_EOK)
    {
        LOG_D("The sensor calibrates success");
        LOG_D("accel_offset: X%6d Y%6d Z%6d", dev->accel_offset.x, dev->accel_offset.y, dev->accel_offset.z);
        LOG_D("gyro_offset : X%6d Y%6d Z%6d", dev->gyro_offset.x, dev->gyro_offset.y, dev->gyro_offset.z);
    }
    else
    {
        LOG_E("The sensor calibrates failure");
        icm20608_deinit(dev);
        return;
    }

    /* 循环读取传感器数据 */
    while (count++ < 100)
    {
        rt_int16_t accel_x, accel_y, accel_z;
        rt_int16_t gyros_x, gyros_y, gyros_z;


        /* 读取三轴加速度 */
        result = icm20608_get_accel(dev, &accel_x, &accel_y, &accel_z);
        if (result == RT_EOK)
        {
            LOG_D("current accelerometer: accel_x%6d, accel_y%6d, accel_z%6d",
                  accel_x, accel_y, accel_z);
        }
        else
        {
            LOG_E("The sensor does not work");
            break;
        }

        /* 读取三轴陀螺仪 */
        result = icm20608_get_gyro(dev, &gyros_x, &gyros_y, &gyros_z);
        if (result == RT_EOK)
        {
            LOG_D("current gyroscope : gyros_x%6d, gyros_y%6d, gyros_z%6d", gyros_x, gyros_y, gyros_z);
            rt_kprintf("current gyroscope : gyros_x%6d, gyros_y%6d, gyros_z%6d", gyros_x, gyros_y, gyros_z);
        }
        else
        {
            LOG_E("The sensor does not work");
            break;
        }
        // 计算加速度向量的大小（模长）
               accel_mag = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
               // 计算角速度向量的大小（模长）
               gyro_mag = sqrt(gyros_x * gyros_x + gyros_y * gyros_y + gyros_z * gyros_z);

               // 判断是否可能摔倒
               if (accel_mag > THRESHOLD_ACCEL || gyro_mag > THRESHOLD_GYRO)
               {
                   fall_detected = RT_TRUE;
                   LOG_W("Fall detected! Accelerometer magnitude: %d, Gyroscope magnitude: %d", accel_mag, gyro_mag);
                   // 可以在这里添加摔倒后的处理逻辑，比如发送警报、保存数据等
                   //检测到用户摔倒，及时查看
//                   uint8_t she[] = {0xFD,0x00,0x1B,0x01,0x01,0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0xD3,0xC3,0xBB,0xA7,0xCB,0xA4,0xB5,0xB9,0xA3,0xAC,0xBC,0xB0,0xCA,0xB1,0xB2,0xE9,0xBF,0xB4,0xA9};
//                   UART_SendString(she, sizeof(she));
//                   rt_thread_mdelay(10000); // 延时 10 秒

               }
               else
               {
                   fall_detected = RT_FALSE;
               }

               LOG_D("current accelerometer: accel_x%6d, accel_y%6d, accel_z%6d, magnitude: %d",
                     accel_x, accel_y, accel_z, accel_mag);
               LOG_D("current gyroscope : gyros_x%6d, gyros_y%6d, gyros_z%6d, magnitude: %d",
                     gyros_x, gyros_y, gyros_z, gyro_mag);

               rt_thread_mdelay(10000); // 延时 10 秒
           }

}


// 4G
static void uart_send_thread_entry(void *parameter)
{
    rt_device_t uart2_device;
//    rt_uint8_t led_switch = 1;     // 定义 LED 开关值
//    rt_uint16_t light_value = 10;  // 定义光照值
//    rt_uint16_t distance = 10;     // 定义距离值
//    rt_int16_t accel_mag = 10;     // 定义加速度值
//    rt_int16_t gyro_mag = 10;      // 定义陀螺仪值

    while (1)
    {
        // 构建 JSON 数据字符串
        char json_data[256];
        rt_sprintf(json_data, "{\"sensorDatas\":[{\"flag\":\"light\",\"switcher\":%d},{\"flag\":\"GQ\",\"value\":%d},{\"flag\":\"distance\",\"value\":%d},{\"flag\":\"acc\",\"value\":%d},{\"flag\":\"gyr\",\"value\":%d}]}",
                   led_switch, light_value, distance, accel_mag, gyro_mag);

        // 查找 UART2 设备
        uart2_device = rt_device_find("uart2");
        if (uart2_device == RT_NULL)
        {
            rt_kprintf("Failed to find uart2 device!\n");
            return;
        }

        // 配置串口参数
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = BAUD_RATE_9600;
        config.data_bits = DATA_BITS_8;
        config.stop_bits = STOP_BITS_1;
        config.parity = PARITY_NONE;

        // 设置串口配置
        rt_device_control(uart2_device, RT_DEVICE_CTRL_CONFIG, &config);

        // 打开串口设备
        if (rt_device_open(uart2_device, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
        {
            rt_kprintf("Failed to open uart2 device!\n");
            return;
        }

        // 发送数据
        rt_size_t send_length = rt_device_write(uart2_device, 0, json_data, strlen(json_data));
        rt_kprintf("Sent %d bytes of data: %s\n", send_length, json_data);

        // 关闭串口设备
        rt_device_close(uart2_device);

        // 延时一段时间，防止发送频率过高
        rt_thread_mdelay(5000);
    }
}




int main(void)
{
    /****************** 新增串口4初始化调用开始 ******************/
    // 初始化串口4
     uart4_init();
    /****************** 新增串口4初始化调用结束 ******************/

    rt_thread_t tid;
    rt_hw_sr04_port();
    sr04_read_distance_sample();

    // 创建光强传感器线程
    tid = rt_thread_create("bh1750_thread",
                           bh1750_thread_entry,
                           RT_NULL,
                           2048,  // 增加堆栈大小
                           20,    // 调整优先级
                           10);

    if (tid != RT_NULL)
    {
        // 线程创建成功，启动线程
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("Failed to create bh1750_thread\n");
        return -1;
    }


//       /* 创建超声波测距线程 */
//       rt_thread_t sr04_thread = rt_thread_create("sr04_thread",
//                                                 sr04_read_distance_sample,
//                                                 RT_NULL,
//                                                 1024,
//                                                 RT_THREAD_PRIORITY_MAX / 2,
//                                                 20);
//       if (sr04_thread != RT_NULL)
//       {
//           /* 启动线程 */
//           rt_thread_startup(sr04_thread);
//       }

//语音模块
    rt_thread_t voice_thread = rt_thread_create("voice",
                                                   voice_thread_entry,
                                                   RT_NULL,
                                                   2048, // 线程栈大小
                                                   25,   // 线程优先级
                                                   10);  // 线程入口函数的超时时间
       if (voice_thread != RT_NULL)
       {
           rt_thread_startup(voice_thread);
       }


       // 创建并启动摔倒检测线程
          rt_thread_t fall_detection_thread = rt_thread_create("falldetect",
                                                               fall_detection_thread_entry,
                                                               RT_NULL,
                                                               2048, // 线程栈大小
                                                               23,   // 线程优先级
                                                               10);  // 线程入口函数的超时时间
          if (fall_detection_thread != RT_NULL)
          {
              rt_thread_startup(fall_detection_thread);
          }

        //4G
          rt_thread_t uart_send_thread = rt_thread_create(
              "uart_send",            // 线程名称
              uart_send_thread_entry, // 线程入口函数
              RT_NULL,                // 线程入口参数
              1024,                   // 线程栈大小（单位：字节）
              29,    // 线程优先级
              10                      // 线程的时间片（单位：滴答）
          );
                if (uart_send_thread != RT_NULL)
                {
                    rt_thread_startup(uart_send_thread);
                }
                else
                {
                    rt_kprintf("Failed to create uart_send thread!\n");
                }

                // 主线程无限循环
                while (1)
                {
                    rt_thread_mdelay(1000);
                }





    return 0;
}
