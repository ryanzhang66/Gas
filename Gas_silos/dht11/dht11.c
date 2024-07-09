#include "dht11.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "iot_adc.h"
#include <stdio.h>
#include "cmsis_os2.h"
#include <ohos_init.h>
#include <unistd.h>
#include <time.h>
/*DHT11复位和检测响应函数，返回值：1-检测到响应信号；0-未检测到响应信号*/
IotGpioValue inputdata = {0};
IotGpioValue outputdata = {0};
int DHT11RstAndCheck(void)
{
    int timer = 0;
    IoTGpioInit(DHT11_DATA_IO);
    IoTGpioSetFunc(DHT11_DATA_IO, IOT_GPIO_FUNC_GPIO_12_GPIO);
    IoTGpioSetPull(DHT11_DATA_IO, IOT_GPIO_PULL_UP);
    IoTGpioSetDir(DHT11_DATA_IO, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(DHT11_DATA_IO, 0);
    osDelay(2U);
    IoTGpioSetOutputVal(DHT11_DATA_IO, 1);
    dht11_delay_us(30); // 拉高20~40us
    IoTGpioSetDir(DHT11_DATA_IO, IOT_GPIO_DIR_IN);
    IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
    int num = 0;
    while (inputdata && num <= 100)
    {
        num++;
        IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
    }
    if (num > 100)
        return 0;
    while (!inputdata && timer <= 100)
    {
        timer++;
        dht11_delay_us(1);
        IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
    }

    if (timer > 100 || timer < 20) // 判断响应时间
    {
        return 0;
    }
    timer = 0;
    while (inputdata && timer <= 100)
    {
        timer++;
        IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
        dht11_delay_us(1);
    }
    if (timer > 100 || timer < 20) // 检测响应信号之后的高电平
    {
        return 0;
    }
    return 1;
}

// /*读取一字节数据，返回值-读到的数据*/
int DHT11ReadByte(void)
{
    uint8_t i;
    int byt = 0;
    for (i = 0; i < 8; i++)
    {
        IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
        int num = 0;
        while (inputdata && num <= 100) // 等待低电平，数据位前都有50us低电平时隙
        {
            IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
            num++;
        }
        num = 0;
        while (!inputdata && num <= 100) // 等待高电平，开始传输数据位
        {
            IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
            num++;
        }
        num = 0;
        while (inputdata && num <= 80)
        {
            IoTGpioGetInputVal(DHT11_DATA_IO, &inputdata);
            num++;
        }
        byt <<= 1;    // 因高位在前，所以左移byt，最低位补0
        if (num > 30) // 将总线电平值读取到byt最低位中
        {
            byt |= 0x01;
        }
    }

    return byt;
}

// /*读取一次数据，返回值：Humi-湿度整数部分数据,Temp-温度整数部分数据；返回值: -1-失败，1-成功*/
int DHT11ReadData(int *Humi, int *Temp)
{
    // printf("access\n");
    int sta = 0;
    uint8_t i;
    int buf[5];

    if (DHT11RstAndCheck()) // 检测响应信号
    {
        for (i = 0; i < 5; i++) // 读取40位数据
        {
            buf[i] = DHT11ReadByte(); // 读取1字节数据
        }
        if (buf[0] + buf[1] + buf[2] + buf[3] == buf[4]) // 校验成功
        {
            *Humi = buf[0]; // 保存湿度整数部分数据
            *Temp = buf[2]; // 保存温度整数部分数据
        }
        sta = 1;
    }
    else // 响应失败返回-1
    {
        *Humi = 0xFF; // 响应失败返回255
        *Temp = 0xFF; // 响应失败返回255
        sta = -1;
    }
    return sta;
}

// 应该延时了1us吧
void dht11_delay_us(int num)
{
    while (num--)
    {
        for (int i = 0; i < 25; i++)
        {
        }
    }
}