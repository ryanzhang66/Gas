#ifndef _DHT11_H
#define _DHT11_H

#define DHT11_OUT PC_OUT(12)
#define DHT11_IN PC_IN(12)

#define DHT11_DATA_IO 12
int DHT11Init(void);
int DHT11ReadData(int *Humi, int *Temp);
int DHT11RstAndCheck(void);
void delay_us(int num);
#endif