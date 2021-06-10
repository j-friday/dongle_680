
#include "gpio.h"
#include "user_config.h"

#define SCLK 0X30
#define SDAT 0X31

#define SCL0_HIGH()		  gpio_set(SCLK,1)
#define SCL0_LOW()		  gpio_set(SCLK,0)
#define SDA0_HIGH()		  gpio_set(SDAT,1)
#define SDA0_LOW()		  gpio_set(SDAT,0)
#define SDA0_SetInput()	  gpio_config(SDAT,INPUT,PULL_HIGH)
#define SDA0_SetOutput()  gpio_config(SDAT,OUTPUT,PULL_HIGH)
#define SDA0_READ()		  gpio_get_input(SDAT)


void delay(uint16_t dly_cnt)
{
    dly_cnt = dly_cnt+5;
	while(dly_cnt--);
	
}
void i2cs_init( )
{
    gpio_config(SCLK,OUTPUT,PULL_HIGH);
    gpio_config(SDAT,OUTPUT,PULL_HIGH);
}

void i2cs_start()
{
	SDA0_HIGH();
	SCL0_HIGH();
	delay(10);
	SDA0_LOW();
	delay(10);
	SCL0_LOW();
}

void i2cs_stop()
{
	SDA0_LOW();
	delay(3);
	SCL0_HIGH();
	SDA0_HIGH();
}

uint8_t i2cs_tx_byte(uint8_t dat)
{
	uint8_t i;
	for(i=0;i<8;i++)
    {
		if(dat&0x80)
        {
			SDA0_HIGH();
		}
        else
        {
			SDA0_LOW();
		}
		delay(3);
		SCL0_HIGH();
		delay(3);
		SCL0_LOW();
		delay(3);
		dat<<=1;
	}
	SDA0_SetInput();
    delay(3);
	SCL0_HIGH();
	delay(3);
	i=SDA0_READ();
	
	SCL0_LOW();
	SDA0_SetOutput();
	return(i==0);
}
uint8_t i2cs_rx_byte(char ack)
{
	uint8_t i;
	uint8_t r=0;
	SDA0_SetInput();
	delay(5);
	for(i=0;i<8;i++)
    {
		SCL0_HIGH();
		delay(5);
		r<<=1;
		if(SDA0_READ())
        {
			r|=1;
		}
		SCL0_LOW();
		delay(5);
	}
	SDA0_SetOutput();
	if(ack)
	    SDA0_LOW();
	else
		SDA0_HIGH();
	SCL0_HIGH();
	delay(5);
	SCL0_LOW();
	SDA0_SetInput();
	return(r);
}

void i2cs_tx_data(uint8_t devAddr7,uint8_t addr,uint8_t*buf,uint8_t size)
{
	int i;
	i2cs_start();
    if(i2cs_tx_byte(devAddr7<<1)==0)
    {
		i2cs_stop();
		return;
	}
	if(i2cs_tx_byte(addr)==0)
	{
		i2cs_stop();
		return;
	}
	for(i=0;i<size;i++)
    {
		if(i2cs_tx_byte(*buf++)==0)
		{
			i2cs_stop();
			return;
		}
	}
	i2cs_stop();
}

void i2cs_rx_data(uint8_t devAddr7,uint8_t addr,uint8_t*buf,uint8_t size)
{
	uint8_t i;
	i2cs_start();
	delay(10);
    if(i2cs_tx_byte(devAddr7<<1)==0)
	{
		i2cs_stop();
		return;
	}
	delay(10);
	if(i2cs_tx_byte(addr)==0)
	{
		i2cs_stop();
		return;
	}
	i2cs_start();
	delay(10);
	if(i2cs_tx_byte((devAddr7<<1)+1)==0)
	{
		i2cs_stop();
		return;
	}
	for(i=0;i<(size-1);i++)
    {
		*buf++=i2cs_rx_byte(1);
	}
	*buf++=i2cs_rx_byte(0);
	SDA0_SetOutput();
	i2cs_stop();
}


