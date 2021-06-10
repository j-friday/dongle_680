#include <stdint.h>        // standard integer definition
#include <string.h>        // string manipulation
#include <stddef.h>     // standard definition
#include "user_config.h"
#include "uart.h"
#include "uart2.h"
#include "gpio.h"
#include "adc.h"
#include "icu.h"
#include "BK3633_RegList.h"



uint16_t g_adc_value;
volatile uint8_t adc_flag;
uint16_t referance_voltage;
extern volatile uint32_t XVR_ANALOG_REG_BAK[16];
/************************************************************************
//ADC�ο���ѹĬ��Ϊ1.05V
//ȷ��ADC�ȶ�������ADC����Ҫ�Ӹ�10nf���صĵ���
*************************************************************************/
void adc_init(uint8_t channel,uint8_t mode)
{
	uint32_t cfg;

	//enable adc clk
	SET_SADC_POWER_UP;
	//set special as peripheral func,set float is  peripheral function,the 2th function id clockout
	gpio_config(0x30 + channel,FLOAT,PULL_NONE); 

	//set adc mode/channel/wait clk
	cfg  = ( (mode << POS_SADC_REG0X0_CFG0_MODE ) 
	       | (channel << POS_SADC_REG0X0_CFG0_CHNL) 
	       | (0x01 << POS_SADC_REG0X0_CFG0_SETING));
	
	cfg |= ((18 << POS_SADC_REG0X0_CFG0_SAMP_RATE) 
	      | (5 << POS_SADC_REG0X0_CFG0_PRE_DIV)
          | (0x0 << POS_SADC_REG0X0_CFG0_ADC_FILTER)
          | (0x01 << POS_SADC_REG0X0_CFG0_INT_CLEAR));
    
    SADC_REG0X0_CFG0=cfg;
	//REG_APB7_ADC_CFG |= (0x01 << BIT_ADC_EN);//������ʹ��ADC����ȻADC FIFO��ʱû�ж����ٴ�����ADC�Ͳ������ж�

    SADC_REG0X2_CFG1 = ((1<<POS_SADC_REG0X2_CHANN_EXPAND)|(1<<POS_SADC_REG0X2_STEADY_CTRL));
    SADC_REG0X3_CFG2 = (3<<POS_SADC_REG0X3_STA_CTRL);
    
	SYS_REG0X10_INT_EN |= (0x01 << POS_SYS_REG0X10_INT_EN_ADC);

    if(mode==3)
        SADC_REG0X0_CFG0 |= SET_ADC_EN;
	
}

void adc_deinit(uint8_t channel)
{
    gpio_config(0x30 + channel,INPUT,PULL_HIGH);

    SADC_REG0X0_CFG0 &= ~(SET_ADC_EN+(0x03 << POS_SADC_REG0X0_CFG0_MODE ));
    SYS_REG0X10_INT_EN &= ~(0x01 << POS_SYS_REG0X10_INT_EN_ADC);
    SET_SADC_POWER_DOWN;
}

void adc_isr(void)
{
	SADC_REG0X0_CFG0 |= (0x01 << POS_SADC_REG0X0_CFG0_INT_CLEAR);
	//bk_printf("adc int\r\n");
    adc_flag=1;	
    if((SADC_REG0X0_CFG0&0x03)==0x03)
        bk_printf("adc_value=%x\r\n",SADC_REG0X4_DAT);
}

extern void Delay_us(int num);
uint16_t adc_get_value(uint8_t channel,uint8_t mode)
{   
    volatile uint16_t adc_cnt;
    adc_cnt=0;
    adc_flag =0;

    if((SADC_REG0X0_CFG0&0x03)==0x03)
        return 0;

    SADC_REG0X0_CFG0 |= SET_ADC_EN+(mode << POS_SADC_REG0X0_CFG0_MODE )+(channel << POS_SADC_REG0X0_CFG0_CHNL);
    
    while (!adc_flag)  
    {
        adc_cnt++;       
        if(adc_cnt>300)
        {
            //bk_printf("g_adc_value_timeout\r\n");
            break;			
        }
        Delay_us(10);
    } 
    
    if(adc_flag == 1)
    {
        g_adc_value=SADC_REG0X4_DAT>>2;

        //bk_printf("g_adc_value=%x,channel=%x\r\n",g_adc_value,channel);        
    }
    
    SADC_REG0X0_CFG0 &= ~(SET_ADC_EN+(0x03 << POS_SADC_REG0X0_CFG0_MODE )+(0x0f << POS_SADC_REG0X0_CFG0_CHNL)); //ADCֵ��ȡ��ɺ�����ʹ��λ���       
    return g_adc_value;     
}

/**************************************************************************
//ע��ȷ��ת��ֵ���ȶ��ԣ�ADC����Ҫ�Ӹ�10nf���صĵ���
//ADCУ׼
//У׼ADC��Ҫ��оƬһ���ȶ��Ĺ���ѹ��Ȼ����ADC�ο���ѹ
//�������У׼Ĭ��ʹ��3V��Դ����,�ڲ���ѹ��Ϊ0.75V
*************************************************************************/
#define CALIB_COUNT 6
#define STABL_VALT 75///��ѹ���0.75V�ȶ���ѹ

void calib_adc(void)
{
    #if(ADC_DRIVER) 
    uint8_t i;
    static uint16_t calib_temp=0;
    
    XVR_ANALOG_REG_BAK[7] |= (1<<19);
    addXVR_Reg0x7 = XVR_ANALOG_REG_BAK[7];
	
    adc_init(7,1);

    for(i=0;i<CALIB_COUNT;i++)
    {
        calib_temp += adc_get_value(7,1);
        Delay_us(1000);
    }
    
    referance_voltage=(0xff*STABL_VALT)/(calib_temp/CALIB_COUNT);///�����ֵΪ103����ô�ο���ѹ��Ϊ1.03V
    bk_printf("referance_voltage=%d\r\n",referance_voltage);

    XVR_ANALOG_REG_BAK[7] &= ~(1<<19);
    addXVR_Reg0x7 = XVR_ANALOG_REG_BAK[7];
    
    #endif
}
