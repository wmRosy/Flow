#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f4xx.h"
 	
   	   		   
//////IO��������
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL_HIGH    GPIO_SetBits(GPIOC,GPIO_Pin_15) //SCL
#define IIC_SCL_LOW     GPIO_ResetBits(GPIOC,GPIO_Pin_15) //SCL
#define IIC_SDA_HIGH    GPIO_SetBits(GPIOC,GPIO_Pin_14) //SDA
#define IIC_SDA_LOW     GPIO_ResetBits(GPIOC,GPIO_Pin_14) //SDA

#define READ_SDA   GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)  //����SDA 


void delay_us(int us);
void delay_ms(int ms);
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















