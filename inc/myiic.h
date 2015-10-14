#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f4xx.h"
 	
   	   		   
//////IO方向设置
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
//IO操作函数	 
#define IIC_SCL_HIGH    GPIO_SetBits(GPIOC,GPIO_Pin_15) //SCL
#define IIC_SCL_LOW     GPIO_ResetBits(GPIOC,GPIO_Pin_15) //SCL
#define IIC_SDA_HIGH    GPIO_SetBits(GPIOC,GPIO_Pin_14) //SDA
#define IIC_SDA_LOW     GPIO_ResetBits(GPIOC,GPIO_Pin_14) //SDA

#define READ_SDA   GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)  //输入SDA 


void delay_us(int us);
void delay_ms(int ms);
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















