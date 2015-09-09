/****************************************************************************
 *
 *   Copyright (C) 2015 EulerSpace corp. All rights reserved.
 *   Author: Joy.Lu <luyang@eulerspace.com>
 *   		
 *
 ****************************************************************************/

#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "ov7675.h"
 const u8 OV7670_reg[][2]=
{	 
	//AWBC - 自动白平衡控制(Automatic white balance control) 
	{0x43, 0x14},//用户手册里这些寄存器的值都是保留(Reserved),不用设置的呀？
	{0x44, 0xf0},
	{0x45, 0x34},        
	{0x46, 0x58},
	{0x47, 0x28},
	{0x48, 0x3a},

	//AWB Control
	{0x59, 0x88},//用户手册连寄存器都是保留，初始值都没提供
	{0x5a, 0x88},        
	{0x5b, 0x44},
	{0x5c, 0x67},
	{0x5d, 0x49},
	{0x5e, 0x0e},

	//AWB Control
	{0x6c, 0x0a},
	{0x6d, 0x55},       
	{0x6e, 0x11},
	{0x6f, 0x9f},

	//AGC/AEC - Automatic Gain Control自动增益补偿/Automatic exposure Control自动曝光控制
	{0x00, 0x00},
	{0x14, 0x20},
	{0x24, 0x75},
	{0x25, 0x63},
	{0x26, 0xA5},
};
void Delay_1ms( vu32 nCnt_1ms )
{
  u32 nCnt;
  for(; nCnt_1ms != 0; nCnt_1ms--)
    for(nCnt = 56580; nCnt != 0; nCnt--);
}
//ÉèÖÃÍ¼ÏñÊä³ö´°¿Ú
//¶ÔQVGAÉèÖÃ¡£

void OV7670_Window_Set(u16 startx,u16 starty,u16 width,u16 height)
{
	u16 endx=(startx+width*2)%332;
	u16 endy=(starty+height*2);
	u8 x_reg, y_reg;
	u8 state,temp;
	x_reg = ov7675_ReadReg(0x32);
	x_reg &= 0xC0;
	y_reg  = ov7675_ReadReg(0x03);
	y_reg &= 0xF0;
	//设置HREF
	temp = x_reg|((endx&0x7)<<3)|(startx&0x7);
	state = ov7675_WriteReg(0x32, temp );
	temp = (startx&0x7F8)>>3;
	state = ov7675_WriteReg(0x17, temp );
	temp = (endx&0x7F8)>>3;
	state = ov7675_WriteReg(0x18, temp );
	//设置VREF
	temp = y_reg|((endy&0x3)<<2)|(starty&0x3);
	state = ov7675_WriteReg(0x03, temp );
	temp = (starty&0x3FC)>>2;
	state = ov7675_WriteReg(0x19, temp );
	temp = (endy&0x3FC)>>2;
	state = ov7675_WriteReg(0x1A, temp );

}

/**
  * @brief  Configures the ov7620 camera
  */
void ov7675_context_configuration(void)
{
	int i;
	volatile unsigned char read_temp;
	volatile int a;
	if(global_data.param[PARAM_IMAGE_LOW_LIGHT])
	{

	}
	else
	{
	
	}
	read_temp = ov7675_ReadReg(OV7675_CHIP_VERSION_REG);
	if (read_temp == 0x76)
	{
	
		ov7675_WriteReg(0x12, 0x80);  //复位所有寄存器
		Delay_1ms( 100 ); 

		ov7675_WriteReg(0x11, 0x01);  //内部不分频
		ov7675_WriteReg(0x3A, 0x04);  //YUYV VYUY
		ov7675_WriteReg(0x12,0x00);    //yuv enabled qvga
		read_temp=ov7675_ReadReg(0xC2);
		if((read_temp&0x02))
		{
			read_temp=ov7675_ReadReg(0xB8);
			ov7675_WriteReg(0xC2,(read_temp&10000111)|0x10);//DOVDD = 1.8V，则0xB8[6:3] 设为 4'b0010
		}
		else //0xC2[2]=0 默认值
		{
			read_temp=ov7675_ReadReg(0xDA);
			ov7675_WriteReg(0xDA,(read_temp & 11110000)|0x02);//DOVDD = 1.8V，则0xDA[3:0] 设为4'b0010
		}
			
		
		ov7675_WriteReg(0x17, 0x13);//0x13
		ov7675_WriteReg(0x18, 0x01);//0x01
		ov7675_WriteReg(0x32, 0x24);//0xb6
		ov7675_WriteReg(0x19, 0x03);//0x03
		ov7675_WriteReg(0x1a, 0x21);
		ov7675_WriteReg(0x03, 0x05);
		ov7675_WriteReg(0x0c, 0x04);
		
		ov7675_WriteReg(0x3e, 0x1A);
		ov7675_WriteReg(0x70, 0x3a);
		ov7675_WriteReg(0x71, 0x35);
		ov7675_WriteReg(0x72, 0x22);
		ov7675_WriteReg(0x73, 0xf2);
		ov7675_WriteReg(0xa2, 0x02);
		
		ov7675_WriteReg(0x1e, 0x27);
		
		ov7675_WriteReg(0x7a, 0x18);
		ov7675_WriteReg(0x7b, 0x04);
		ov7675_WriteReg(0x7c, 0x09);
		ov7675_WriteReg(0x7d, 0x18);
		ov7675_WriteReg(0x7e, 0x38);
		ov7675_WriteReg(0x7f, 0x47);
		ov7675_WriteReg(0x80, 0x56);
		ov7675_WriteReg(0x81, 0x66);
		ov7675_WriteReg(0x82, 0x74);
		ov7675_WriteReg(0x83, 0x7f);
		ov7675_WriteReg(0x84, 0x89);
		ov7675_WriteReg(0x85, 0x9a);
		ov7675_WriteReg(0x86, 0xA9);
		ov7675_WriteReg(0x87, 0xC4);
		ov7675_WriteReg(0x88, 0xDb);
		ov7675_WriteReg(0x89, 0xEe);
	/*	
		//
		ov7675_WriteReg(0x13, 0xe0);
		ov7675_WriteReg(0x01, 0x50);
		ov7675_WriteReg(0x02, 0x68);
		ov7675_WriteReg(0x00, 0x00);
		ov7675_WriteReg(0x10, 0x00);
		ov7675_WriteReg(0x0d, 0x40);
		ov7675_WriteReg(0x14, 0x18);
		ov7675_WriteReg(0xa5, 0x07);
		ov7675_WriteReg(0xab, 0x08);
		ov7675_WriteReg(0x24, 0x60);
		ov7675_WriteReg(0x25, 0x50);
		ov7675_WriteReg(0x26, 0xe3);
		ov7675_WriteReg(0x9f, 0x78);
		ov7675_WriteReg(0xa0, 0x68);
		///////////////////////////////
	
		//
		ov7675_WriteReg(0xa1, 0x03);
		ov7675_WriteReg(0xa6, 0xd8);
		ov7675_WriteReg(0xa7, 0xd8);
		ov7675_WriteReg(0xa8, 0xf0);
		ov7675_WriteReg(0xa9, 0x90);
		ov7675_WriteReg(0xaa, 0x14);
		ov7675_WriteReg(0x13, 0xe5);
		//
		ov7675_WriteReg(0x0e, 0x61);
		ov7675_WriteReg(0x0f, 0x4b);
		ov7675_WriteReg(0x16, 0x02);
//		ov7675_WriteReg(0x1e, 0x07);
		ov7675_WriteReg(0x21, 0x02);
		ov7675_WriteReg(0x22, 0x91);
		ov7675_WriteReg(0x29, 0x07);
		ov7675_WriteReg(0x33, 0x0b);
		ov7675_WriteReg(0x35, 0x0b);
		ov7675_WriteReg(0x37, 0x1d);
		ov7675_WriteReg(0x38, 0x71);
		ov7675_WriteReg(0x39, 0x2a);
		ov7675_WriteReg(0x3c, 0x78);
		//ov7675_WriteReg(0x4d, 0x40);
		ov7675_WriteReg(0x4e, 0x20);
		ov7675_WriteReg(0x69, 0x00);
		ov7675_WriteReg(0x6b, 0x8a);
		ov7675_WriteReg(0x74, 0x10);
		ov7675_WriteReg(0x8d, 0x4f);
		ov7675_WriteReg(0x8e, 0x00);
		ov7675_WriteReg(0x8f, 0x00);
		ov7675_WriteReg(0x90, 0x00);
		ov7675_WriteReg(0x91, 0x00);
		ov7675_WriteReg(0x92, 0x66);
		ov7675_WriteReg(0x96, 0x00);
		ov7675_WriteReg(0x9a, 0x80);
		ov7675_WriteReg(0xb0, 0x84);
		ov7675_WriteReg(0xb1, 0x0c);
		ov7675_WriteReg(0xb2, 0x0e);
		ov7675_WriteReg(0xb3, 0x82);
		ov7675_WriteReg(0xb8, 0x0a);
		////////////////////////////////////////////////////////////
		//
		ov7675_WriteReg(0x43, 0x14);
		ov7675_WriteReg(0x44, 0xf0);
		ov7675_WriteReg(0x45, 0x41);
		ov7675_WriteReg(0x46, 0x66);
		ov7675_WriteReg(0x47, 0x2a);
		ov7675_WriteReg(0x48, 0x3e);
		ov7675_WriteReg(0x59, 0x8d);
		ov7675_WriteReg(0x5a, 0x8e);
		ov7675_WriteReg(0x5b, 0x53);
		ov7675_WriteReg(0x5c, 0x83);
		ov7675_WriteReg(0x5d, 0x4f);
		ov7675_WriteReg(0x5e, 0x0e);
		ov7675_WriteReg(0x6c, 0x0a);
		ov7675_WriteReg(0x6d, 0x55);
		ov7675_WriteReg(0x6e, 0x11);
		ov7675_WriteReg(0x6f, 0x9e);
		//
		ov7675_WriteReg(0x62, 0x90);
		ov7675_WriteReg(0x63, 0x30);
		ov7675_WriteReg(0x64, 0x11);
		ov7675_WriteReg(0x65, 0x00);
		ov7675_WriteReg(0x66, 0x05);
		ov7675_WriteReg(0x94, 0x11);
		ov7675_WriteReg(0x95, 0x18);
		//
		ov7675_WriteReg(0x6a, 0x40);
		ov7675_WriteReg(0x01, 0x40);
		ov7675_WriteReg(0x02, 0x40);
		ov7675_WriteReg(0x13, 0xe7);
		//
		ov7675_WriteReg(0x4f, 0x80);
		ov7675_WriteReg(0x50, 0x80);
		ov7675_WriteReg(0x51, 0x00);
		ov7675_WriteReg(0x52, 0x22);
		ov7675_WriteReg(0x53, 0x5e);
		ov7675_WriteReg(0x54, 0x80);
		ov7675_WriteReg(0x58, 0x9e);
		//
		ov7675_WriteReg(0x41, 0x08);
		ov7675_WriteReg(0x3f, 0x00);
		ov7675_WriteReg(0x75, 0x03);
		ov7675_WriteReg(0x76, 0xe1);
		ov7675_WriteReg(0x4c, 0x00);
		ov7675_WriteReg(0x77, 0x00);
		ov7675_WriteReg(0x3d, 0xc2);
		ov7675_WriteReg(0x4b, 0x09);
		ov7675_WriteReg(0xc9, 0x60);
		ov7675_WriteReg(0x41, 0x38);
		ov7675_WriteReg(0x56, 0x40);
		//
		ov7675_WriteReg(0x34, 0x11);
		ov7675_WriteReg(0x3b, 0x0a);
		ov7675_WriteReg(0xa4, 0x88);
		ov7675_WriteReg(0x96, 0x00);
		ov7675_WriteReg(0x97, 0x30);
		ov7675_WriteReg(0x98, 0x20);
		ov7675_WriteReg(0x99, 0x30);
		ov7675_WriteReg(0x9a, 0x84);
		ov7675_WriteReg(0x9b, 0x29);
		ov7675_WriteReg(0x9c, 0x03);
		//
		ov7675_WriteReg(0x9d, 0x98);
		ov7675_WriteReg(0x9e, 0x3f);
		ov7675_WriteReg(0x78, 0x04);
		//
		ov7675_WriteReg(0x79, 0x01);
		ov7675_WriteReg(0xc8, 0xf0);
		ov7675_WriteReg(0x79, 0x0f);
		ov7675_WriteReg(0xc8, 0x00);
		ov7675_WriteReg(0x79, 0x10);
		ov7675_WriteReg(0xc8, 0x7e);
		ov7675_WriteReg(0x79, 0x0a);
		ov7675_WriteReg(0xc8, 0x80);
		ov7675_WriteReg(0x79, 0x0b);
		ov7675_WriteReg(0xc8, 0x01);
		ov7675_WriteReg(0x79, 0x0c);
		ov7675_WriteReg(0xc8, 0x0f);
		ov7675_WriteReg(0x79, 0x0d);
		ov7675_WriteReg(0xc8, 0x20);
		ov7675_WriteReg(0x79, 0x09);
		ov7675_WriteReg(0xc8, 0x80);
		ov7675_WriteReg(0x79, 0x02);
		ov7675_WriteReg(0xc8, 0xc0);
		ov7675_WriteReg(0x79, 0x03);
		ov7675_WriteReg(0xc8, 0x40);
		ov7675_WriteReg(0x79, 0x05);
		ov7675_WriteReg(0xc8, 0x30);
		ov7675_WriteReg(0x79, 0x26);
		ov7675_WriteReg(0x2d, 0x00);
		ov7675_WriteReg(0x2e, 0x00);
		//exposure
		ov7675_WriteReg(0x55, 0x30);
		//office
		ov7675_WriteReg(0x13, 0xe5); //AWB off
		ov7675_WriteReg(0x01, 0x84);
		ov7675_WriteReg(0x02, 0x4c);
		ov7675_WriteReg(0x6a, 0x40);
		ov7675_WriteReg(0x3b, 0x0a);
		ov7675_WriteReg(0x2d, 0x00);
		ov7675_WriteReg(0x2e, 0x00);
		//Contrast -2
		ov7675_WriteReg(0x56, 0x30);
		//OV7670_Window_Set(156,14,160,120);	//ÉèÖÃ´°¿Ú	
*/
		 
	}

}

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: ov7675 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t ov7675_WriteReg(uint8_t Addr, uint8_t Data)
{
  uint32_t timeout = TIMEOUT_MAX;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7675_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send Data */
  I2C_SendData(I2C2, Data);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send I2C2 STOP Condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* If operation is OK, return 0 */
  return 0;
}


/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: mt9v034 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t ov7675_ReadReg(uint8_t Addr)
{
  uint32_t timeout = TIMEOUT_MAX;
  uint8_t Data = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7675_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) 
	{return 0xFF;}
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (uint8_t)(Addr));

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Clear AF flag if arised */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Send DCMI selcted device slave Address for write */
  I2C_Send7bitAddress(I2C2, ov7675_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */ 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  /* return the read data */
  return Data;
}
