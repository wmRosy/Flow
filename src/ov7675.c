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

		//ov7675_WriteReg(0x11, 0x01);  //内部不分频
		//ov7675_WriteReg(0x3A, 0x04);  //YUYV VYUY
		//ov7675_WriteReg(0x12,0x00);    //yuv enabled qvga
		
			
		
	//		ov7675_WriteReg(0x12 , 0x80);//系统复位
		  ov7675_WriteReg(0x09 , 0x10);//软待机模式
		  ov7675_WriteReg(0xc1 , 0x7f);//某种测试模式
		  ov7675_WriteReg(0x11 , 0x81);//[7]位：测试  [6]位：直接用外部时钟（不允许预分频）[5：0]位：内部时钟分频系数 F(内部时钟)＝F(外部时钟)/（位[5：0]＋1）范围：[0 0000]-[1 1111]
		  ov7675_WriteReg(0x3a , 0x0c);// TSLB[3] 1 U Y V Y  V Y U Y [2:1]测试模式
		  ov7675_WriteReg(0x3d , 0xc0);//伽马允许 UV饱和度自动调整  U Y V Y
		//  ov7675_WriteReg(0x12 , 0x00);//系统保持
		  ov7675_WriteReg(0x15 , 0x60);//测试模式
		  ov7675_WriteReg(0x17 , 0x13);//水平起始点
		  ov7675_WriteReg(0x18 , 0x01);//水平结束点
		  ov7675_WriteReg(0x32 , 0xbf);//水平起始点
		  ov7675_WriteReg(0x19 , 0x02);//垂直起始点
		  ov7675_WriteReg(0x1a , 0x7a);//垂直结束点
		  ov7675_WriteReg(0x03 , 0x0a);//垂直起始点
		  ov7675_WriteReg(0x0c , 0x00);//待机时输出三态
		  ov7675_WriteReg(0x3e , 0x00);//正常PCLK 不分频
		  ov7675_WriteReg(0x70 , 0x3a);//没啥卵用 可以输出彩条
		  ov7675_WriteReg(0x71 , 0x35);//没啥卵用 可以输出彩条
		  ov7675_WriteReg(0x72 , 0x11);//测试模式
		  ov7675_WriteReg(0x73 , 0xf0);//测试模式
		  ov7675_WriteReg(0xa2 , 0x02);//测试模式
		  ov7675_WriteReg(0x7a , 0x20);//伽马曲线最高部分斜率
		  ov7675_WriteReg(0x7b , 0x03);//伽马曲线第1段
		  ov7675_WriteReg(0x7c , 0x0a);//伽马曲线第2段
		  ov7675_WriteReg(0x7d , 0x1a);//伽马曲线第3段
		  ov7675_WriteReg(0x7e , 0x3f);//伽马曲线第4段
		  ov7675_WriteReg(0x7f , 0x4e);//伽马曲线第5段
		  ov7675_WriteReg(0x80 , 0x5b);//伽马曲线第6段
		  ov7675_WriteReg(0x81 , 0x68);//伽马曲线第7段
		  ov7675_WriteReg(0x82 , 0x75);//伽马曲线第8段
		  ov7675_WriteReg(0x83 , 0x7f);//伽马曲线第9段
		  ov7675_WriteReg(0x84 , 0x89);//伽马曲线第10段
		  ov7675_WriteReg(0x85 , 0x9a);//伽马曲线第11段
		  ov7675_WriteReg(0x86 , 0xa6);//伽马曲线第12段
		  ov7675_WriteReg(0x87 , 0xbd);//伽马曲线第13段
		  ov7675_WriteReg(0x88 , 0xd3);//伽马曲线第14段
		  ov7675_WriteReg(0x89 , 0xe8);//伽马曲线第15段
		  ov7675_WriteReg(0x13 , 0xe0);//快速自动曝光/自动增益 自动曝光步长无限制 Banding开 测试模式 最小曝光为1行 自动增益手动 白平衡手动 曝光手动
		  ov7675_WriteReg(0x00 , 0x00);//GAIN 寄存器 10位自动增益控制AGC[7:0] (VREF[7:6] (0x03)是高2位)增益＝(0x03[7]位+1）×(0x03[6]位＋1） ×(0x00[7]位＋1）×(0x00[6]位＋1）×([0x005]位＋1）×([0x004]位＋1）×(0x00[3:0]位/16＋1）
		  ov7675_WriteReg(0x10 , 0x00);//自动曝光值 AEC[9:2] (AEC[15:10]和AEC[1:0]参见寄存器 AECHH[5:0](0x07)和COM1[1:0](0x04))
		  ov7675_WriteReg(0x0d , 0x40);//测试模式
		  ov7675_WriteReg(0x14 , 0x28);//自动增益最大值:64倍
		  ov7675_WriteReg(0xa5 , 0x02);//50HZ曝光步数限制 0x02
		  ov7675_WriteReg(0xab , 0x02);//60HZ曝光步数限制 0x02
		  ov7675_WriteReg(0x24 , 0x68);//自动曝光/自动增益稳定范围（上限）
		  ov7675_WriteReg(0x25 , 0x58);//自动曝光/自动增益稳定范围（下限）
		  ov7675_WriteReg(0x26 , 0xc2);//自动曝光/自动增益快速调节范围 上限12 下限2
		  ov7675_WriteReg(0x9f , 0x78);//测试模式
		  ov7675_WriteReg(0xa0 , 0x68);//测试模式
		  ov7675_WriteReg(0xa1 , 0x03);//测试模式
		  ov7675_WriteReg(0xa6 , 0xd8);//测试模式
		  ov7675_WriteReg(0xa7 , 0xd8);//测试模式
		  ov7675_WriteReg(0xa8 , 0xf0);//测试模式
		  ov7675_WriteReg(0xa9 , 0x90);//测试模式
		  ov7675_WriteReg(0xaa , 0x14);//测试模式
		  ov7675_WriteReg(0x13 , 0xe5);//快速自动曝光/自动增益 关闭 步长无限制 banding开 测试模式 最小曝光1行 自动增益自动 白平衡手动 曝光自动
		  ov7675_WriteReg(0x0e , 0x61);//测试模式
		  ov7675_WriteReg(0x0f , 0x4b);//测试模式 格式改变时复位全部时序
		  ov7675_WriteReg(0x16 , 0x02);//未使用
		  ov7675_WriteReg(0x1e , 0x37);//镜像图像 垂直翻转 测试模式
		  ov7675_WriteReg(0x21 , 0x02);//未使用
		  ov7675_WriteReg(0x22 , 0x91);//未使用
		  ov7675_WriteReg(0x29 , 0x07);//未使用
		  ov7675_WriteReg(0x33 , 0x0b);//未使用
		  ov7675_WriteReg(0x35 , 0x0b);//未使用
		  ov7675_WriteReg(0x37 , 0x1d);//未使用
		  ov7675_WriteReg(0x38 , 0x71);//未使用
		  ov7675_WriteReg(0x39 , 0x2a);//未使用
		  ov7675_WriteReg(0x3c , 0x78);//当VSYNC为低时无HREF 测试模式
		  ov7675_WriteReg(0x4d , 0x40);//测试模式
		  ov7675_WriteReg(0x4e , 0x20);//测试模式
		  ov7675_WriteReg(0x69 , 0x00);//Gr 通道固定增益1x Gb:1x R:1x B:1x
		  ov7675_WriteReg(0x6b , 0x0a);//测试模式
		  ov7675_WriteReg(0x74 , 0x10);//数字增益由REG74[1:0]控制
		  ov7675_WriteReg(0x8d , 0x4f);//测试模式
		  ov7675_WriteReg(0x8e , 0x00);//测试模式
		  ov7675_WriteReg(0x8f , 0x00);//测试模式
		  ov7675_WriteReg(0x90 , 0x00);//测试模式
		  ov7675_WriteReg(0x91 , 0x00);//测试模式
		  ov7675_WriteReg(0x96 , 0x00);//测试模式
		  ov7675_WriteReg(0x9a , 0x80);//测试模式
		  ov7675_WriteReg(0xb0 , 0x84);
		  ov7675_WriteReg(0xb1 , 0x0c);
		  ov7675_WriteReg(0xb2 , 0x0e);
		  ov7675_WriteReg(0xb3 , 0x82);
		  ov7675_WriteReg(0xb8 , 0x0a);
		  ov7675_WriteReg(0x43 , 0x0a);//测试模式 格式改变时不复位全部时序
		  ov7675_WriteReg(0x44 , 0xf2);//测试模式
		  ov7675_WriteReg(0x45 , 0x39);//测试模式
		  ov7675_WriteReg(0x46 , 0x62);//测试模式
		  ov7675_WriteReg(0x47 , 0x3d);//测试模式
		  ov7675_WriteReg(0x48 , 0x55);//测试模式
		  ov7675_WriteReg(0x59 , 0x83);//测试模式
		  ov7675_WriteReg(0x5a , 0x0d);//测试模式
		  ov7675_WriteReg(0x5b , 0xcd);//测试模式
		  ov7675_WriteReg(0x5c , 0x8c);//测试模式
		  ov7675_WriteReg(0x5d , 0x77);//测试模式
		  ov7675_WriteReg(0x5e , 0x16);//测试模式
		  ov7675_WriteReg(0x6c , 0x0a);//AWB控制寄存器3
		  ov7675_WriteReg(0x6d , 0x65);//AWB控制寄存器2
		  ov7675_WriteReg(0x6e , 0x11);//AWB控制寄存器1
		  ov7675_WriteReg(0x6a , 0x40);//AWB控制寄存器0
		  ov7675_WriteReg(0x01 , 0x56);//AWB蓝通道增益
		  ov7675_WriteReg(0x02 , 0x44);//AWB红通道增益
		  ov7675_WriteReg(0x13 , 0xe7);//快速自动曝光/自动增益 关闭 步长无限制 banding开  最小曝光1行 自动增益自动 白平衡自动 曝光自动
		  ov7675_WriteReg(0x4f , 0x88);//色彩矩阵系数1
		  ov7675_WriteReg(0x50 , 0x8b);//色彩矩阵系数2
		  ov7675_WriteReg(0x51 , 0x04);//色彩矩阵系数3
		  ov7675_WriteReg(0x52 , 0x11);//色彩矩阵系数4
		  ov7675_WriteReg(0x53 , 0x8c);//色彩矩阵系数5
		  ov7675_WriteReg(0x54 , 0x9d);//色彩矩阵系数6
		  ov7675_WriteReg(0x55 , 0x00);//亮度控制
		  ov7675_WriteReg(0x56 , 0x40);//对比度控制
		  ov7675_WriteReg(0x57 , 0x80);//对比度中心
		  ov7675_WriteReg(0x58 , 0x9a);//对比度自动中心允许 几个符号位
		  ov7675_WriteReg(0x41 , 0x08);//禁止YUV输出边缘增强自动调整结果在0x3f里范围在0x75 降噪自动调整允许 结果保存在0x4C 范围0x77
		  ov7675_WriteReg(0x3f , 0x00);//边缘增强强度
		  ov7675_WriteReg(0x75 , 0x04);//边缘增强下限
		  ov7675_WriteReg(0x76 , 0xC0);//黑像素矫正关 白像素矫正开 边缘增强上限0
		  ov7675_WriteReg(0x4c , 0x00);//降噪强度
		  ov7675_WriteReg(0x77 , 0x01);//降噪偏移量
		  ov7675_WriteReg(0x3d , 0xc2);//伽马允许 UV饱和度自动调整 输出顺序变YUYV YVYU
		  ov7675_WriteReg(0x4b , 0x09);
		  ov7675_WriteReg(0xc9 , 0x30);//UV 饱和度控制最小值
		  ov7675_WriteReg(0x41 , 0x38);//允许YUV边缘增强自动调整 降噪阈值自动调整  AWB自动增益允许
		  ov7675_WriteReg(0x56 , 0x40);//对比度控制
		  ov7675_WriteReg(0x34 , 0x11);//测试模式 光线太强，曝光值可以小于条纹滤波器的值
		  ov7675_WriteReg(0x3b , 0x12);
		  ov7675_WriteReg(0xa4 , 0x88);//自动帧率控制调整 -> 帧率减半
		  ov7675_WriteReg(0x96 , 0x00);//测试模式
		  ov7675_WriteReg(0x97 , 0x30);//测试模式
		  ov7675_WriteReg(0x98 , 0x20);//测试模式
		  ov7675_WriteReg(0x99 , 0x30);//测试模式
		  ov7675_WriteReg(0x9a , 0x84);//测试模式
		  ov7675_WriteReg(0x9b , 0x29);//测试模式
		  ov7675_WriteReg(0x9c , 0x03);//测试模式
		  ov7675_WriteReg(0x9d , 0x99);//测试模式
		  ov7675_WriteReg(0x9e , 0x7f);//测试模式
		  ov7675_WriteReg(0x78 , 0x04);//测试模式
		  ov7675_WriteReg(0x79 , 0x01);//测试模式
		  ov7675_WriteReg(0xc8 , 0xf0);//测试模式
		  ov7675_WriteReg(0x79 , 0x0f);//测试模式
		  ov7675_WriteReg(0xc8 , 0x00);//测试模式
		  ov7675_WriteReg(0x79 , 0x10);//测试模式
		  ov7675_WriteReg(0xc8 , 0x7e);//测试模式
		  ov7675_WriteReg(0x79 , 0x0a);//测试模式
		  ov7675_WriteReg(0xc8 , 0x80);//测试模式
		  ov7675_WriteReg(0x79 , 0x0b);
		  ov7675_WriteReg(0xc8 , 0x01);
		  ov7675_WriteReg(0x79 , 0x0c);
		  ov7675_WriteReg(0xc8 , 0x0f);
		  ov7675_WriteReg(0x79 , 0x0d);
		  ov7675_WriteReg(0xc8 , 0x20);
		  ov7675_WriteReg(0x79 , 0x09);
		  ov7675_WriteReg(0xc8 , 0x80);
		  ov7675_WriteReg(0x79 , 0x02);
		  ov7675_WriteReg(0xc8 , 0xc0);
		  ov7675_WriteReg(0x79 , 0x03);
		  ov7675_WriteReg(0xc8 , 0x40);
		  ov7675_WriteReg(0x79 , 0x05);
		  ov7675_WriteReg(0xc8 , 0x30);
		  ov7675_WriteReg(0x79 , 0x26);//测试模式
		  ov7675_WriteReg(0x62 , 0x00);//镜头补偿选项符号正  镜头补偿中心相对传感器水平坐标
		  ov7675_WriteReg(0x63 , 0x00);//镜头补偿选项符号正  镜头补偿中心相对传感器竖直坐标
		  ov7675_WriteReg(0x64 , 0x06);//镜头补偿
		  ov7675_WriteReg(0x65 , 0x00);
		  ov7675_WriteReg(0x66 , 0x05);//镜头补偿使能
		  ov7675_WriteReg(0x94 , 0x05);//镜头补偿
		  ov7675_WriteReg(0x95 , 0x09);//镜头补偿
		  ov7675_WriteReg(0x2a , 0x10);//插入无效像素
		  ov7675_WriteReg(0x2b , 0xc2);//插入无效像素
		  ov7675_WriteReg(0x15 , 0x00);    
		  ov7675_WriteReg(0x3a , 0x04);//YUYV
		  ov7675_WriteReg(0x3d , 0xc3);//伽马允许 UV饱和度自动调整  YUVV
		  ov7675_WriteReg(0x19 , 0x03);//垂直窗大小高8位
		  ov7675_WriteReg(0x1a , 0x7b);//垂直窗大小高8位
		  ov7675_WriteReg(0x2a , 0x00);//水平方向插入无效像素
		  ov7675_WriteReg(0x2b , 0x00);//水平方向插入无效像素
		  ov7675_WriteReg(0x18 , 0x01);//水平窗口大小高8位
		  ov7675_WriteReg(0x19 , 0x03);//垂直窗口起始点高8位
		  ov7675_WriteReg(0x1a , 0x21);//垂直窗大小高8位
		  ov7675_WriteReg(0x03 , 0x05);//VREF结束低2位 高两位
		  ov7675_WriteReg(0x17 , 0x13);//水平窗口起始点高8位
		  ov7675_WriteReg(0x18 , 0x27);//水平窗口大小高8位
		  ov7675_WriteReg(0x32 , 0x24);//HREF结束低3位 高三位
		  ov7675_WriteReg(0xe6 , 0x80);//测试模式
		  ov7675_WriteReg(0xe1 , 0x40);//测试模式 60hz条纹滤波器
		  ov7675_WriteReg(0xe4 , 0xb8);//测试模式
		  ov7675_WriteReg(0xe5 , 0x33);//测试模式
		  ov7675_WriteReg(0xbf , 0xf0);
		  ov7675_WriteReg(0x66 , 0x05);//镜头补偿选项5
		  ov7675_WriteReg(0x62 , 0x10);//镜头补偿选项1
		  ov7675_WriteReg(0x63 , 0x0b);//镜头补偿选项2
		  ov7675_WriteReg(0x65 , 0x07);//镜头补偿选项4
		  ov7675_WriteReg(0x64 , 0x0f);//镜头补偿选项3
		  ov7675_WriteReg(0x94 , 0x0e);//镜头补偿选项6(仅当LCC5[2](0x66)=1时有效)
		  ov7675_WriteReg(0x95 , 0x12);//镜头补偿选项7(仅当LCC5[2](0x66)=1时有效)
		  ov7675_WriteReg(0x4f , 0x74);//色彩矩阵系数1
		  ov7675_WriteReg(0x50 , 0x59);
		  ov7675_WriteReg(0x51 , 0x1a);
		  ov7675_WriteReg(0x52 , 0x12);
		  ov7675_WriteReg(0x53 , 0x6a);
		  ov7675_WriteReg(0x54 , 0x7c);
		  ov7675_WriteReg(0x58 , 0x1e);
		  ov7675_WriteReg(0x41 , 0x38);//允许YUV输出边缘增强自动调整  降噪阈值自动调整允许 AWB增益允许 颜色矩阵系数不加倍
		  ov7675_WriteReg(0x75 , 0x05);//自动降噪下限
		  ov7675_WriteReg(0x76 , 0xe0);//白像素矫正 降噪偏移量上限
		  ov7675_WriteReg(0x77 , 0x07);//降噪偏移量
		  ov7675_WriteReg(0x24 , 0x38);//自动曝光/自动增益稳定范围（上限）
		  ov7675_WriteReg(0x25 , 0x28);//自动曝光/增益下限
		  ov7675_WriteReg(0x26 , 0x80);//自动曝光/增益快速调节范围
		  ov7675_WriteReg(0x7a , 0x10);//gamma
		  ov7675_WriteReg(0x7b , 0x0c);//gama 1
		  ov7675_WriteReg(0x7c , 0x17);
		  ov7675_WriteReg(0x7d , 0x2c);
		  ov7675_WriteReg(0x7e , 0x50);
		  ov7675_WriteReg(0x7f , 0x60);
		  ov7675_WriteReg(0x80 , 0x6e);
		  ov7675_WriteReg(0x81 , 0x7b);
		  ov7675_WriteReg(0x82 , 0x87);
		  ov7675_WriteReg(0x83 , 0x92);
		  ov7675_WriteReg(0x84 , 0x9c);
		  ov7675_WriteReg(0x85 , 0xaf);
		  ov7675_WriteReg(0x86 , 0xbf);
		  ov7675_WriteReg(0x87 , 0xd7);
		  ov7675_WriteReg(0x88 , 0xe8);
		  ov7675_WriteReg(0x89 , 0xf4);
		  ov7675_WriteReg(0x43 , 0x0a);//测试模式
		  ov7675_WriteReg(0x44 , 0xf2);//测试模式
		  ov7675_WriteReg(0x45 , 0x46);//测试模式
		  ov7675_WriteReg(0x46 , 0x5f);//测试模式
		  ov7675_WriteReg(0x47 , 0x2e);//测试模式
		  ov7675_WriteReg(0x48 , 0x42);//测试模式
		  ov7675_WriteReg(0x59 , 0xb1);//测试模式
		  ov7675_WriteReg(0x5a , 0xb5);//测试模式
		  ov7675_WriteReg(0x5b , 0xdd);//测试模式
		  ov7675_WriteReg(0x5c , 0x7b);//测试模式
		  ov7675_WriteReg(0x5d , 0x57);//测试模式
		  ov7675_WriteReg(0x5e , 0x14);//测试模式
		  ov7675_WriteReg(0x6c , 0x0e);//AWB控制寄存器3
		  ov7675_WriteReg(0x6d , 0x65);//AWB控制寄存器2
		  ov7675_WriteReg(0x6e , 0x11); //AWB控制寄存器1
		  ov7675_WriteReg(0x6f , 0x9e); //AWB控制寄存器0
		  ov7675_WriteReg(0x09 , 0x00); //输出驱动电流1x
		  
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
		
		  //B&W
	//	ov7675_WriteReg(0x3a, 0x14);
		ov7675_WriteReg(0x67, 0x80);
		ov7675_WriteReg(0x68, 0x80);
		//contrast +2
		ov7675_WriteReg(0x56, 0x60);
		//Brightness +1
		ov7675_WriteReg(0x55, 0x18);
		 
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
