/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "utils.h"
#include "usart.h"
#include "settings.h"
#include "sonar.h"
#include "sonar_mode_filter.h"

#define SONAR_SCALE	1000.0f
#define SONAR_MIN	0.02f		/** 0.12m sonar minimum distance */
#define SONAR_MAX	4.5f		/** 3.50m sonar maximum distance */

extern int atoi (__const char *__nptr);
extern uint32_t get_boot_time_us(void);

static char data_buffer[5]; // array for collecting decoded data

static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;
static volatile int data_counter = 0;
static volatile int data_valid = 0;
static volatile int new_value = 0;

static volatile uint32_t sonar_measure_time_interrupt = 0;
static volatile uint32_t sonar_measure_time = 0;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float sonar_raw = 0.0f;  // m

float sonar_mode = 0.0f;
float sonar_valid = false;				/**< the mode of all sonar measurements */

#define SONAR_TIMEOUT 60000			// 0.06 second, ~10 meter
#define SOUND_SPEED 3.4f			// sound speed in mm/10us
#define SONAR_MIN_SR04 20			// min valid distance in milli-meter
#define SONAR_MAX_SR04 5000			// max valid distance in milli-meter

int sr04_initialized = 0;
static int64_t trigger_time = 0;
static int rising_time = -1;

// private sonar variables
bool timeout = false;
static int private_sonar_initialized = 0;
int last_pulse_time = -1;
int first_pulse_time = -1;
int pulse_counter = 0;
float distance = NAN;

#define DEADBAND 0
#define PULSE_TIMEOUT 50
#define MIN_PULSE_COUNT 8

inline void delay_private()
{
	for(volatile int i=0; i<172; i++)
		;
}

void reset_timer()
{
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	timeout = false;
	pulse_counter = 0;
	last_pulse_time = -1;
	first_pulse_time = -1;
	TIM_Cmd(TIM5, ENABLE);
	distance = NAN;
}

/**
  * @brief  Triggers the sonar to measure the next value
  *
  * see datasheet for more info
  */
void sonar_trigger(){
	GPIO_SetBits(GPIOE, GPIO_Pin_8);
	
	if (sr04_initialized)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		// delay 10~20us
		volatile int time = TIM6->CNT;
		while(time == TIM6->CNT);
		time = TIM6->CNT;
		while(time == TIM6->CNT);
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		
		trigger_time = get_boot_time_us();
	}
	
	if (private_sonar_initialized)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);	// pull down comparator modifier
		reset_timer();
		
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		delay_private();
		
		GPIO_SetBits(GPIOA, GPIO_Pin_0);	// release comparator modifier
		
	}
}

/**
  * @brief  Sonar interrupt handler
  */
void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		uint8_t data = (USART_ReceiveData(UART4));

		if (data == 'R')
		{
			/* this is the first char (start of transmission) */
			data_counter = 0;
			data_valid = 1;

			/* set sonar pin 4 to low -> we want triggered mode */
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);
		}
		else if (0x30 <= data && data <= 0x39)
		{
			if (data_valid)
			{
				data_buffer[data_counter] = data;
				data_counter++;
			}
		}
		else if (data == 0x0D)
		{
			if (data_valid && data_counter == 4)
			{
				data_buffer[4] = 0;
				int temp = atoi(data_buffer);

				/* use real-world maximum ranges to cut off pure noise */
				if ((temp > SONAR_MIN*SONAR_SCALE) && (temp < SONAR_MAX*SONAR_SCALE))
				{
					/* it is in normal sensor range, take it */
					last_measure_time = measure_time;
					measure_time = get_boot_time_us();
					sonar_measure_time_interrupt = measure_time;
					dt = ((float)(measure_time - last_measure_time)) / 1000000.0f;

					valid_data = temp;
					sonar_mode = insert_sonar_value_and_get_mode_value(valid_data / SONAR_SCALE);
					new_value = 1;
					sonar_valid = true;
					
				} else {
					sonar_valid = false;
				}
			}

			data_valid = 0;
		}
		else
		{
			data_valid = 0;
		}
	}
}

/**
  * @brief  Basic Kalman filter
  */
void sonar_filter()
{
	/* no data for long time */
	if (dt > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt * v_pred;
	v_pred = v_post;

	float x_new = sonar_mode;
	sonar_raw = x_new;
	x_post = x_pred + global_data.param[PARAM_SONAR_KALMAN_L1] * (x_new - x_pred);
	v_post = v_pred + global_data.param[PARAM_SONAR_KALMAN_L2] * (x_new - x_pred);

}


/**
  * @brief  Read out newest sonar data
  *
  * @param  sonar_value_filtered Filtered return value
  * @param  sonar_value_raw Raw return value
  */
bool sonar_read(float* sonar_value_filtered, float* sonar_value_raw)
{
	// check SR04 timeout
	if (sr04_initialized && trigger_time > 0 && get_boot_time_us() - trigger_time > SONAR_TIMEOUT)
	{
		printf("TIMEOUT\n");
		sonar_valid = false;
		trigger_time = 0;
	}
	
	if (private_sonar_initialized && timeout && isnan(distance))
	{
		sonar_valid = false;
	}
	
	/* getting new data with only around 10Hz */
	if (new_value) {
		sonar_filter();
		new_value = 0;
		sonar_measure_time = get_boot_time_us();
	}

	/* catch post-filter out of band values */
	if (x_post < SONAR_MIN || x_post > SONAR_MAX) {
		sonar_valid = false;
	}

	*sonar_value_filtered = x_post;
	*sonar_value_raw = sonar_raw;

	return sonar_valid;
}

void take_mesaure(float measure)
{
	last_measure_time = measure_time;
	measure_time = get_boot_time_us();
	sonar_measure_time_interrupt = measure_time;
	dt = ((float)(measure_time - last_measure_time)) / 1000000.0f;
	
	//printf("R%f\n", measure);

	valid_data = measure*1000;
	sonar_mode = insert_sonar_value_and_get_mode_value(measure);
	new_value = 1;
	sonar_valid = true;
	trigger_time = 0;
}

void EXTI1_IRQHandler(void)
{
	if (private_sonar_initialized)
	{
		volatile int t = TIM5->CNT;
		
		if (t > DEADBAND && !timeout)
		{
			//
			if (t-last_pulse_time > PULSE_TIMEOUT)
			{
				pulse_counter = 0;
				first_pulse_time = t;
			}
			else
			{
				pulse_counter ++;
			}
			
			if (pulse_counter >= MIN_PULSE_COUNT)
			{
				distance = first_pulse_time*0.000001f * 340/2;
				printf("%f\n", distance);
				take_mesaure(distance);
				timeout = true;
			}
				
			last_pulse_time = t;
		}
	}
	
	EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI9_5_IRQHandler(void)
{
	// ignore any pulses if not triggered by caller
	if (sr04_initialized && trigger_time > 0)
	{
		volatile int time = TIM6->CNT;
		if(GPIOA->IDR & GPIO_Pin_8)
			rising_time = time;
		else if (rising_time > 0)
		{
			volatile int delta_time = time - rising_time;
			if (delta_time < 0)
				delta_time += 60000;
			int temp = delta_time * SOUND_SPEED/2;
			if (temp <= SONAR_MAX_SR04 && temp >= SONAR_MIN_SR04)
			{
				/* it is in normal sensor range, take it */
				last_measure_time = measure_time;
				measure_time = get_boot_time_us();
				sonar_measure_time_interrupt = measure_time;
				dt = ((float)(measure_time - last_measure_time)) / 1000000.0f;
				
				printf("R%d\n", temp);

				valid_data = temp;
				sonar_mode = insert_sonar_value_and_get_mode_value(valid_data / SONAR_SCALE);
				new_value = 1;
				sonar_valid = true;
				trigger_time = 0;
			}
			else
			{
				printf("R-1\n");
				sonar_valid = false;
			}
			rising_time = -1;
		}
		else
		{
			sonar_valid = false;
			rising_time = -1;	// no corresponding rising edge.
			trigger_time = -1;
		}
	}
	


	EXTI_ClearITPendingBit(EXTI_Line8);
}

void TIM5_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);
	TIM_Cmd(TIM5, DISABLE);
	timeout = true;
}


/**
 * @brief  Configures the SR-04 sonar sensor Peripheral.
 */
void sonar_config_hc_sr04(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	// TIM6 as timer, 10us resolution, 60000 overflow ~= 600ms
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	TIM_DeInit(TIM6);
	TIM_InternalClockConfig(TIM6);
	TIM_TimeBaseStructure.TIM_Prescaler=839;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=60000-1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM6,DISABLE);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,ENABLE);
	
	// A8 as echo
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// D14 as trigger
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);

	// EXTI
	EXTI_ClearITPendingBit(EXTI_Line8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_Init(&EXTI_InitStructure);

	// priority : lowest
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	sr04_initialized = 1;
}

/**
 * @brief  Configures the private sonar sensor Peripheral.
 */
void sonar_config_private()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// TIM5 as timer and timeout controller
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_Period = 60000-1;		// ~60ms overflow, 1us per tick
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM5, ENABLE);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);

	// A0 as comparator modifier
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	
	// C4 as sender
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	
	// A1 as echo
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// EXTI
	EXTI_ClearITPendingBit(EXTI_Line1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_Init(&EXTI_InitStructure);

	// priority : lowest
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	private_sonar_initialized = true;
}

void sonar_config(void)
{
	sonar_config_private();
	valid_data = 0;

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure l3gd20 CS pin in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configures the nested vectored interrupt controller. */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Connect UART pins to AF7 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	GPIO_InitTypeDef GPIO_InitStructure_Serial2;
	GPIO_InitStructure_Serial2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure_Serial2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_Serial2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_Serial2.GPIO_PuPd = GPIO_PuPd_UP;

	/* USART RX pin configuration */
	GPIO_InitStructure_Serial2.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure_Serial2);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	/* Configure the UART4 */
	USART_Init(UART4, &USART_InitStructure);

	/* Enable UART4 interrupt */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	USART_Cmd(UART4, ENABLE);

}

uint32_t get_sonar_measure_time()
{
    return sonar_measure_time;
}

uint32_t get_sonar_measure_time_interrupt()
{
    return sonar_measure_time_interrupt;
}

