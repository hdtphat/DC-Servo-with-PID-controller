#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

void Clock_init (void);
void GPIO_init (void);
void USART1_init (void);
void USART1_send_string (char *str);
void USART1_IRQHandler (void);
void TIM3_init (void);
void TIM3_IRQHandler (void);
void TIM4_init (void);
void TIM4_IRQHandler (void);
void TIM2_init (void);
void EXTI_init (void);
void EXTI9_5_IRQHandler (void);

static char Rx_Buffer[100], Tx_Buffer[100];
static uint8_t Rx_count = 0, Enable_graphing = 0;
static uint32_t Encoder_CPR = 96;				// Encoder's Pulses Per Revolution (x1 mode)
static uint32_t Vel_count = 0;					// Number of Pulses to calculate rotating speed every sampling
static uint32_t Vel_RPM = 0;						// Real Rotational Speed of motor at the moment (round per minute)
static double Vel_RAD = 0.0;						// Real Angular Velocity of motor at the moment (RAD per second)
static uint32_t Pos_cur_state = 0x00;		// Current state of encoder
static uint32_t Pos_pre_state = 0x00;		// Previous state of encoder
static int64_t Pos_count = 0;						// Number of Pulses to define position of motor


int main (void)
{
	Clock_init();
	GPIO_init();
	USART1_init();
	TIM2_init();
	TIM3_init();
	TIM4_init();
	EXTI_init();
	
	while(1);
}


void EXTI9_5_IRQHandler (void) // PA5 and PA6 external interupt on both rising/falling edge
{
	// PA5 external interupt
	if(EXTI->PR & (1UL<<5)){
		EXTI->PR |= (1UL<<5);
		// Increase Vel_count
		Vel_count++;
		// Store previous state for later comparing
		Pos_pre_state = Pos_cur_state;
		// Get current state of encoder
		Pos_cur_state = ((GPIOA->IDR)&(3UL<<5))>>5;
		// Define position
		switch(Pos_pre_state){
			case 0x00: // Previous state: 0b00
				if(Pos_cur_state == 0x02) Pos_count++; // Current state: 0b10
				else Pos_count--; // Current state: 0b01
				break;
			case 0x01: // Previous state: 0b01
				if(Pos_cur_state == 0x00) Pos_count++; // Current state: 0b00
				else Pos_count--; // Current state: 0b11
				break;
			case 0x02: // Previous state: 0b10
				if(Pos_cur_state == 0x03) Pos_count++; // Current state: 0b11
				else Pos_count--; // Current state: 0b00
				break;
			case 0x03: // Previous state: 0b11
				if(Pos_cur_state == 0x01) Pos_count++; // Current state: 0b01
				else Pos_count--; // Current state: 0b10
				break;
		}
	}
	// PA6 external interupt
	if(EXTI->PR & (1UL<<6)){
		EXTI->PR |= (1UL<<6);
		// Increase Vel_count
		Vel_count++;
		// Store previous state for later comparing
		Pos_pre_state = Pos_cur_state;
		// Get current state of encoder
		Pos_cur_state = ((GPIOA->IDR)&(3UL<<5))>>5;
		// Define position
		switch(Pos_pre_state){
			case 0x00: // Previous state: 0b00
				if(Pos_cur_state == 0x02) Pos_count++; // Current state: 0b10
				else Pos_count--; // Current state: 0b01
				break;
			case 0x01: // Previous state: 0b01
				if(Pos_cur_state == 0x00) Pos_count++; // Current state: 0b00
				else Pos_count--; // Current state: 0b11
				break;
			case 0x02: // Previous state: 0b10
				if(Pos_cur_state == 0x03) Pos_count++; // Current state: 0b11
				else Pos_count--; // Current state: 0b00
				break;
			case 0x03: // Previous state: 0b11
				if(Pos_cur_state == 0x01) Pos_count++; // Current state: 0b01
				else Pos_count--; // Current state: 0b10
				break;
		}
	}
}

void EXTI_init (void)
{
	// Enable clock to AFIO
	RCC->APB2ENR |= (1UL<<0);
	// Configure the EXTI Registers (port: 0-A, 1-B, 2-C, 3-D, 4-E)
	AFIO->EXTICR[0] = 0x0000; // EXTICR1: pin3 <- pin0
	AFIO->EXTICR[1] = 0x0000; // EXTICR1: pin7 <- pin4
	AFIO->EXTICR[2] = 0x0000; // EXTICR1: pin11 <- pin8
	AFIO->EXTICR[3] = 0x0000; // EXTICR1: pin15 <- pin12
	// Disable the EXTI Mask
	EXTI->IMR |= (1UL<<6) | (1UL<<5);
	// Configure the Rising Edge Trigger
	EXTI->RTSR |= (1UL<<6) | (1UL<<5);
	// Configure the Falling Edge Trigger
	EXTI->FTSR |= (1UL<<6) | (1UL<<5);
	// Set interupt priority
	NVIC_SetPriority (EXTI9_5_IRQn, 0);
	// Enable interupt
	NVIC_EnableIRQ (EXTI9_5_IRQn);
}

void TIM2_init (void)	// TIM2_CH1	PWM 1kHz -> Control motor rotational speed
{
	// Enable clock to TIM2
	RCC->APB1ENR |= (1UL<<0);
	// Set TIM2 prescaler
	TIM2->PSC = 18-1;
	// Max counting value
	TIM2->ARR = 1000-1;
	// Reset counter
	TIM2->CNT = 0;	
	// Set CH1 duty cycle
	TIM2->CCR1 = 0; 
	// Set CH1 as PWM mode 1
	TIM2->CCMR1 |= (6U<<4);
	// Enable CH1
	TIM2->CCER |= (1U<<0);
	// Enable counter
	TIM2->CR1 |= (1U<<0);
}

void TIM4_init (void)
{

/*
	Clock before PSC = PCLK1*2 = 18MHz
	Clock after PSC = 18/18 = 1MHz
	Every count takes 1ns
	Since maximum counting value ARR=50000
	So timer interupt every 50ms
*/	

	// Enable clock to TIM4
	RCC->APB1ENR |= (1UL<<2);
	// Set TIM4 prescaler
	TIM4->PSC = 18-1;
	// Max counting value
	TIM4->ARR = 50000;
	// Reset counter
	TIM4->CNT = 0;
	// Enable timer interrupt
	TIM4->DIER |= (1UL<<0);
	// Enable global interupt vector
	NVIC_EnableIRQ(TIM4_IRQn);	
	// enable counter 
	TIM4->CR1 |= (1UL<<0);
}

void TIM4_IRQHandler (void) // TIM4 Interupts every 50ms -> Plot graphs on PC
{
	// Clear interupt flag
	TIM4->SR = 0;
	// Send data to PC every 50ms
	if(Enable_graphing){
		sprintf(Tx_Buffer, "V%d\r \n", Vel_RPM);
		USART1_send_string(Tx_Buffer);
		sprintf(Tx_Buffer, "P%lld\r \n", Pos_count);
		USART1_send_string(Tx_Buffer);
	}
}

void TIM3_init (void)
{

/*
	Clock before PSC = PCLK1*2 = 18MHz
	Clock after PSC = 18/18 = 1MHz
	Every count takes 1ns
	Since maximum counting value ARR=5000
	So timer interupt every 5ms
*/	

	// Enable clock to TIM3
	RCC->APB1ENR |= (1UL<<1);
	// Set TIM3 prescaler
	TIM3->PSC = 18-1;
	// Max counting value
	TIM3->ARR = 5000;
	// Reset counter
	TIM3->CNT = 0;
	// Enable timer interrupt
	TIM3->DIER |= (1UL<<0);
	// Enable global interupt vector
	NVIC_EnableIRQ(TIM3_IRQn);	
	// enable counter 
	TIM3->CR1 |= (1UL<<0);
}

void TIM3_IRQHandler (void)	// TIM3 Interupts every 5ms	-> Calculate velocity and position
{
	uint32_t temp = 0;
	// Clear interupt flag
	TIM3->SR = 0;
	// Caculate Rotating Speed
	temp = Vel_count*(1000/10)*60; // Number of count per minute
	Vel_RPM = temp/(Encoder_CPR*4); // Rotating Speed (RPM)
	Vel_RAD = (double)(Vel_RPM)*(2*PI/60); // Angular Velocity (RAD/s)
	Vel_count = 0; // Refresh counter
}

void USART1_send_string (char *str)
{
	while(*str){
		USART1->DR = *str;  
		while (!(USART1->SR & (1<<6))); 
		str++;
	}
}

void USART1_IRQHandler (void) // Receive a string (end with semi-colon)
{
	// Check for Rx interupt
	if(USART1->SR & (1UL<<5)){
		// Clear Rx interupt flag
		USART1->SR &= ~(1UL<<5);
		// Get a string of data
		if (USART1->DR != ';'){
			Rx_Buffer[Rx_count] = (char)(USART1->DR);
			Rx_count++;
		} else {
			Rx_Buffer[Rx_count] = '\0';
			Rx_count = 0;
			// Estimate Rx string
			switch(Rx_Buffer[0]){
				case 'r':	// Press "Run" button
					Enable_graphing = 1;
					break;
				case 'e':	// Press "Stop" button
					Enable_graphing = 0;
					break;
				case 'f':	// Press "Pause" button
					Enable_graphing = 0;
					break;
				case 'g':	// Press "Resume" button
					Enable_graphing = 1;
					break;
				default:
					break;
			}
		}
	}
}

void USART1_init (void)	// Baudrate 115200bps, 8bit data, 1 stop bit, no parity bit
{

/*
	How to define Baudrate
	
	APB2CLK = 36MHz
	Target_Baudrate = 115200bps
	USART1_prescaler = (APB2CLK * 1000000) / (16 * Target_Baudrate) = 39.0625
	
	Set bit4-15 of USART1_BRR = 39
	Set bit0-3 of USART1_BRR = (0.0625 * 16) = 1
*/	

	// Enable clock to AFIO, USART1
	RCC->APB2ENR |= (1UL<<0) | (1UL<<14);
	// Disable UART1
	USART1->CR1 &= ~(1UL<<13); 
	// Configure data frame
	USART1->CR1 &= ~(1UL<<12);	// 1 Start bit, 8 Data bits, n Stop bit
	USART1->CR1 &= ~(1UL<<10);	// No parity bit
	USART1->CR2 &= ~(3UL<<12);	// 1 stop bit
	// Desired baud rate: 9600
	USART1->BRR = (39UL<<4) | (1UL<<0);
	// Enable Receive/Transmit mode
	USART1->CR1 = (3UL<<2);
	// Enable RX interupt
	USART1->CR1 |= (1UL<<5);
	// Enable global interupt vector
	NVIC_EnableIRQ(USART1_IRQn);
	// Enable UART1
	USART1->CR1 |= (1UL<<13);
}

void GPIO_init (void)
{
	// Enable clock to GPIOA
	RCC->APB2ENR |= (1UL<<2);
	// Reset register
	GPIOA->CRL = 0;
	GPIOA->CRH = 0;
	// Set PA9 as UART_Tx -> Altenate function output 50MHz
	GPIOA->CRH |= (3UL<<4);
	GPIOA->CRH |= (2UL<<6);
	// Set PA10 as UART_Rx -> Input with pull up
	GPIOA->CRH &= ~(3UL<<8);
	GPIOA->CRH |= (2UL<<10);
	GPIOA->ODR |= (1UL<<10);
	// Set PA0 as TIM2_CH1 -> Alternate function output 50MHz
	GPIOA->CRL |= (3UL<<0);
	GPIOA->CRL |= (2UL<<2);
	// Set PA1 as output -> General output push-pull 2MHz
	GPIOA->CRL |= (2UL<<4);
	GPIOA->CRL &= ~(3UL<<6);
	GPIOA->ODR |= (1UL<<1);
	// Set PA2 as output -> General output push-pull 2MHz
	GPIOA->CRL |= (2UL<<8);
	GPIOA->CRL &= ~(3UL<<10);
	GPIOA->ODR &= ~(1UL<<2);
	// Set PA5 external interupt input -> General input
	GPIOA->CRL &= ~(3UL<<20);
	GPIOA->CRL |= (1UL<<22);
	// Set PA6 external interupt input -> General input
	GPIOA->CRL &= ~(3UL<<24);
	GPIOA->CRL |= (1UL<<26);
}

void Clock_init (void)
{

	// APB2 prescaler = 2 -> PCLK2 = 36MHz
	//RCC->CFGR |= (4U<<11);
	// APB1 prescaler = 8 -> PCLK1 = 9MHz
	RCC->CFGR |= (6<<8);
	// Enable clock to AFIO, GPIOA, GPIOB, USART1
	RCC->APB2ENR |= (1UL<<0) | (1UL<<2) | (1UL<<3) | (1UL<<14);
	// Enable clock to TIM3
	RCC->APB1ENR |= (1UL<<1);
}
