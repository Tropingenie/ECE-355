//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// Goal: Measure the frequency and period of an input square wave from a
// function generator. Max frequency of this implementation: 480MHz (note:
// actual values will be significantly lower due to the time other operations
// take). Min frequency of this implementation: 1Hz.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

#define TRUE 1 == 1
#define FALSE 1 == 0

#define CLOCKSPEED 48000000

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myLCD_Init(void);
void ADC1_Init(void);
void DAC1_Init(void);
void myLCD_Print(uint32_t frequency);


// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.
volatile char first_edge = TRUE;
volatile char measure_pa1 = FALSE;
volatile uint32_t resistance;

int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myLCD_Init();		/* Initialize LCD */
	ADC1_Init();			/* Initialize ADC */
	DAC1_Init();			/* Initialize DAC */


	while (1)
	{
//		myLCD_Print(1234, 5678, 3);
		// Smoke pot
		if((ADC1->ISR & ADC_ISR_EOC) != 0){

			resistance = (resistance + 1)%1234;

		}
	}

	return 0;

}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA2 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_0);

	/* Ensure no pull-up/pull-down for PA2 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_MODER_MODER2_0);
}

void myGPIOB_Init()
{
	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB7 as input */
	// Relevant register: GPIOB->MODER
	GPIOB->MODER &= ~(GPIO_MODER_MODER7_0);

	/* Ensure no pull-up/pull-down for PB7 */
	// Relevant register: GPIOB->PUPDR
	GPIOB->PUPDR &= ~(GPIO_MODER_MODER7_0);

	/* Configure PB4-6,8-15 as output */
	// Relevant register: GPIOB->MODER
	GPIOB->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);

	/* Ensure no pull-up/pull-down for PB4-6,8-15  */
	// Relevant register: GPIOB->PUPDR
	GPIOB->PUPDR &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void ADC1_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	ADC1->SMPR = 0x7;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL5;
	ADC1->CFGR1 &= ~(ADC_CFGR1_RES|ADC_CFGR1_ALIGN);
	ADC1->CFGR1 |= (ADC_CFGR1_OVRMOD|ADC_CFGR1_CONT);

	ADC1->CR |= ADC_CR_ADEN;
	ADC1->CR &= ~ADC_CR_ADDIS;
	ADC1->CR |= ADC_CR_ADSTART;
}

void DAC1_Init(void){
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
}

void handshake(){
	// Set PB4 to 1 (assert "Enable")
	GPIOB->ODR |= GPIO_ODR_4;

	//Wait for PB7 to become 1 ("Done" to be asserted)
	while ((GPIOB->IDR & GPIO_IDR_7) == 0);

	//Set PB4 to 0 (deassert "Enable")
	GPIOB->ODR &= ~(GPIO_ODR_4);

	//Wait for PB7 to become 0 ("Done" to be deasserted)
	while ((GPIOB->IDR & GPIO_IDR_7) != 0);
}

void myLCD_Init(){
	//Function set
	// DB5 needs to be set to 1 (PB13=DB5=1)
	// DL=1 - DDRAM access is performed using 8-bit interface (PB12=DB4=1)
	// N=1 & F=0 - Two lines of eight characters are displayed (PB11=DB3=1 & PB10=DB2=0)
	GPIOB->ODR = ((GPIO_ODR_13 |  GPIO_ODR_12 |  GPIO_ODR_11) & ~(GPIO_ODR_10));
	handshake();

	//Display on/off control
	// DB3 needs to be set to 1 (PB11=DB3=1)
	// D=1 - Display is on (PB10=DB2=1)
	// C=0 & B=0 - Cursor is not displayed, and it is not blinking (PB9=DB1=0 & PB8=DB0=0)
	GPIOB->ODR = ((GPIO_ODR_11 |  GPIO_ODR_10) & ~(GPIO_ODR_9 | GPIO_ODR_8));
	handshake();

	//Entry mode set
	// DB2 needs to be set to 1 (PB10=DB2=1)
	// I/D=1 - DDRAM address is auto-incremented after each access (PB9=DB1=1)
	// S=0 - Display is not shifted (PB8=DB0=0)
	GPIOB->ODR = ((GPIO_ODR_10 |  GPIO_ODR_9) & ~(GPIO_ODR_8));
	handshake();

	//Clear display
	// DB0 needs to be set to 1 (PB8=DB0=1)
	GPIOB->ODR = (GPIO_ODR_8);
	handshake();
}

void determine_digits(uint32_t num, char* digits){

	// Determine the number of each position
	digits[0] = digits[1] = digits[2] = digits[3] = 0;
	while(num >= 1000){
		num -= 1000;
		digits[0]++;
	}
	while(num >= 100){
		num -= 100;
		digits[1]++;
	}
	while(num >= 10){
		num -= 10;
		digits[2]++;
	}
	while(num >= 1){
		num--;
		digits[3]++;
	}
}

void myLCD_Print(uint32_t frequency){

	char digits[4];

		//Write first line

		//LCD instructions - initialize
		// RS=0 R/W=0 DB7=1 (PB5=0 PB6=0 PB15=DB7=1)
		GPIOB->ODR = (GPIO_ODR_15 & ~(GPIO_ODR_5 | GPIO_ODR_6));
		handshake();

		//Print "F"
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x46<<8);
		handshake();
		//Print ":"
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x3A<<8);
		handshake();

		//Print digits
		determine_digits(frequency, digits);
		//Print the digits
		for(int i = 0; i < 4; i++){
			//LCD instructions - write ASCII codes to display
			// RS=1 R/W=0 DB7-0=ASCII code (PB5=1 PB6=0)
			digits[i] += 48; //Convert to ASCII code
			GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(digits[i]<<8);
			handshake();
		}

		//Print "Hz"
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x48<<8);
		handshake();
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x7A<<8);
		handshake();
		// Deliberate no break at end of case (for debugging)

//	//LCD instructions - write ASCII codes to display
//	// RS=1 R/W=0 DB7-0=ASCII code (PB5=1 PB6=0)
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x30<<8);
//	handshake();
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x31<<8);
//	handshake();
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x32<<8);
//	handshake();
//
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x33<<8);
//	handshake();
//
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x34<<8);
//	handshake();

		//Write second line

		//LCD instructions - initialize
		// RS=0 R/W=0 DB7=1 DB6=1 (PB5=0 PB6=0 PB15=DB7=1 PB14=DB6=1)
		GPIOB->ODR = ((GPIO_ODR_15 | GPIO_ODR_14) & ~(GPIO_ODR_5 | GPIO_ODR_6));
		handshake();

		//Print "R"
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x52<<8);
		handshake();
		//Print ":"
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x3A<<8);
		handshake();

		//Print digits
		determine_digits(resistance, digits);
		//Print the digits
		for(int i = 0; i < 4; i++){
			//LCD instructions - write ASCII codes to display
			// RS=1 R/W=0 DB7-0=ASCII code (PB5=1 PB6=0)
			digits[i] += 48; //Convert to ASCII code
			GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(digits[i]<<8);
			handshake();
		}

		//Print "Oh"
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x4F<<8);
		handshake();
		GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x68<<8);
		handshake();


//	//LCD instructions - write ASCII codes to display
//	// RS=1 R/W=0 DB7-0=ASCII code (PB5=1 PB6=0)
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x39<<8);
//	handshake();
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x38<<8);
//	handshake();
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x37<<8);
//	handshake();
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x36<<8);
//	handshake();
//
//	GPIOB->ODR = ((GPIO_ODR_5) & ~(GPIO_ODR_6))|(0x35<<8);
//	handshake();



}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}


void myEXTI_Init()
{
	/// EXTI0 Init

	/* Map EXTI0 line to PA0 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR0;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR0;

	// NVIC set priority is same as EXTI1 so it's set there (below)

	/// EXTI1 Init

	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);


	/// EXTI2 Init

	/* Map EXTI2 line to PA2 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;

	/* EXTI2 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR2;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR2;

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI2_3_IRQn, 0);


// Enable all interrups

	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI2_3_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
	}
}


void freq_calc(void){
	//
			if(first_edge == TRUE){
			// 1. If this is the first edge:
				first_edge = FALSE;
			//	- Clear count register (TIM2->CNT).
				TIM2->CNT = 0;
			//	- Start timer (TIM2->CR1).
				TIM2->CR1 |= TIM_CR1_CEN;
			}else{
			//    Else (this is the second edge):
				first_edge = TRUE;
			//	- Stop timer (TIM2->CR1).
				TIM2->CR1 &= ~TIM_CR1_CEN;
			//	- Read out count register (TIM2->CNT).
				uint32_t count = TIM2->CNT;
			//	- Calculate signal period and frequency.
				double freq = (double)CLOCKSPEED/count; // Hz
				double period = 1000000/freq; // microseconds
			//	- Print calculated values to the console.
				myLCD_Print((uint32_t)(freq < 0 ? (freq - 0.5) : (freq + 0.5)));
//				myLCD_Print(0, (uint32_t)(period < 0 ? (period - 0.5) : (period + 0.5)), 2);// For debugging
			// TODO: Write to display instead of trace_printf
//				trace_printf("%d Hz\n", (uint32_t)(freq < 0 ? (freq - 0.5) : (freq + 0.5)));
//				trace_printf("%d microseconds\n", (uint32_t)(period < 0 ? (period - 0.5) : (period + 0.5)));
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.
			}
}


void EXTI0_1_IRQHandler()
{
	/* Check if EXTI0 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR0) != 0){
		trace_printf("Button press detected\n");
		// Logical NOT the measure pa1 variable
		(measure_pa1 == TRUE) ? (measure_pa1 = FALSE) : (measure_pa1 = TRUE);

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		EXTI->PR |= EXTI_PR_PR0;
	}

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0){
		// Run the frequency calculator function here
		if (measure_pa1 == TRUE){
			freq_calc();
		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		EXTI->PR |= EXTI_PR_PR1;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI2_3_IRQHandler()
{
	// Declare/initialize your local variables here...

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR2) != 0)
	{
		if (measure_pa1 == FALSE){
			freq_calc();
		}
		// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		EXTI->PR |= EXTI_PR_PR2;
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
