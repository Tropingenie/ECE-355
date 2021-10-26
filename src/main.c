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
void myTIM2_Init(void);
void myEXTI_Init(void);


// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.
volatile char first_edge = TRUE;

int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */

	while (1)
	{
		// Nothing is going on here...
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


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI2_3_IRQHandler()
{
	// Declare/initialize your local variables here...

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR2) != 0)
	{
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
			trace_printf("%d Hz\n", (uint32_t)(freq < 0 ? (freq - 0.5) : (freq + 0.5)));
			trace_printf("%d microseconds\n", (uint32_t)(period < 0 ? (period - 0.5) : (period + 0.5)));
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
		}
		// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		EXTI->PR |= EXTI_PR_PR1;
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------