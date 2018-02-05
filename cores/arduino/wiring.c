/*
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "wiring_private.h"

// the prescaler is set so that timerb3 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
uint16_t microseconds_per_timerb3_overflow;

uint32_t F_CPU_CORRECTED = F_CPU;

#define PWM_TIMER_PERIOD	0xFF	/* For frequency */
#define PWM_TIMER_COMPARE	0x80	/* For duty cycle */

#define TIME_TRACKING_TIMER_PERIOD		0xFF
#define TIME_TRACKING_TICKS_PER_OVF		(TIME_TRACKING_TIMER_PERIOD + 1)	/* Timer ticks per overflow of TCB3 */
#define TIME_TRACKING_TIMER_DIVIDER		64		/* Clock divider for TCB3 */
#define TIME_TRACKING_CYCLES_PER_OVF	(TIME_TRACKING_TICKS_PER_OVF * TIME_TRACKING_TIMER_DIVIDER)

// the whole number of milliseconds per timerb3 overflow
uint16_t millis_inc;

// the fractional number of milliseconds per timerb3 overflow
uint16_t fract_inc;
#define FRACT_MAX (1000)

// whole number of microseconds per timerb3 tick
uint16_t microseconds_per_timerb3_tick;

volatile uint32_t timerb3_overflow_count = 0;
volatile uint32_t timerb3_millis = 0;
static uint16_t timerb3_fract = 0;

inline uint16_t clockCyclesPerMicrosecondComp(uint32_t clk){
	return ( (clk) / 1000000L );
}

inline uint16_t clockCyclesPerMicrosecond(){
	return clockCyclesPerMicrosecondComp(F_CPU_CORRECTED);
}

inline uint16_t clockCyclesToMicroseconds(uint16_t cycles){
	return ( cycles / clockCyclesPerMicrosecond() );
}

inline uint32_t microsecondsToClockCycles(uint16_t microseconds){
	return ( microseconds * clockCyclesPerMicrosecond() );
}

ISR(TCB3_INT_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	uint32_t m = timerb3_millis;
	uint16_t f = timerb3_fract;

	m += millis_inc;
	f += fract_inc;
	if (f >= FRACT_MAX) {

		f -= FRACT_MAX;
		m += 1;
	}

	timerb3_fract = f;
	timerb3_millis = m;
	timerb3_overflow_count++;

	/* Clear flag */
	TCB3.INTFLAGS = TCB_CAPT_bm;
}

unsigned long millis()
{
	unsigned long m;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	uint8_t status = SREG;
	cli();
	m = timerb3_millis;

	SREG = status;

	return m;
}

unsigned long micros() {
	unsigned long overflows, microseconds;
	uint8_t ticks;

	/* Save current state and disable interrupts */
	uint8_t status = SREG;
	cli();

	/* Get current number of overflows and timer count */
	overflows = timerb3_overflow_count;
	ticks = TCB3.CNTL;

	/* If the timer overflow flag is raised, we just missed it,
	increment to account for it, & read new ticks */
	if(TCB3.INTFLAGS & TCB_CAPT_bm){
		overflows++;
		ticks = TCB3.CNTL;
	}

	/* Restore state */
	SREG = status;

	/* Return microseconds of up time  (resets every ~70mins) */
	microseconds = ((overflows * microseconds_per_timerb3_overflow)
				+ (ticks * microseconds_per_timerb3_tick));
	return microseconds;
}

void delay(unsigned long ms)
{
	uint32_t start_time = micros(), delay_time = 1000*ms;

	/* Calculate future time to return */
	uint32_t return_time = start_time + delay_time;

	/* If return time overflows */
	if(return_time < delay_time){
		/* Wait until micros overflows */
		while(micros() > return_time);
	}

	/* Wait until return time */
	while(micros() < return_time);
}

/* Delay for the given number of microseconds.  Assumes a 1, 8, 12, 16, 20 or 24 MHz clock. */
void delayMicroseconds(unsigned int us)
{
	// call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 24000000L
	// for the 24 MHz clock for the aventurous ones, trying to overclock

	// zero delay fix
	if (!us) return; //  = 3 cycles, (4 when true)

	// the following loop takes a 1/6 of a microsecond (4 cycles)
	// per iteration, so execute it six times for each microsecond of
	// delay requested.
	us *= 6; // x6 us, = 7 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 22 (24) cycles above, remove 5, (5*4=20)
	// us is at least 6 so we can substract 5
	us -= 5; //=2 cycles

#elif F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call takes 18 (20) cycles, which is 1us
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop" "\n\t"
		"nop" "\n\t"
		"nop"); //just waiting 4 cycles
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	us = (us << 2) + us; // x5 us, = 7 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 26 (28) cycles above, remove 7, (7*4=28)
	// us is at least 10 so we can substract 7
	us -= 7; // 2 cycles

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/4 of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2; // x4 us, = 4 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 19 (21) cycles above, remove 5, (5*4=20)
	// us is at least 8 so we can substract 5
	us -= 5; // = 2 cycles,

#elif F_CPU >= 12000000L
	// for the 12 MHz clock if somebody is working with USB

	// for a 1 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1.5us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/3 of a microsecond (4 cycles)
	// per iteration, so execute it three times for each microsecond of
	// delay requested.
	us = (us << 1) + us; // x3 us, = 5 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 20 (22) cycles above, remove 5, (5*4=20)
	// us is at least 6 so we can substract 5
	us -= 5; //2 cycles

#elif F_CPU >= 8000000L
	// for the 8 MHz internal clock

	// for a 1 and 2 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 2us
	if (us <= 2) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/2 of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1; //x2 us, = 2 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 17 (19) cycles above, remove 4, (4*4=16)
	// us is at least 6 so we can substract 4
	us -= 4; // = 2 cycles

#else
	// for the 1 MHz internal clock (default settings for common Atmega microcontrollers)

	// the overhead of the function calls is 14 (16) cycles
	if (us <= 16) return; //= 3 cycles, (4 when true)
	if (us <= 25) return; //= 3 cycles, (4 when true), (must be at least 25 if we want to substract 22)

	// compensate for the time taken by the preceeding and next commands (about 22 cycles)
	us -= 22; // = 2 cycles
	// the following loop takes 4 microseconds (4 cycles)
	// per iteration, so execute it us/4 times
	// us is at least 4, divided by 4 gives us 1 (no zero delay bug)
	us >>= 2; // us div 4, = 4 cycles


#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
	// return = 4 cycles
}

void init()
{
	// this needs to be called before setup() or some functions won't
	// work there
	
/*************************** GET VCC & FUSE SETTING ***************************/


	/* Measure VDD using ADC */
	uint8_t supply_voltage;
	
	/* Initialize AC reference (what we are measuring) - 1.5V known */
	VREF.CTRLA |= VREF_AC0REFSEL_1V5_gc;	
	
	/* Enable AC reference */
	VREF.CTRLB |= VREF_AC0REFEN_bm;

	/* DAC to max -- output reference voltage */
	AC0.DACREF = 0xFF;
	
	/* Enable DAC REF by selecting it as input and enabling AC */
	AC0.MUXCTRLA |= AC_MUXNEG_DACREF_gc;
	AC0.CTRLA |= ADC_ENABLE_bm;

	/* Initialize ADC reference (VDD) */
	ADC0.CTRLC = ADC_REFSEL_VDDREF_gc;
	
	/* Initialize MUX (DAC/AC reference from VREF) */
	ADC0.MUXPOS = ADC_MUXPOS_DACREF_gc;
	
	/* Enable ADC */
	ADC0.CTRLA |= ADC_ENABLE_bm;
	
	/* Start a conversion */
	ADC0.COMMAND |= ADC_STCONV_bm;
	
	/* Wait until result is ready */
	while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));
	
	/* Result ready */
	/* supply_voltage = (VIN * 1024)/result where VIN = 1.5V from VREF */
	uint16_t adc_result = ADC0.RES;
	
	uint16_t voltage = (15 * 1024) / adc_result; /* using 1.5 << 1 to avoid using float */
	
	/* Only for the purposes of staying within safe operating range -- approximate */
	if(voltage >= 48){ /* 4.8V+ -> 5V */
		supply_voltage = VCC_5V0;
	} else if (voltage >= 30){ /* 3V-4V7 -> 3V3 */
		supply_voltage = VCC_3V3;
	} else { /* < 3V -> 1V8 */
		supply_voltage = VCC_1V8;
	}
	
	/* Fuse setting for 16/20MHz oscillator */
	uint8_t fuse_setting = FUSE.OSCCFG & FUSE_FREQSEL_gm;
	
	/* Deinitialize ADC, AC & VREF */
	ADC0.CTRLA = 0x00;
	ADC0.MUXPOS = 0x00;	
	ADC0.CTRLC = 0x00;		
	
	AC0.CTRLA = 0x00;	
	AC0.MUXCTRLA = 0x00;	
	AC0.DACREF = 0xFF;	
	
	VREF.CTRLB = 0x00;	
	VREF.CTRLA = 0x00;

/******************************** CLOCK STUFF *********************************/

 	int64_t cpu_freq;
 	
 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
 	int8_t sigrow_val = 0;
 	#endif

 	/* Initialize clock divider to stay within safe operating area */

 	if(supply_voltage >= VCC_5V0){
	 	
	 	/* Disable system clock prescaler - F_CPU should now be ~16/20MHz */
	 	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0x00);
	 	
	 	/* Assign cpu_freq value and sigrow_val depending on fuse setting */
	 	if(fuse_setting == FREQSEL_20MHZ_gc){
		 	cpu_freq = 20000000;
		 	
		 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
		 	sigrow_val = SIGROW.OSC20ERR5V;
		 	#endif

		 } else { /* fuse_setting == FREQSEL_16MHZ_gc */
		 	cpu_freq = 16000000;

		 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
		 	sigrow_val = SIGROW.OSC16ERR5V;
		 	#endif

	 	}
	 	
	 } else if (supply_voltage == VCC_3V3) {
	 	
	 	/* Enable system clock prescaler to DIV2 - F_CPU should now be ~8/10MHz */
	 	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, (CLKCTRL_PEN_bm | CLKCTRL_PDIV_2X_gc));
	 	
	 	/* Assign cpu_freq value and sigrow_val depending on fuse setting */
	 	if(fuse_setting == FREQSEL_20MHZ_gc){
		 	cpu_freq = 10000000;
		 	
		 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
		 	sigrow_val = SIGROW.OSC20ERR3V;
		 	#endif

		 } else { /* fuse_setting == FREQSEL_16MHZ_gc */
		 	cpu_freq = 8000000;
		 	
		 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
		 	sigrow_val = SIGROW.OSC16ERR3V;
		 	#endif
	 	}
	 	
	 } else {
	 	/* Shouldn't get here but just in case... */
	 	
	 	/* Enable system clock prescaler to DIV4 - F_CPU should now be ~4/5MHz */
	 	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, (CLKCTRL_PEN_bm | CLKCTRL_PDIV_4X_gc));
	 	
	 	
	 	if(fuse_setting == FREQSEL_20MHZ_gc){
		 	cpu_freq = 5000000;
		 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
		 	sigrow_val = SIGROW.OSC20ERR3V;
		 	#endif
		 	
		 } else { /* fuse_setting == FREQSEL_16MHZ_gc */
		 	cpu_freq = 4000000;
		 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
		 	sigrow_val = SIGROW.OSC16ERR3V;
		 	#endif
	 	}
 	}

 	#if (PERFORM_SIGROW_CORRECTION_F_CPU == 1)
 	/* Calculate actual F_CPU with error values from signature row */
 	cpu_freq *= (1024 + sigrow_val);
 	cpu_freq /= 1024; 	
 	#endif /* (CORRECT_F_CPU == 1) */

	/* Apply calculated value to F_CPU_CORRECTED */
	F_CPU_CORRECTED = (uint32_t)cpu_freq;

/***************************** TIMERS FOR PWM *********************************/



							/*  TYPE A TIMER   */

	/* PORTMUX setting for TCA -> all outputs [0:2] point to PORTB pins [0:2] */
	PORTMUX.TCAROUTEA	= PORTMUX_TCA0_PORTB_gc;

	/* Setup timers for single slope PWM, but do not enable, will do in analogWrite() */
	TCA0.SINGLE.CTRLB |= (TCA_SINGLE_WGMODE_SINGLESLOPE_gc);

	/* Period setting, 16 bit register but val resolution is 8 bit */
	TCA0.SINGLE.PER	= PWM_TIMER_PERIOD;

	/* Default duty 50%, will re-assign in analogWrite() */
	TCA0.SINGLE.CMP0BUF = PWM_TIMER_COMPARE;
	TCA0.SINGLE.CMP1BUF = PWM_TIMER_COMPARE;
	TCA0.SINGLE.CMP2BUF = PWM_TIMER_COMPARE;

	/* Use DIV64 prescaler (giving 250kHz clock), enable TCA timer */
	TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV64_gc) | (TCA_SINGLE_ENABLE_bm);


						    /*	TYPE B TIMERS  */

	/* PORTMUX alternate location needed for TCB0 & 1, TCB2 is default location */
	PORTMUX.TCBROUTEA	|= (PORTMUX_TCB0_bm | PORTMUX_TCB1_bm);

	/* Start with TCB0 */
	TCB_t *timer_B = (TCB_t *)&TCB0;

	/* Timer B Setup loop for TCB[0:2] */
	do{
		/* 8 bit PWM mode, but do not enable output yet, will do in analogWrite() */
		timer_B->CTRLB = (TCB_CNTMODE_PWM8_gc);

		/* Assign 8-bit period */
		timer_B->CCMPL = PWM_TIMER_PERIOD;

		/* default duty 50%, set when output enabled */
		timer_B->CCMPH = PWM_TIMER_COMPARE;

		/* Use TCA clock (250kHz) and enable */
		/* (sync update commented out, might try to synchronize later */
		timer_B->CTRLA = (TCB_CLKSEL_CLKTCA_gc)
						//|(TCB_SYNCUPD_bm)
						|(TCB_ENABLE_bm);

		/* Increment pointer to next TCB instance */
		timer_B++;

	/* Stop when pointing to TCB3 */
	} while (timer_B < (TCB_t *)&TCB3);



/* Stuff for synchronizing PWM timers */
// 	/* Restart TCA to sync TCBs */
// 	/* should not be needed		*/
// 	TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESTART_gc;
// 	TCA0.SINGLE.CTRLECLR = TCA_SINGLE_CMD_RESTART_gc;
//
// 	timer_B = (TCB_t *)&TCB0;
//
// 	/* TCB are sync to TCA, remove setting	*/
// 	for (uint8_t digitial_pin_timer = (TIMERB0 - TIMERB0);
// 	digitial_pin_timer < (TIMERB3 - TIMERB0);
// 	digitial_pin_timer++)
// 	{
// 		/* disable sync with tca */
// 		timer_B->CTRLA &= ~ (TCB_SYNCUPD_bm);
//
// 		/* Add offset to register	*/
// 		timer_B++;
//
// 	}

/********************************* ADC ****************************************/

//#if defined(ADC0)

	/************* Need to double check no other init required for ADC ***************/

	/* ADC clock between 50-200KHz */

	#if F_CPU >= 16000000 // 16 MHz / 128 = 125 KHz
		ADC0.CTRLC |= ADC_PRESC_DIV128_gc;
	#elif F_CPU >= 8000000 // 8 MHz / 64 = 125 KHz
		ADC0.CTRLC |= ADC_PRESC_DIV64_gc;
	#elif F_CPU >= 4000000 // 4 MHz / 32 = 125 KHz
		ADC0.CTRLC |= ADC_PRESC_DIV32_gc;
	#elif F_CPU >= 2000000 // 2 MHz / 16 = 125 KHz
		ADC0.CTRLC |= ADC_PRESC_DIV16_gc;
	#elif F_CPU >= 1000000 // 1 MHz / 8 = 125 KHz
		ADC0.CTRLC |= ADC_PRESC_DIV8_gc;
	#else // 128 kHz / 2 = 64 KHz -> This is the closest you can get, the prescaler is 2
		ADC0.CTRLC |= ADC_PRESC_DIV2_gc;
	#endif

	/* Enable ADC */
	ADC0.CTRLA |= ADC_ENABLE_bm;

//#endif

/****************************** TWI *******************************************/

// 	PORTMUX.TWISPIROUTEA = (PORTMUX_TWI0_ALT1_gc | PORTMUX_SPI0_ALT2_gc);


/****************************** USART *****************************************/

#ifdef REV_A_ENGINEERING_SAMPLE
	/* Configure PORTMUX for USARTS */
	//PORTMUX.USARTROUTEA = (PORTMUX_USART1_ALT1_gc // MAIN
	//| PORTMUX_USART0_ALT1_gc // SPARE
	//| PORTMUX_USART3_ALT1_gc); // DEBUG

	PORTMUX.USARTROUTEA = (PORTMUX_USART0_ALT1_gc // SPARE
	| PORTMUX_USART3_ALT1_gc); // DEBUG
#else
	/* Configure PORTMUX for USARTS */
	PORTMUX.USARTROUTEA = (PORTMUX_USART1_ALT1_gc // MAIN
					| PORTMUX_USART0_ALT1_gc // SPARE
					| PORTMUX_USART3_ALT1_gc); // DEBUG
#endif

/********************* TCB3 for system time tracking **************************/

	/* Calculate relevant time tracking values */
	microseconds_per_timerb3_overflow = clockCyclesToMicroseconds(TIME_TRACKING_CYCLES_PER_OVF);
	microseconds_per_timerb3_tick = microseconds_per_timerb3_overflow/TIME_TRACKING_TIMER_PERIOD;

	millis_inc = microseconds_per_timerb3_overflow / 1000;
	fract_inc = ((microseconds_per_timerb3_overflow % 1000));

	/* Default Periodic Interrupt Mode */
	/* TOP value for overflow every 1024 clock cycles */
	TCB3.CCMP = TIME_TRACKING_TIMER_PERIOD;

	/* Enable TCB3 interrupt */
	TCB3.INTCTRL |= TCB_CAPT_bm;

	/* Clock selection -> same as TCA (F_CPU/64 -- 250kHz) */
	TCB3.CTRLA = TCB_CLKSEL_CLKTCA_gc;

	/* Enable & start */
	TCB3.CTRLA |= TCB_ENABLE_bm;	/* Keep this last before enabling interrupts to ensure tracking as accurate as possible */

/*************************** ENABLE GLOBAL INTERRUPTS *************************/

	sei();
}
