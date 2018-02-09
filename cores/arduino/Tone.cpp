/* Tone.cpp

  A Tone Generator Library 
  
  Written by Brett Hagman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  -------- ----------- -------- --------
  0001    B Hagman    09/08/02 Initial coding
  0002    B Hagman    09/08/18 Multiple pins
  0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
  0004    B Hagman    09/09/26 Fixed problems with ATmega8
  0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                      09/11/25 Changed pin toggle method to XOR
                      09/11/25 Fixed timer0 from being excluded
  0006    D Mellis    09/12/29 Replaced objects with functions
  0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
  0008    S Kanemoto  12/06/22 Fixed for Leonardo by @maris_HY
  0009    J Reucker   15/04/10 Issue #292 Fixed problems with ATmega8 (thanks to Pete62)
  0010    jipp        15/04/13 added additional define check #2923
  0011	   E Roy	   13/02/18 ported to ATmega4809
  *************************************************/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"

/* For more than one tone, change AVAILABLE_TONE_PINS and uncomment the correct
	number of timers 
*/
#define AVAILABLE_TONE_PINS 1	

#define USE_TIMERB1		// interferes with PWM on pin 3
/*
#define USE_TIMERB2		// interferes with PWM on pin 11
#define USE_TIMERB0		// interferes with PWM on pin 6
*/

// Can't use TIMERB3 -- used for application time tracking 
// Using TIMERA0 NOT RECOMMENDED -- all other timers use its clock!
const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 
												#if defined(USE_TIMERB1)
												TIMERB1, 
												#endif
												#if defined(USE_TIMERB2)
												TIMERB2,	
												#endif
												#if defined(USE_TIMERB0)
												TIMERB0,
												#endif	 
												};
static uint8_t tone_pins[AVAILABLE_TONE_PINS] ={
												#if defined(USE_TIMERB1)
												NOT_A_PIN,
												#endif
												#if defined(USE_TIMERB2)
												NOT_A_PIN,
												#endif
												#if defined(USE_TIMERB0)
												NOT_A_PIN,
												#endif
												};
	
// timerx_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)

#if defined(USE_TIMERB0)
volatile long timerb0_toggle_count;
volatile uint8_t *timerb0_outtgl_reg;
volatile uint8_t timerb0_bit_mask;
#endif

#if defined(USE_TIMERB1)
	
volatile long timerb1_toggle_count;
volatile uint8_t *timerb1_outtgl_reg;
volatile uint8_t timerb1_bit_mask;
#endif

#if defined(USE_TIMERB2)
volatile long timerb2_toggle_count;
volatile uint8_t *timerb2_outtgl_reg;
volatile uint8_t timerb2_bit_mask;
#endif

static int8_t toneBegin(uint8_t _pin)
{
	int8_t _timer = -1;

	// If pin already being used for tone, return the associated timer  
	for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
		if (tone_pins[i] == _pin) {
			return pgm_read_byte(tone_pin_to_timer_PGM + i);
		}
	}
  
	// If not, search for an unused timer
	for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
		if (tone_pins[i] == NOT_A_PIN) {
			tone_pins[i] = _pin;
			_timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
		break;
		}
	}

	return _timer;
}



// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
	long toggle_count = 0;
	uint32_t compare_val = 0;

	// Initialize, get timer
	int8_t _timer = toneBegin(_pin);
	
	// If a valid timer was returned
	if (_timer > NOT_ON_TIMER){
			
		// Get pin related stuff
		PORT_t *port = digitalPinToPortStruct(_pin);
		uint8_t *port_outtgl = (uint8_t *)&(port->OUTTGL);
		uint8_t bit_mask = digitalPinToBitMask(_pin);
		
		// Set the pinMode as OUTPUT
		pinMode(_pin, OUTPUT);
		
		// Calculate compare value
		compare_val = F_CPU_CORRECTED / frequency / 2 - 1;
		// If compare larger than 16bits, need to prescale (will be DIV64)
		uint8_t prescaler_needed = 0;
		if (compare_val > 0xFFFF){
			// recalculate with new prescaler
			compare_val = F_CPU_CORRECTED / frequency / 2 / 64 - 1;
			prescaler_needed = 1;
		}
		
		// Calculate the toggle count
		if (duration > 0){	// Duration defined
			toggle_count = 2 * frequency * duration / 1000;
		} else {			// Duration not defined -- tone until noTone() call
			toggle_count = -1;
		}
			
		// Timer settings -- will be type B
				
		// Get timer struct
		TCB_t *timer_B = ((TCB_t *)&TCB0 + (_timer - TIMERB0));
			
		// Disable for now, set clk according to 'prescaler_needed'	
		// (Prescaled clock will come from TCA -- 
		//  by default it should have a prescaler of 64 (250kHz clock)
		// TCA default initialization is in wiring.c -- init()  )
		if(prescaler_needed){
			timer_B->CTRLA = TCB_CLKSEL_CLKTCA_gc;
		} else {
			timer_B->CTRLA = TCB_CLKSEL_CLKDIV1_gc;	
		}	
						
		// Timer to Periodic interrupt mode
		// This write will also disable any active PWM outputs
		timer_B->CTRLB = TCB_CNTMODE_INT_gc;
				
		// Write compare register
		timer_B->CCMP = compare_val;
			
		// Enable interrupt
		timer_B->INTCTRL = TCB_CAPTEI_bm;
				
		// Populate variables needed in interrupt
		#if defined(USE_TIMERB1)
		if(_timer == TIMERB1){
			timerb1_outtgl_reg = port_outtgl;
			timerb1_bit_mask = bit_mask;
			timerb1_toggle_count = toggle_count;
		}
		#endif
		#if defined(USE_TIMERB2)			
		if(_timer == TIMERB2){
			timerb2_outtgl_reg = port_outtgl;
			timerb2_bit_mask = bit_mask;
			timerb2_toggle_count = toggle_count;
		}
		#endif
		#if defined(USE_TIMERB0)			
		if(_timer == TIMERB0){
			timerb0_outtgl_reg = port_outtgl;
			timerb0_bit_mask = bit_mask;
			timerb0_toggle_count = toggle_count;
		}
		#endif
		
			
		// Enable timer
		timer_B->CTRLA |= TCB_ENABLE_bm;
				
	}
}

/* Works for all timers -- the timer being disabled will go back to the 
	configuration it had to output PWM for analogWrite() */
void disableTimer(uint8_t _timer)
{
	// Reinit back to producing PWM -- timer will be type B
	  
	// Get timer struct
	TCB_t *timer_B = ((TCB_t *)&TCB0 + (_timer - TIMERB0));
			
	// Disable interrupt
	timer_B->INTCTRL = 0;			  
			
	// Disable timer
	timer_B->CTRLA = 0;
			
	// RESTORE PWM FUNCTIONALITY:
			
	/* 8 bit PWM mode, but do not enable output yet, will do in analogWrite() */
	timer_B->CTRLB = (TCB_CNTMODE_PWM8_gc);

	/* Assign 8-bit period */
	timer_B->CCMPL = PWM_TIMER_PERIOD;

	/* default duty 50%, set when output enabled */
	timer_B->CCMPH = PWM_TIMER_COMPARE;

	/* Use TCA clock (250kHz) and enable */
	/* (sync update commented out, might try to synchronize later */
	timer_B->CTRLA = (TCB_CLKSEL_CLKTCA_gc)	| (TCB_ENABLE_bm);
			
}

// pin which currently is being used for a tone
void noTone(uint8_t _pin)
{
	int8_t _timer = NOT_ON_TIMER;
  
	// Find timer associated with pin
	for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
		
		// Got a match!
		if (tone_pins[i] == _pin) {
			
			// Get timer
			_timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
			
			// Reset pin to nothing
			tone_pins[i] = NOT_A_PIN;
			
			// Stop looking
			break;
		}
	}

	if(_timer > NOT_ON_TIMER){
		disableTimer(_timer);
	
		// Keep pin low after disabling of timer
		digitalWrite(_pin, LOW);
	}

}

// helper function for noTone()

#ifdef USE_TIMERB0
ISR(TCB0_INT_vect)
{
	if (timerb0_toggle_count != 0){
		
		// toggle the pin
		*timerb0_outtgl_reg = timerb0_bit_mask;
		
		// If duration was defined, decrement
		if (timerb0_toggle_count > 0){
			timerb0_toggle_count--;
		}
		
		// If no duration (toggle count negative), go on until noTone() call
		
	} else {	// If toggle count = 0, stop
		
		disableTimer(TIMERB0);
		
		// keep pin low after stop (OUTCLR = OUTTGL - 1)
		*(timerb0_outtgl_reg - 1) = timerb0_bit_mask;
	}
	
	/* Clear flag */
	TCB0.INTFLAGS = TCB_CAPT_bm;
}
#endif

#ifdef USE_TIMERB1
ISR(TCB1_INT_vect)
{
	if (timerb1_toggle_count != 0){
		
		// toggle the pin
		*timerb1_outtgl_reg = timerb1_bit_mask;
		
		// If duration was defined, decrement
		if (timerb1_toggle_count > 0){
			timerb1_toggle_count--;
		}
		
		// If no duration (toggle count negative), go on until noTone() call
		
	} else {	// If toggle count = 0, stop
		
		disableTimer(TIMERB1);
		
		// keep pin low after stop (OUTCLR = OUTTGL - 1)
		*(timerb1_outtgl_reg - 1) = timerb1_bit_mask;
	}
	
	/* Clear flag */
	TCB1.INTFLAGS = TCB_CAPT_bm;
}
#endif

#ifdef USE_TIMERB2
ISR(TCB2_INT_vect)
{
	if (timerb2_toggle_count != 0){
		
		// toggle the pin
		*timerb2_outtgl_reg = timerb2_bit_mask;
		
		// If duration was defined, decrement
		if (timerb2_toggle_count > 0){
			timerb2_toggle_count--;
		}
		
		// If no duration (toggle count negative), go on until noTone() call
		
	} else {	// If toggle count = 0, stop
		
		disableTimer(TIMERB2);
		
		// keep pin low after stop (OUTCLR = OUTTGL - 1)
		*(timerb2_outtgl_reg - 1) = timerb2_bit_mask;
	}
	
	/* Clear flag */
	TCB2.INTFLAGS = TCB_CAPT_bm;
}
#endif

