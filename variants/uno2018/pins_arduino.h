/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

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

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6
#define NUM_TESTPOINT_PINS			9
#define NUM_OCCUPIED_PINS			6 // (TOSC1/2, VREF, RESET, DEBUG USART Rx/Tx)
#define NUM_TOTAL_FREE_PINS			(NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS + NUM_TESTPOINT_PINS)
#define NUM_TOTAL_PINS				(NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS + NUM_TESTPOINT_PINS + NUM_OCCUPIED_PINS)
#define ANALOG_INPUT_OFFSET			14
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + ANALOG_INPUT_OFFSET : NOT_A_PIN)

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)

#define PIN_SPI_SS    (10)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (20)
#define PIN_WIRE_SCL        (21)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// Uno2 Main USART available on Arduino header pins
// USART1 on mega4809 (alternative pins)
// Mapped to HWSERIAL0 in Serial library
#define HWSERIAL0 (&USART1)
#define HWSERIAL0_DRE_VECTOR (USART1_DRE_vect)
#define HWSERIAL0_DRE_VECTOR_NUM (USART1_DRE_vect_num)
#define HWSERIAL0_RXC_VECTOR (USART1_RXC_vect)
#ifdef REV_A_ENGINEERING_SAMPLE
#define PIN_WIRE_HWSERIAL0_RX (12)
#define PIN_WIRE_HWSERIAL0_TX (11)
#else
#define PIN_WIRE_HWSERIAL0_RX (0)
#define PIN_WIRE_HWSERIAL0_TX (1)
#endif

// Uno2 Debug USART (not available on headers, only via the EDGB virtual COM port)
// USART3 on mega4809 (alternative pins)
// Mapped to HWSERIAL1 in Serial library
#define HWSERIAL1 (&USART3)
#define HWSERIAL1_DRE_VECTOR (USART3_DRE_vect)
#define HWSERIAL1_DRE_VECTOR_NUM (USART3_DRE_vect_num)
#define HWSERIAL1_RXC_VECTOR (USART3_RXC_vect)
#define PIN_WIRE_HWSERIAL1_RX (26)
#define PIN_WIRE_HWSERIAL1_TX (27)

// Uno2 Spare USART available on testpoints
// USART0 on mega4809 (alternative pins)
// Mapped to HWSERIAL2 in Serial library
#define HWSERIAL2 (&USART0)
#define HWSERIAL2_DRE_VECTOR (USART0_DRE_vect)
#define HWSERIAL2_DRE_VECTOR_NUM (USART0_DRE_vect_num)
#define HWSERIAL2_RXC_VECTOR (USART0_RXC_vect)
#define PIN_WIRE_HWSERIAL2_RX (23)
#define PIN_WIRE_HWSERIAL2_TX (24)

#define LED_BUILTIN 25

#define PIN_A0   (0)
#define PIN_A1   (1)
#define PIN_A2   (2)
#define PIN_A3   (3)
#define PIN_A4   (4)
#define PIN_A5   (5)
#define PIN_A6   (6)
#define PIN_A7   (7)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEGA4809 / ARDUINO
//
//						SCL  SDA
//					   (A5*)(A4*)	   (7)  (2)			   (R)		(3~) (6~)
//				   PA4  PA3  PA2  PA1  PA0  GND  VDD  UPDI PF6  PF5  PF4  PF3
//
// 					48   47   46   45   44   43   42   41   40   39   38   37
//				 + ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ +
//		  PA5   1|															   |36  PF2
//		  PA6   2|															   |35  PF1 (TOSC2)
//	      PA7   3|															   |34  PF0 (TOSC1)
//   (9~) PB0   4|															   |33  PE3 (8)
//  (10~) PB1   5|															   |32  PE2
//   (5~) PB2   6|															   |31  PE1
//		  PB3   7|							48pin QFN						   |30  PE0
//   (Tx) PB4   8|															   |29  GND
//   (Rx) PB5   9|															   |28  AVDD
//	(11~) PC0  10|															   |27  PD7 (VREF)
//	 (12) PC1  11|															   |26  PD6
//	 (13) PC2  12|															   |25  PD5 (A5)
//			     + ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ +
//		   			13   14   15   16   17   18   19   20   21   22   23   24
//
//	                PC3  VDD  GND  PC4  PC5  PC6  PC7  PD0  PD1  PD2  PD3  PD4
//				   (10~*)		   (1)	(0)	 (4)	   (A0) (A1) (A2) (A3) (A4)
//

const uint8_t PROGMEM digital_pin_to_interrupt[] = {
	19, //0	USART1_Rx PC5
	18, //1	USART1_Tx PC4
	0, //2 PA0
	39, //3 PF5
	20, //4 PC6
	10, //5 PB2
	38, //6 PF4
	1, //7 PA1
	33, //8 PE3
	8, //9 PB0
	9, //10 PB1
	14, //11 PC0
	15, //12 PC1
	16, //13 PC2
	22, //14 AI0 PD0
	23, //15 AI1 PD1
	24, //16 AI2 PD2
	25, //17 AI3 PD3
	26, //18 AI4 PD4
	27, //19 AI5 PD5
	2, //20 TWI_SDA PA2
	3, //21 TWI_SCL PA3
	17, //22 SPI SS PC3
	5, //23 USART0_Rx PA5
	4, //24 USART0_Tx PA4
	28, //25 LED_BUILTIN PD6
	13, //26 USART3_Rx PB5
	12, //27 USART3_Tx PB4
	6, // 28 PA6
	7, // 29 PA7
	11, // 30 PB3
	21, // 31 PC7
	30, // 32 PE0
	31, // 33 PE1
	32, // 34 PE2
	36, // 35 PF2
	37, // 36 PF3
	34, // 37 PF0 TOSC 1
	35, // 38 PF1 TOSC 2
	29, // 39 PD7 VREF
	40 // 40 PF6 RESET
	};

const uint8_t PROGMEM port_interrupt_offset[] = {
	0, //PA
	8, //PB
	14, //PC
	22, //PD
	30, //PE
	34, //PF
};

const uint8_t PROGMEM digital_pin_to_port[] = {
	PC, //0 USART1_Rx PC5
	PC, //1 USART1_Tx PC4
	PA, //2 PA0
	PF, //3 PF5
	PC, //4 PC6
	PB, //5 PB2
	PF, //6 PF4
	PA, //7 PA1
	PE, //8 PE3
	PB, //9 PB0
	PB, //10 PB1
	PC, //11 PC0
	PC, //12 PC1
	PC, //13 PC2
	PD, //14 AI0 PD0
	PD, //15 AI1 PD1
	PD, //16 AI2 PD2
	PD, //17 AI3 PD3
	PD, //18 AI4 PD4
	PD, //19 AI5 PD5
	PA, //20 TWI_SDA PA2
	PA, //21 TWI_SCL PA3
	PC, //22 SPI SS PC3
	PA, //23 USART0_Rx PA5
	PA, //24 USART0_Tx PA4
	PD, //25 LED_BUILTIN PD6
	PB, //26 USART3_Rx PB5
	PB, //27 USART3_Tx PB4
	PA,	// 28 PA6
	PA,	// 29 PA7
	PB,	// 30 PB3
	PC,	// 31 PC7
	PE,	// 32 PE0
	PE,	// 33 PE1
	PE,	// 34 PE2
	PF,	// 35 PF2
	PF,	// 36 PF3
	PF,	// 37 PF0 TOSC 1
	PF,	// 38 PF1 TOSC 2
	PD,	// 39 PD7 VREF
	PF	// 40 PF6 RESET
};

/* Use this for accessing PINnCTRL register */
const uint8_t PROGMEM digital_pin_to_bit_position[] = {
	PIN5_bp,	//0 USART1_Rx PC5
	PIN4_bp,	//1 USART1_Tx PC4
	PIN0_bp,	//2 PA0
	PIN5_bp,	//3 PF5
	PIN6_bp,	//4 PC6
	PIN2_bp,	//5 PB2
	PIN4_bp,	//6 PF4
	PIN1_bp,	//7 PA1
	PIN3_bp,	//8 PE3
	PIN0_bp,	//9 PB0
	PIN1_bp,	//10 PB1
	PIN0_bp,	//11 PC0
	PIN1_bp,	//12 PC1
	PIN2_bp,	//13 PC2
	PIN0_bp,	//14 AI0 PD0
	PIN1_bp,	//15 AI1 PD1
	PIN2_bp,	//16 AI2 PD2
	PIN3_bp,	//17 AI3 PD3
	PIN4_bp,	//18 AI4 PD4
	PIN5_bp,	//19 AI5 PD5
	PIN2_bp,	//20 TWI_SDA PA2
	PIN3_bp,	//21 TWI_SCL PA3
	PIN3_bp,	//22 SPI SS PC3
	PIN5_bp,	//23 USART0_Rx PA5
	PIN4_bp,	//24 USART0_Tx PA4
	PIN6_bp,	//25 LED_BUILTIN PD6
	PIN5_bp,	//26 USART3_Rx PB5
	PIN4_bp,	//27 USART3_Tx PB4
	PIN6_bp,	// 28 PA6
	PIN7_bp,	// 29 PA7
	PIN3_bp,	// 30 PB3
	PIN7_bp,	// 31 PC7
	PIN0_bp,	// 32 PE0
	PIN1_bp,	// 33 PE1
	PIN2_bp,	// 34 PE2
	PIN2_bp,	// 35 PF2
	PIN3_bp,	// 36 PF3
	PIN0_bp,	// 37 PF0 TOSC 1
	PIN1_bp,	// 38 PF1 TOSC 2
	PIN7_bp,	// 39 PD7 VREF
	PIN6_bp 	// 40 PF6 RESET
};

const uint8_t PROGMEM digital_pin_to_timer[] = {
	NOT_ON_TIMER,	//0 USART1_Tx PC5
	NOT_ON_TIMER,	//1 USART1_Tx PC4
	NOT_ON_TIMER,	//2 PA0
	TIMERB1,		//3 PF5	// TCB1 WO
	NOT_ON_TIMER,	//4 PC6
	TIMERA0,		//5 PB2	// TCA0 WO2
	TIMERB0,		//6 PF4	// TCB0 WO
	NOT_ON_TIMER,	//7 PA1
	NOT_ON_TIMER,	//8 PE3
	TIMERA0,		//9 PB0		// TCA0 WO0
	TIMERA0,		//10 PB1	// TCA0 WO1
	TIMERB2,		//11 PC0	// TCB2 WO2
	NOT_ON_TIMER,	//12 PC1
	NOT_ON_TIMER,	//13 PC2
	NOT_ON_TIMER,	//14 AI0 PD0
	NOT_ON_TIMER,	//15 AI1 PD1
	NOT_ON_TIMER,	//16 AI2 PD2
	NOT_ON_TIMER,	//17 AI3 PD3
	NOT_ON_TIMER,	//18 AI4 PD4
	NOT_ON_TIMER,	//19 AI5 PD5
	NOT_ON_TIMER,	//20 TWI_SDA PA2
	NOT_ON_TIMER,	//21 TWI_SCL PA3
	NOT_ON_TIMER,	//22 SPI SS PC3
	NOT_ON_TIMER,	//23 USART0_Rx PA5
	NOT_ON_TIMER,	//24 USART0_Tx PA4
	NOT_ON_TIMER,	//25 LED_BUILTIN PD6
	NOT_ON_TIMER,	//26 USART3_Rx PB5
	NOT_ON_TIMER,	//27 USART3_Tx PB4
	NOT_ON_TIMER,	// 28 PA6
	NOT_ON_TIMER,	// 29 PA7
	NOT_ON_TIMER,	// 30 PB3
	NOT_ON_TIMER,	// 31 PC7
	NOT_ON_TIMER,	// 32 PE0
	NOT_ON_TIMER,	// 33 PE1
	NOT_ON_TIMER,	// 34 PE2
	NOT_ON_TIMER,	// 35 PF2
	NOT_ON_TIMER,	// 36 PF3
	NOT_ON_TIMER,	// 37 PF0 TOSC 1
	NOT_ON_TIMER,	// 38 PF1 TOSC 2
	NOT_ON_TIMER,	// 39 PD7 VREF
	NOT_ON_TIMER,	// 40 PF6 RESET
};


#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR			Serial
#define SERIAL_PORT_HARDWARE		Serial
#define SERIAL_PORT_USBVIRTUAL		Serial1
#define SERIAL_PORT_HARDWARE_OPEN	Serial2

#endif
