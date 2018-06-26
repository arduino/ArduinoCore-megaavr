#include "UNO_compat.h"

/*
  ARDUINO PIN  ATMEGA 328  ATMEGA 4809
  0         PD0         PC5
  1         PD1         PC4
  2         PD2         PA0
  3         PD3         PF5
  4         PD4         PC6
  5         PD5         PB2
  6         PD6         PF4
  7         PD7         PA1
  8         PB0       PE3
  9         PB1       PB0
  10          PB2       PB1
  11          PB3       PE0
  12          PB4       PE1
  13          PB5       PE2
  A0          PC0       PD0
  A1          PC1       PD1
  A2          PC2       PD2
  A3          PC3       PD3
  A4          PC4       PD4
  A5          PC5       PD5
*/

pinPort mapping[20] = {
  {&PORTC_ARDUINO, 5},
  {&PORTC_ARDUINO, 4},
  {&PORTA_ARDUINO, 0},
  {&PORTF_ARDUINO, 5},
  {&PORTC_ARDUINO, 6},
  {&PORTB_ARDUINO, 2},
  {&PORTF_ARDUINO, 4},
  {&PORTA_ARDUINO, 1},
  {&PORTE_ARDUINO, 3},
  {&PORTB_ARDUINO, 0},
  {&PORTB_ARDUINO, 1},
  {&PORTE_ARDUINO, 0},
  {&PORTE_ARDUINO, 1},
  {&PORTE_ARDUINO, 2},
  {&PORTD_ARDUINO, 0},
  {&PORTD_ARDUINO, 1},
  {&PORTD_ARDUINO, 2},
  {&PORTD_ARDUINO, 3},
  {&PORTD_ARDUINO, 4},
  {&PORTD_ARDUINO, 5},
};


PORTClass PORTB(PORTB_OFFSET, 6, mapping);
PORTClass PORTC(PORTC_OFFSET, 6, mapping);
PORTClass PORTD(PORTD_OFFSET, 8, mapping);

DDRClass DDRB(PORTB_OFFSET, 6, mapping);
DDRClass DDRC(PORTC_OFFSET, 6, mapping);
DDRClass DDRD(PORTD_OFFSET, 8, mapping);
