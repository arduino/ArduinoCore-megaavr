#pragma once
#include "Arduino.h"

//#define COMPATIBILITY_DEBUG true

#define PORTA_ARDUINO        (*(PORT_t *) 0x0400) /* I/O Ports */
#define PORTB_ARDUINO        (*(PORT_t *) 0x0420) /* I/O Ports */
#define PORTC_ARDUINO        (*(PORT_t *) 0x0440) /* I/O Ports */
#define PORTD_ARDUINO        (*(PORT_t *) 0x0460) /* I/O Ports */
#define PORTE_ARDUINO        (*(PORT_t *) 0x0480) /* I/O Ports */
#define PORTF_ARDUINO        (*(PORT_t *) 0x04A0) /* I/O Ports */

#define PORTD_OFFSET    0
#define PORTB_OFFSET    8
#define PORTC_OFFSET    14

#undef PORTB
#undef PORTC
#undef PORTD

#ifdef COMPATIBILITY_DEBUG
static inline void printPortAndPin(int port, uint8_t pin, bool value) {
  Serial.print("Writing ");
  Serial.print(value);
  Serial.print(" on ");

  switch (port) {
    case (int) &PORTA_ARDUINO: Serial.print("PORTA"); break;
    case (int) &PORTB_ARDUINO: Serial.print("PORTB"); break;
    case (int) &PORTC_ARDUINO: Serial.print("PORTC"); break;
    case (int) &PORTD_ARDUINO: Serial.print("PORTD"); break;
    case (int) &PORTE_ARDUINO: Serial.print("PORTE"); break;
    case (int) &PORTF_ARDUINO: Serial.print("PORTF"); break;
  }

  Serial.println(pin);
}
#endif

typedef struct pinPort {
  volatile PORT_t*  port;
  uint8_t pin;
};

/** DDR Classes**/
class DDRClass {
  public:
    DDRClass(uint8_t _offset, uint8_t _limit, pinPort* _mapping): offset(_offset), limit(_limit), mapping(_mapping) {}
    DDRClass& operator=(uint8_t value) {
      for (int i = 0; i < limit; i++) {
        if (value & (1 << i)) {
          mapping[i + offset].port->DIR |= ( 1 << mapping[i + offset].pin);
        } else {
          mapping[i + offset].port->DIR &= ~( 1 << mapping[i + offset].pin);
        }
      }
      registerValue = value;
      return *this;
    }
    DDRClass& operator&=(uint8_t value) {
      registerValue &= value;
      for (int i = 0; i < limit; i++) {
        if (registerValue & (1 << i)) {
          mapping[i + offset].port->DIR |= ( 1 << mapping[i + offset].pin);
        } else {
          mapping[i + offset].port->DIR &= ~( 1 << mapping[i + offset].pin);
        }
      }
      return *this;
    }
    DDRClass& operator|=(uint8_t value) {
      registerValue |= value;
      for (int i = 0; i < limit; i++) {
        if (registerValue & (1 << i)) {
          mapping[i + offset].port->DIR |= ( 1 << mapping[i + offset].pin);
        } else {
          mapping[i + offset].port->DIR &= ~( 1 << mapping[i + offset].pin);
        }
      }
      return *this;
    }
  private:
    uint8_t offset, limit, registerValue = 0;
    pinPort* mapping;
};

extern DDRClass DDRB;
extern DDRClass DDRC;
extern DDRClass DDRD;

/** PORT Classes**/
class PORTClass {
  public:
    PORTClass(uint8_t _offset, uint8_t _limit, pinPort* _mapping): offset(_offset), limit(_limit), mapping(_mapping) {}
    PORTClass& operator=(uint8_t value) {
      registerValue = value;
      for (int i = 0; i < limit; i++) {
        if (value & (1 << i)) {
#ifdef COMPATIBILITY_DEBUG
          printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, true);
#endif
          mapping[i + offset].port->OUTSET =  ( 1 << mapping[i + offset].pin);
        } else {
#ifdef COMPATIBILITY_DEBUG
          printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, false);
#endif
          mapping[i + offset].port->OUTCLR = ( 1 << mapping[i + offset].pin);
        }
      }
      return *this;
    }

    PORTClass& operator&=(uint8_t value) {
      registerValue &= value;
      for (int i = 0; i < limit; i++) {
        if (registerValue & (1 << i)) {
#ifdef COMPATIBILITY_DEBUG
          printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, true);
#endif
          mapping[i + offset].port->OUTSET =  ( 1 << mapping[i + offset].pin);
        } else {
#ifdef COMPATIBILITY_DEBUG
          printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, false);
#endif
          mapping[i + offset].port->OUTCLR = ( 1 << mapping[i + offset].pin);
        }
      }
      return *this;
    }

    PORTClass& operator|=(uint8_t value) {
      registerValue |= value;
      for (int i = 0; i < limit; i++) {
        if (registerValue & (1 << i)) {
#ifdef COMPATIBILITY_DEBUG
          printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, true);
#endif
          mapping[i + offset].port->OUTSET =  ( 1 << mapping[i + offset].pin);
        } else {
#ifdef COMPATIBILITY_DEBUG
          printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, false);
#endif
          mapping[i + offset].port->OUTCLR = ( 1 << mapping[i + offset].pin);
        }
      }
      return *this;
    }
  private:
    uint8_t offset, limit, registerValue = 0;
    pinPort* mapping;
};

extern PORTClass PORTB;
extern PORTClass PORTC;
extern PORTClass PORTD;
