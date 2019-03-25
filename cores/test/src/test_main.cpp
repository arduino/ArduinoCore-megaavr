/*
  Copyright (c) 2019 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "../../arduino/NANO_Compat.h"

/*****************************************************************************/

SCENARIO("Testing Arduino Nano 4809 DDRB compatibility class", "NANO_Compat::DDRBClass") {
  PORT_t portb, porte;

  DDRBClass DDRB(&portb, &porte);

  REQUIRE(portb.DIR() == 0);
  REQUIRE(porte.DIR() == 0);

  GIVEN("Testing operator = ") {
    WHEN("ATMEGA328P DDRB = (1<<0)") { DDRB = (1<<0); THEN("ATMEGA4809 PORTE.DIR = (1<<3)") REQUIRE(porte.DIR() == (1<<3)); }
    WHEN("ATMEGA328P DDRB = (1<<1)") { DDRB = (1<<1); THEN("ATMEGA4809 PORTB.DIR = (1<<0)") REQUIRE(portb.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRB = (1<<2)") { DDRB = (1<<2); THEN("ATMEGA4809 PORTB.DIR = (1<<1)") REQUIRE(portb.DIR() == (1<<1)); }
    WHEN("ATMEGA328P DDRB = (1<<3)") { DDRB = (1<<3); THEN("ATMEGA4809 PORTE.DIR = (1<<0)") REQUIRE(porte.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRB = (1<<4)") { DDRB = (1<<4); THEN("ATMEGA4809 PORTE.DIR = (1<<1)") REQUIRE(porte.DIR() == (1<<1)); }
    WHEN("ATMEGA328P DDRB = (1<<5)") { DDRB = (1<<5); THEN("ATMEGA4809 PORTE.DIR = (1<<2)") REQUIRE(porte.DIR() == (1<<2)); }
  }

  GIVEN("Testing operator &= ") {
    portb.DIR.set(0xFF);
    porte.DIR.set(0xFF);
    WHEN("ATMEGA328P DDRB &= ~(1<<0)") { DDRB &= ~(1<<0); THEN("ATMEGA4809 PORTE.DIR(3) = 0") REQUIRE(porte.DIR() == 0xF7); }
    WHEN("ATMEGA328P DDRB &= ~(1<<1)") { DDRB &= ~(1<<1); THEN("ATMEGA4809 PORTB.DIR(0) = 0") REQUIRE(portb.DIR() == 0xFE); }
    WHEN("ATMEGA328P DDRB &= ~(1<<2)") { DDRB &= ~(1<<2); THEN("ATMEGA4809 PORTB.DIR(1) = 0") REQUIRE(portb.DIR() == 0xFD); }
    WHEN("ATMEGA328P DDRB &= ~(1<<3)") { DDRB &= ~(1<<3); THEN("ATMEGA4809 PORTE.DIR(0) = 0") REQUIRE(porte.DIR() == 0xFE); }
    WHEN("ATMEGA328P DDRB &= ~(1<<4)") { DDRB &= ~(1<<4); THEN("ATMEGA4809 PORTE.DIR(1) = 0") REQUIRE(porte.DIR() == 0xFD); }
    WHEN("ATMEGA328P DDRB &= ~(1<<5)") { DDRB &= ~(1<<5); THEN("ATMEGA4809 PORTE.DIR(2) = 0") REQUIRE(porte.DIR() == 0xFB); }
  }

  GIVEN("Testing operator |= ") {
    WHEN("ATMEGA328P DDRB |= (1<<0)") { DDRB |= (1<<0); THEN("ATMEGA4809 PORTE.DIR = (1<<3)") REQUIRE(porte.DIR() == (1<<3)); }
    WHEN("ATMEGA328P DDRB |= (1<<1)") { DDRB |= (1<<1); THEN("ATMEGA4809 PORTB.DIR = (1<<0)") REQUIRE(portb.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRB |= (1<<2)") { DDRB |= (1<<2); THEN("ATMEGA4809 PORTB.DIR = (1<<1)") REQUIRE(portb.DIR() == (1<<1)); }
    WHEN("ATMEGA328P DDRB |= (1<<3)") { DDRB |= (1<<3); THEN("ATMEGA4809 PORTE.DIR = (1<<0)") REQUIRE(porte.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRB |= (1<<4)") { DDRB |= (1<<4); THEN("ATMEGA4809 PORTE.DIR = (1<<1)") REQUIRE(porte.DIR() == (1<<1)); }
    WHEN("ATMEGA328P DDRB |= (1<<5)") { DDRB |= (1<<5); THEN("ATMEGA4809 PORTE.DIR = (1<<2)") REQUIRE(porte.DIR() == (1<<2)); }
  }
}

/*****************************************************************************/

SCENARIO("Testing Arduino Nano 4809 PORTB compatibility class", "NANO_Compat::PORTBClass") {
  PORT_t portb, porte;

  PORTBClass PORTB(&portb, &porte);

  REQUIRE(portb.OUT() == 0);
  REQUIRE(porte.OUT() == 0);

  GIVEN("Testing operator = ") {
    WHEN("ATMEGA328P PORTB = (1<<0)") { PORTB = (1<<0); THEN("ATMEGA4809 PORTE.OUT = (1<<3)") REQUIRE(porte.OUT() == (1<<3)); }
    WHEN("ATMEGA328P PORTB = (1<<1)") { PORTB = (1<<1); THEN("ATMEGA4809 PORTB.OUT = (1<<0)") REQUIRE(portb.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTB = (1<<2)") { PORTB = (1<<2); THEN("ATMEGA4809 PORTB.OUT = (1<<1)") REQUIRE(portb.OUT() == (1<<1)); }
    WHEN("ATMEGA328P PORTB = (1<<3)") { PORTB = (1<<3); THEN("ATMEGA4809 PORTE.OUT = (1<<0)") REQUIRE(porte.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTB = (1<<4)") { PORTB = (1<<4); THEN("ATMEGA4809 PORTE.OUT = (1<<1)") REQUIRE(porte.OUT() == (1<<1)); }
    WHEN("ATMEGA328P PORTB = (1<<5)") { PORTB = (1<<5); THEN("ATMEGA4809 PORTE.OUT = (1<<2)") REQUIRE(porte.OUT() == (1<<2)); }
  }

  GIVEN("Testing operator &= ") {
    portb.OUT.set(0xFF);
    porte.OUT.set(0xFF);
    WHEN("ATMEGA328P PORTB &= ~(1<<0)") { PORTB &= ~(1<<0); THEN("ATMEGA4809 PORTE.OUT(3) = 0") REQUIRE(porte.OUT() == 0xF7); }
    WHEN("ATMEGA328P PORTB &= ~(1<<1)") { PORTB &= ~(1<<1); THEN("ATMEGA4809 PORTB.OUT(0) = 0") REQUIRE(portb.OUT() == 0xFE); }
    WHEN("ATMEGA328P PORTB &= ~(1<<2)") { PORTB &= ~(1<<2); THEN("ATMEGA4809 PORTB.OUT(1) = 0") REQUIRE(portb.OUT() == 0xFD); }
    WHEN("ATMEGA328P PORTB &= ~(1<<3)") { PORTB &= ~(1<<3); THEN("ATMEGA4809 PORTE.OUT(0) = 0") REQUIRE(porte.OUT() == 0xFE); }
    WHEN("ATMEGA328P PORTB &= ~(1<<4)") { PORTB &= ~(1<<4); THEN("ATMEGA4809 PORTE.OUT(1) = 0") REQUIRE(porte.OUT() == 0xFD); }
    WHEN("ATMEGA328P PORTB &= ~(1<<5)") { PORTB &= ~(1<<5); THEN("ATMEGA4809 PORTE.OUT(2) = 0") REQUIRE(porte.OUT() == 0xFB); }
  }

  GIVEN("Testing operator |= ") {
    WHEN("ATMEGA328P PORTB |= (1<<0)") { PORTB |= (1<<0); THEN("ATMEGA4809 PORTE.OUT = (1<<3)") REQUIRE(porte.OUT() == (1<<3)); }
    WHEN("ATMEGA328P PORTB |= (1<<1)") { PORTB |= (1<<1); THEN("ATMEGA4809 PORTB.OUT = (1<<0)") REQUIRE(portb.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTB |= (1<<2)") { PORTB |= (1<<2); THEN("ATMEGA4809 PORTB.OUT = (1<<1)") REQUIRE(portb.OUT() == (1<<1)); }
    WHEN("ATMEGA328P PORTB |= (1<<3)") { PORTB |= (1<<3); THEN("ATMEGA4809 PORTE.OUT = (1<<0)") REQUIRE(porte.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTB |= (1<<4)") { PORTB |= (1<<4); THEN("ATMEGA4809 PORTE.OUT = (1<<1)") REQUIRE(porte.OUT() == (1<<1)); }
    WHEN("ATMEGA328P PORTB |= (1<<5)") { PORTB |= (1<<5); THEN("ATMEGA4809 PORTE.OUT = (1<<2)") REQUIRE(porte.OUT() == (1<<2)); }
  }
}

/*****************************************************************************/

SCENARIO("Testing Arduino Nano 4809 DDRC compatibility class", "NANO_Compat::DDRCClass") {
  PORT_t porta, portd;

  DDRCClass DDRC(&porta, &portd);

  REQUIRE(porta.DIR() == 0);
  REQUIRE(portd.DIR() == 0);

  GIVEN("Testing operator = ") {
    WHEN("ATMEGA328P DDRC = (1<<0)") { DDRC = (1<<0); THEN("ATMEGA4809 PORTD.DIR = (1<<3)") REQUIRE(portd.DIR() == (1<<3)); }
    WHEN("ATMEGA328P DDRC = (1<<1)") { DDRC = (1<<1); THEN("ATMEGA4809 PORTD.DIR = (1<<2)") REQUIRE(portd.DIR() == (1<<2)); }
    WHEN("ATMEGA328P DDRC = (1<<2)") { DDRC = (1<<2); THEN("ATMEGA4809 PORTD.DIR = (1<<1)") REQUIRE(portd.DIR() == (1<<1)); }
    WHEN("ATMEGA328P DDRC = (1<<3)") { DDRC = (1<<3); THEN("ATMEGA4809 PORTD.DIR = (1<<0)") REQUIRE(portd.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRC = (1<<4)") { DDRC = (1<<4); THEN("ATMEGA4809 PORTA.DIR = (1<<2)") REQUIRE(porta.DIR() == (1<<2)); }
    WHEN("ATMEGA328P DDRC = (1<<5)") { DDRC = (1<<5); THEN("ATMEGA4809 PORTA.DIR = (1<<3)") REQUIRE(porta.DIR() == (1<<3)); }
    WHEN("ATMEGA328P DDRC = (1<<6)") { DDRC = (1<<6); THEN("ATMEGA4809 PORTD.DIR = (1<<4)") REQUIRE(portd.DIR() == (1<<4)); }
    WHEN("ATMEGA328P DDRC = (1<<7)") { DDRC = (1<<7); THEN("ATMEGA4809 PORTD.DIR = (1<<5)") REQUIRE(portd.DIR() == (1<<5)); }
  }

  GIVEN("Testing operator &= ") {
    porta.DIR.set(0xFF);
    portd.DIR.set(0xFF);
    WHEN("ATMEGA328P DDRC &= ~(1<<0)") { DDRC &= ~(1<<0); THEN("ATMEGA4809 PORTD.DIR = 0xF7") REQUIRE(portd.DIR() == 0xF7); }
    WHEN("ATMEGA328P DDRC &= ~(1<<1)") { DDRC &= ~(1<<1); THEN("ATMEGA4809 PORTD.DIR = 0xFB") REQUIRE(portd.DIR() == 0xFB); }
    WHEN("ATMEGA328P DDRC &= ~(1<<2)") { DDRC &= ~(1<<2); THEN("ATMEGA4809 PORTD.DIR = 0xFD") REQUIRE(portd.DIR() == 0xFD); }
    WHEN("ATMEGA328P DDRC &= ~(1<<3)") { DDRC &= ~(1<<3); THEN("ATMEGA4809 PORTD.DIR = 0xFE") REQUIRE(portd.DIR() == 0xFE); }
    WHEN("ATMEGA328P DDRC &= ~(1<<4)") { DDRC &= ~(1<<4); THEN("ATMEGA4809 PORTA.DIR = 0xFB") REQUIRE(porta.DIR() == 0xFB); }
    WHEN("ATMEGA328P DDRC &= ~(1<<5)") { DDRC &= ~(1<<5); THEN("ATMEGA4809 PORTA.DIR = 0xF7") REQUIRE(porta.DIR() == 0xF7); }
    WHEN("ATMEGA328P DDRC &= ~(1<<6)") { DDRC &= ~(1<<6); THEN("ATMEGA4809 PORTD.DIR = 0xFE") REQUIRE(portd.DIR() == 0xEF); }
    WHEN("ATMEGA328P DDRC &= ~(1<<7)") { DDRC &= ~(1<<7); THEN("ATMEGA4809 PORTD.DIR = 0xDF") REQUIRE(portd.DIR() == 0xDF); }
  }

  GIVEN("Testing operator |= ") {
    WHEN("ATMEGA328P DDRC |= (1<<0)") { DDRC |= (1<<0); THEN("ATMEGA4809 PORTD.DIR = (1<<3)") REQUIRE(portd.DIR() == (1<<3)); }
    WHEN("ATMEGA328P DDRC |= (1<<1)") { DDRC |= (1<<1); THEN("ATMEGA4809 PORTD.DIR = (1<<2)") REQUIRE(portd.DIR() == (1<<2)); }
    WHEN("ATMEGA328P DDRC |= (1<<2)") { DDRC |= (1<<2); THEN("ATMEGA4809 PORTD.DIR = (1<<1)") REQUIRE(portd.DIR() == (1<<1)); }
    WHEN("ATMEGA328P DDRC |= (1<<3)") { DDRC |= (1<<3); THEN("ATMEGA4809 PORTD.DIR = (1<<0)") REQUIRE(portd.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRC |= (1<<4)") { DDRC |= (1<<4); THEN("ATMEGA4809 PORTA.DIR = (1<<2)") REQUIRE(porta.DIR() == (1<<2)); }
    WHEN("ATMEGA328P DDRC |= (1<<5)") { DDRC |= (1<<5); THEN("ATMEGA4809 PORTA.DIR = (1<<3)") REQUIRE(porta.DIR() == (1<<3)); }
    WHEN("ATMEGA328P DDRC |= (1<<6)") { DDRC |= (1<<6); THEN("ATMEGA4809 PORTD.DIR = (1<<4)") REQUIRE(portd.DIR() == (1<<4)); }
    WHEN("ATMEGA328P DDRC |= (1<<7)") { DDRC |= (1<<7); THEN("ATMEGA4809 PORTD.DIR = (1<<5)") REQUIRE(portd.DIR() == (1<<5)); }
  }
}

/*****************************************************************************/

SCENARIO("Testing Arduino Nano 4809 PORTC compatibility class", "NANO_Compat::PORTCClass") {
  PORT_t porta, portd;

  PORTCClass PORTC(&porta, &portd);

  REQUIRE(porta.OUT() == 0);
  REQUIRE(portd.OUT() == 0);

  GIVEN("Testing operator = ") {
    WHEN("ATMEGA328P PORTC = (1<<0)") { PORTC = (1<<0); THEN("ATMEGA4809 PORTD.OUT = (1<<3)") REQUIRE(portd.OUT() == (1<<3)); }
    WHEN("ATMEGA328P PORTC = (1<<1)") { PORTC = (1<<1); THEN("ATMEGA4809 PORTD.OUT = (1<<2)") REQUIRE(portd.OUT() == (1<<2)); }
    WHEN("ATMEGA328P PORTC = (1<<2)") { PORTC = (1<<2); THEN("ATMEGA4809 PORTD.OUT = (1<<1)") REQUIRE(portd.OUT() == (1<<1)); }
    WHEN("ATMEGA328P PORTC = (1<<3)") { PORTC = (1<<3); THEN("ATMEGA4809 PORTD.OUT = (1<<0)") REQUIRE(portd.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTC = (1<<4)") { PORTC = (1<<4); THEN("ATMEGA4809 PORTA.OUT = (1<<2)") REQUIRE(porta.OUT() == (1<<2)); }
    WHEN("ATMEGA328P PORTC = (1<<5)") { PORTC = (1<<5); THEN("ATMEGA4809 PORTA.OUT = (1<<3)") REQUIRE(porta.OUT() == (1<<3)); }
    WHEN("ATMEGA328P PORTC = (1<<6)") { PORTC = (1<<6); THEN("ATMEGA4809 PORTD.OUT = (1<<4)") REQUIRE(portd.OUT() == (1<<4)); }
    WHEN("ATMEGA328P PORTC = (1<<7)") { PORTC = (1<<7); THEN("ATMEGA4809 PORTD.OUT = (1<<5)") REQUIRE(portd.OUT() == (1<<5)); }
  }

  GIVEN("Testing operator &= ") {
    porta.OUT.set(0xFF);
    portd.OUT.set(0xFF);
    WHEN("ATMEGA328P PORTC &= ~(1<<0)") { PORTC &= ~(1<<0); THEN("ATMEGA4809 PORTD.OUT = 0xF7") REQUIRE(portd.OUT() == 0xF7); }
    WHEN("ATMEGA328P PORTC &= ~(1<<1)") { PORTC &= ~(1<<1); THEN("ATMEGA4809 PORTD.OUT = 0xFB") REQUIRE(portd.OUT() == 0xFB); }
    WHEN("ATMEGA328P PORTC &= ~(1<<2)") { PORTC &= ~(1<<2); THEN("ATMEGA4809 PORTD.OUT = 0xFD") REQUIRE(portd.OUT() == 0xFD); }
    WHEN("ATMEGA328P PORTC &= ~(1<<3)") { PORTC &= ~(1<<3); THEN("ATMEGA4809 PORTD.OUT = 0xFE") REQUIRE(portd.OUT() == 0xFE); }
    WHEN("ATMEGA328P PORTC &= ~(1<<4)") { PORTC &= ~(1<<4); THEN("ATMEGA4809 PORTA.OUT = 0xFB") REQUIRE(porta.OUT() == 0xFB); }
    WHEN("ATMEGA328P PORTC &= ~(1<<5)") { PORTC &= ~(1<<5); THEN("ATMEGA4809 PORTA.OUT = 0xF7") REQUIRE(porta.OUT() == 0xF7); }
    WHEN("ATMEGA328P PORTC &= ~(1<<6)") { PORTC &= ~(1<<6); THEN("ATMEGA4809 PORTD.OUT = 0xFE") REQUIRE(portd.OUT() == 0xEF); }
    WHEN("ATMEGA328P PORTC &= ~(1<<7)") { PORTC &= ~(1<<7); THEN("ATMEGA4809 PORTD.OUT = 0xDF") REQUIRE(portd.OUT() == 0xDF); }
  }

  GIVEN("Testing operator |= ") {
    WHEN("ATMEGA328P PORTC |= (1<<0)") { PORTC |= (1<<0); THEN("ATMEGA4809 PORTD.OUT = (1<<3)") REQUIRE(portd.OUT() == (1<<3)); }
    WHEN("ATMEGA328P PORTC |= (1<<1)") { PORTC |= (1<<1); THEN("ATMEGA4809 PORTD.OUT = (1<<2)") REQUIRE(portd.OUT() == (1<<2)); }
    WHEN("ATMEGA328P PORTC |= (1<<2)") { PORTC |= (1<<2); THEN("ATMEGA4809 PORTD.OUT = (1<<1)") REQUIRE(portd.OUT() == (1<<1)); }
    WHEN("ATMEGA328P PORTC |= (1<<3)") { PORTC |= (1<<3); THEN("ATMEGA4809 PORTD.OUT = (1<<0)") REQUIRE(portd.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTC |= (1<<4)") { PORTC |= (1<<4); THEN("ATMEGA4809 PORTA.OUT = (1<<2)") REQUIRE(porta.OUT() == (1<<2)); }
    WHEN("ATMEGA328P PORTC |= (1<<5)") { PORTC |= (1<<5); THEN("ATMEGA4809 PORTA.OUT = (1<<3)") REQUIRE(porta.OUT() == (1<<3)); }
    WHEN("ATMEGA328P PORTC |= (1<<6)") { PORTC |= (1<<6); THEN("ATMEGA4809 PORTD.OUT = (1<<4)") REQUIRE(portd.OUT() == (1<<4)); }
    WHEN("ATMEGA328P PORTC |= (1<<7)") { PORTC |= (1<<7); THEN("ATMEGA4809 PORTD.OUT = (1<<5)") REQUIRE(portd.OUT() == (1<<5)); }
  }
}

/*****************************************************************************/

SCENARIO("Testing Arduino Nano 4809 DDRD compatibility class", "NANO_Compat::DDRDClass") {
  PORT_t porta, portb, portc, portf;

  DDRDClass DDRD(&porta, &portb, &portc, &portf);

  REQUIRE(porta.DIR() == 0);
  REQUIRE(portb.DIR() == 0);
  REQUIRE(portc.DIR() == 0);
  REQUIRE(portf.DIR() == 0);

  GIVEN("Testing operator = ") {
    WHEN("ATMEGA328P DDRD = (1<<0)") { DDRD = (1<<0); THEN("ATMEGA4809 PORTC.DIR = (1<<4)") REQUIRE(portc.DIR() == (1<<4)); }
    WHEN("ATMEGA328P DDRD = (1<<1)") { DDRD = (1<<1); THEN("ATMEGA4809 PORTC.DIR = (1<<5)") REQUIRE(portc.DIR() == (1<<5)); }
    WHEN("ATMEGA328P DDRD = (1<<2)") { DDRD = (1<<2); THEN("ATMEGA4809 PORTA.DIR = (1<<0)") REQUIRE(porta.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRD = (1<<3)") { DDRD = (1<<3); THEN("ATMEGA4809 PORTF.DIR = (1<<5)") REQUIRE(portf.DIR() == (1<<5)); }
    WHEN("ATMEGA328P DDRD = (1<<4)") { DDRD = (1<<4); THEN("ATMEGA4809 PORTC.DIR = (1<<6)") REQUIRE(portc.DIR() == (1<<6)); }
    WHEN("ATMEGA328P DDRD = (1<<5)") { DDRD = (1<<5); THEN("ATMEGA4809 PORTB.DIR = (1<<2)") REQUIRE(portb.DIR() == (1<<2)); }
    WHEN("ATMEGA328P DDRD = (1<<6)") { DDRD = (1<<6); THEN("ATMEGA4809 PORTF.DIR = (1<<4)") REQUIRE(portf.DIR() == (1<<4)); }
    WHEN("ATMEGA328P DDRD = (1<<7)") { DDRD = (1<<7); THEN("ATMEGA4809 PORTA.DIR = (1<<1)") REQUIRE(porta.DIR() == (1<<1)); }
  }

  GIVEN("Testing operator &= ") {
    porta.DIR.set(0xFF);
    portb.DIR.set(0xFF);
    portc.DIR.set(0xFF);
    portf.DIR.set(0xFF);
    WHEN("ATMEGA328P DDRD &= ~(1<<0)") { DDRD &= ~(1<<0); THEN("ATMEGA4809 PORTC.DIR = 0xEF") REQUIRE(portc.DIR() == 0xEF); }
    WHEN("ATMEGA328P DDRD &= ~(1<<1)") { DDRD &= ~(1<<1); THEN("ATMEGA4809 PORTC.DIR = 0xDF") REQUIRE(portc.DIR() == 0xDF); }
    WHEN("ATMEGA328P DDRD &= ~(1<<2)") { DDRD &= ~(1<<2); THEN("ATMEGA4809 PORTA.DIR = 0xFE") REQUIRE(porta.DIR() == 0xFE); }
    WHEN("ATMEGA328P DDRD &= ~(1<<3)") { DDRD &= ~(1<<3); THEN("ATMEGA4809 PORTF.DIR = 0xDF") REQUIRE(portf.DIR() == 0xDF); }
    WHEN("ATMEGA328P DDRD &= ~(1<<4)") { DDRD &= ~(1<<4); THEN("ATMEGA4809 PORTC.DIR = 0xBF") REQUIRE(portc.DIR() == 0xBF); }
    WHEN("ATMEGA328P DDRD &= ~(1<<5)") { DDRD &= ~(1<<5); THEN("ATMEGA4809 PORTB.DIR = 0xFB") REQUIRE(portb.DIR() == 0xFB); }
    WHEN("ATMEGA328P DDRD &= ~(1<<6)") { DDRD &= ~(1<<6); THEN("ATMEGA4809 PORTF.DIR = 0xEF") REQUIRE(portf.DIR() == 0xEF); }
    WHEN("ATMEGA328P DDRD &= ~(1<<7)") { DDRD &= ~(1<<7); THEN("ATMEGA4809 PORTA.DIR = 0xFD") REQUIRE(porta.DIR() == 0xFD); }
  }

  GIVEN("Testing operator |= ") {
    WHEN("ATMEGA328P DDRD |= (1<<0)") { DDRD |= (1<<0); THEN("ATMEGA4809 PORTC.DIR = (1<<4)") REQUIRE(portc.DIR() == (1<<4)); }
    WHEN("ATMEGA328P DDRD |= (1<<1)") { DDRD |= (1<<1); THEN("ATMEGA4809 PORTC.DIR = (1<<5)") REQUIRE(portc.DIR() == (1<<5)); }
    WHEN("ATMEGA328P DDRD |= (1<<2)") { DDRD |= (1<<2); THEN("ATMEGA4809 PORTA.DIR = (1<<0)") REQUIRE(porta.DIR() == (1<<0)); }
    WHEN("ATMEGA328P DDRD |= (1<<3)") { DDRD |= (1<<3); THEN("ATMEGA4809 PORTF.DIR = (1<<5)") REQUIRE(portf.DIR() == (1<<5)); }
    WHEN("ATMEGA328P DDRD |= (1<<4)") { DDRD |= (1<<4); THEN("ATMEGA4809 PORTC.DIR = (1<<6)") REQUIRE(portc.DIR() == (1<<6)); }
    WHEN("ATMEGA328P DDRD |= (1<<5)") { DDRD |= (1<<5); THEN("ATMEGA4809 PORTB.DIR = (1<<2)") REQUIRE(portb.DIR() == (1<<2)); }
    WHEN("ATMEGA328P DDRD |= (1<<6)") { DDRD |= (1<<6); THEN("ATMEGA4809 PORTF.DIR = (1<<4)") REQUIRE(portf.DIR() == (1<<4)); }
    WHEN("ATMEGA328P DDRD |= (1<<7)") { DDRD |= (1<<7); THEN("ATMEGA4809 PORTA.DIR = (1<<1)") REQUIRE(porta.DIR() == (1<<1)); }
  }
}

/*****************************************************************************/

SCENARIO("Testing Arduino Nano 4809 PORTD compatibility class", "NANO_Compat::PORTDClass") {
  PORT_t porta, portb, portc, portf;

  PORTDClass PORTD(&porta, &portb, &portc, &portf);

  REQUIRE(porta.OUT() == 0);
  REQUIRE(portb.OUT() == 0);
  REQUIRE(portc.OUT() == 0);
  REQUIRE(portf.OUT() == 0);

  GIVEN("Testing operator = ") {
    WHEN("ATMEGA328P PORTD = (1<<0)") { PORTD = (1<<0); THEN("ATMEGA4809 PORTC.OUT = (1<<4)") REQUIRE(portc.OUT() == (1<<4)); }
    WHEN("ATMEGA328P PORTD = (1<<1)") { PORTD = (1<<1); THEN("ATMEGA4809 PORTC.OUT = (1<<5)") REQUIRE(portc.OUT() == (1<<5)); }
    WHEN("ATMEGA328P PORTD = (1<<2)") { PORTD = (1<<2); THEN("ATMEGA4809 PORTA.OUT = (1<<0)") REQUIRE(porta.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTD = (1<<3)") { PORTD = (1<<3); THEN("ATMEGA4809 PORTF.OUT = (1<<5)") REQUIRE(portf.OUT() == (1<<5)); }
    WHEN("ATMEGA328P PORTD = (1<<4)") { PORTD = (1<<4); THEN("ATMEGA4809 PORTC.OUT = (1<<6)") REQUIRE(portc.OUT() == (1<<6)); }
    WHEN("ATMEGA328P PORTD = (1<<5)") { PORTD = (1<<5); THEN("ATMEGA4809 PORTB.OUT = (1<<2)") REQUIRE(portb.OUT() == (1<<2)); }
    WHEN("ATMEGA328P PORTD = (1<<6)") { PORTD = (1<<6); THEN("ATMEGA4809 PORTF.OUT = (1<<4)") REQUIRE(portf.OUT() == (1<<4)); }
    WHEN("ATMEGA328P PORTD = (1<<7)") { PORTD = (1<<7); THEN("ATMEGA4809 PORTA.OUT = (1<<1)") REQUIRE(porta.OUT() == (1<<1)); }
  }

  GIVEN("Testing operator &= ") {
    porta.OUT.set(0xFF);
    portb.OUT.set(0xFF);
    portc.OUT.set(0xFF);
    portf.OUT.set(0xFF);
    WHEN("ATMEGA328P PORTD &= ~(1<<0)") { PORTD &= ~(1<<0); THEN("ATMEGA4809 PORTC.OUT = 0xEF") REQUIRE(portc.OUT() == 0xEF); }
    WHEN("ATMEGA328P PORTD &= ~(1<<1)") { PORTD &= ~(1<<1); THEN("ATMEGA4809 PORTC.OUT = 0xDF") REQUIRE(portc.OUT() == 0xDF); }
    WHEN("ATMEGA328P PORTD &= ~(1<<2)") { PORTD &= ~(1<<2); THEN("ATMEGA4809 PORTA.OUT = 0xFE") REQUIRE(porta.OUT() == 0xFE); }
    WHEN("ATMEGA328P PORTD &= ~(1<<3)") { PORTD &= ~(1<<3); THEN("ATMEGA4809 PORTF.OUT = 0xDF") REQUIRE(portf.OUT() == 0xDF); }
    WHEN("ATMEGA328P PORTD &= ~(1<<4)") { PORTD &= ~(1<<4); THEN("ATMEGA4809 PORTC.OUT = 0xBF") REQUIRE(portc.OUT() == 0xBF); }
    WHEN("ATMEGA328P PORTD &= ~(1<<5)") { PORTD &= ~(1<<5); THEN("ATMEGA4809 PORTB.OUT = 0xFB") REQUIRE(portb.OUT() == 0xFB); }
    WHEN("ATMEGA328P PORTD &= ~(1<<6)") { PORTD &= ~(1<<6); THEN("ATMEGA4809 PORTF.OUT = 0xEF") REQUIRE(portf.OUT() == 0xEF); }
    WHEN("ATMEGA328P PORTD &= ~(1<<7)") { PORTD &= ~(1<<7); THEN("ATMEGA4809 PORTA.OUT = 0xFD") REQUIRE(porta.OUT() == 0xFD); }
  }

  GIVEN("Testing operator |= ") {
    WHEN("ATMEGA328P PORTD |= (1<<0)") { PORTD |= (1<<0); THEN("ATMEGA4809 PORTC.OUT = (1<<4)") REQUIRE(portc.OUT() == (1<<4)); }
    WHEN("ATMEGA328P PORTD |= (1<<1)") { PORTD |= (1<<1); THEN("ATMEGA4809 PORTC.OUT = (1<<5)") REQUIRE(portc.OUT() == (1<<5)); }
    WHEN("ATMEGA328P PORTD |= (1<<2)") { PORTD |= (1<<2); THEN("ATMEGA4809 PORTA.OUT = (1<<0)") REQUIRE(porta.OUT() == (1<<0)); }
    WHEN("ATMEGA328P PORTD |= (1<<3)") { PORTD |= (1<<3); THEN("ATMEGA4809 PORTF.OUT = (1<<5)") REQUIRE(portf.OUT() == (1<<5)); }
    WHEN("ATMEGA328P PORTD |= (1<<4)") { PORTD |= (1<<4); THEN("ATMEGA4809 PORTC.OUT = (1<<6)") REQUIRE(portc.OUT() == (1<<6)); }
    WHEN("ATMEGA328P PORTD |= (1<<5)") { PORTD |= (1<<5); THEN("ATMEGA4809 PORTB.OUT = (1<<2)") REQUIRE(portb.OUT() == (1<<2)); }
    WHEN("ATMEGA328P PORTD |= (1<<6)") { PORTD |= (1<<6); THEN("ATMEGA4809 PORTF.OUT = (1<<4)") REQUIRE(portf.OUT() == (1<<4)); }
    WHEN("ATMEGA328P PORTD |= (1<<7)") { PORTD |= (1<<7); THEN("ATMEGA4809 PORTA.OUT = (1<<1)") REQUIRE(porta.OUT() == (1<<1)); }
  }
}