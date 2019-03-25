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

#ifndef PORT_MEGAAVR_PORT_T_H_
#define PORT_MEGAAVR_PORT_T_H_

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/

#include <iostream>

#include <test/Register.hpp>
#include <test/RegisterObserver.hpp>

/*****************************************************************************
 * CLASS DECLARATION
 *****************************************************************************/

class RegisterDIRSETObserver : public Register8Observer
{
public:
  RegisterDIRSETObserver(Register8 & DIR) : _DIR(DIR) { }
  virtual void onChange(Register8 & DIRSET) override { 
    for(uint8_t b = 0; b < 8; b++) if(DIRSET.isBitSet(b)) _DIR.setBit(b);
    DIRSET.set(0);
  }
private:
  Register8 & _DIR;
};

class RegisterDIRCLRObserver : public Register8Observer
{
public:
  RegisterDIRCLRObserver(Register8 & DIR) : _DIR(DIR) { }
  virtual void onChange(Register8 & DIRCLR) override {
    for(uint8_t b = 0; b < 8; b++) if(DIRCLR.isBitSet(b)) _DIR.clrBit(b);
    DIRCLR.set(0);
  }
private:
  Register8 & _DIR;
};

class RegisterOUTSETObserver : public Register8Observer
{
public:
  RegisterOUTSETObserver(Register8 & OUT) : _OUT(OUT) { }
  virtual void onChange(Register8 & OUTSET) override { 
    for(uint8_t b = 0; b < 8; b++) if(OUTSET.isBitSet(b)) _OUT.setBit(b);
    OUTSET.set(0);
  }
private:
  Register8 & _OUT;
};

class RegisterOUTCLRObserver : public Register8Observer
{
public:
  RegisterOUTCLRObserver(Register8 & OUT) : _OUT(OUT) { }
  virtual void onChange(Register8 & OUTCLR) override {
    for(uint8_t b = 0; b < 8; b++) if(OUTCLR.isBitSet(b)) _OUT.clrBit(b);
    OUTCLR.set(0);
  }
private:
  Register8 & _OUT;
};

class PORT_t
{
public:

  PORT_t();

  Register8 DIR;     /* Data Direction */
  Register8 DIRSET;  /* Data Direction Set */
  Register8 DIRCLR;  /* Data Direction Clear */
  Register8 OUT;     /* Output Value */
  Register8 OUTSET;  /* Output Value Set */
  Register8 OUTCLR;  /* Output Value Clear */
  
private:

  RegisterDIRSETObserver _DIRSET_observer;
  RegisterDIRCLRObserver _DIRCLR_observer;
  RegisterDIRSETObserver _OUTSET_observer;
  RegisterDIRCLRObserver _OUTCLR_observer;

};

std::ostream & operator << (std::ostream & os, PORT_t const & port);

#endif /* PORT_MEGAAVR_PORT_T_H_ */