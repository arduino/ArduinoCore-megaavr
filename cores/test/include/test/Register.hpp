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

#ifndef REGISTER_HPP_
#define REGISTER_HPP_

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/

#include <stdint.h>

#include <vector>
#include <memory>
#include <iostream>

#include <test/RegisterObserver.hpp>

/*****************************************************************************
 * CLASS DECLARATION
 *****************************************************************************/

template <typename T>
class Register
{
public:
  Register();

  T    operator () () const;  
  bool isBitSet(uint8_t const bit_pos) const;
  bool isBitClr(uint8_t const bit_pos) const;

  /* =, |=, &=, 'setBit', 'clrBit' all notify registered observers */
  void operator  = (uint8_t const val);
  void operator |= (uint8_t const val);
  void operator &= (uint8_t const val);
  void setBit      (uint8_t const bit_pos);
  void clrBit      (uint8_t const bit_pos);
  
  /* 'set' does not notify registered observers, unless argument 'notify' is set to true (default = false) */
  void set(uint8_t const val, bool const notify = false);

  void registerObserver(RegisterObserver<T> * observer);
  
private:
  
  typedef std::vector<RegisterObserver<T> *> RegisterObserverVector;

  T _reg_val;
  RegisterObserverVector _observer_vect;
  
  void notifyObserver();
};

template <typename T>
std::ostream & operator << (std::ostream &out, Register<T> const & reg);

/*****************************************************************************
 * TEMPLATE IMPLEMENTATION
 *****************************************************************************/

#include "Register.ipp"

/*****************************************************************************
 * TYPEDEF
 *****************************************************************************/

typedef Register<uint8_t>  Register8;
typedef Register<uint16_t> Register16;
typedef Register<uint32_t> Register32;
typedef Register<uint64_t> Register64;

#endif /* REGISTER_HPP_ */
