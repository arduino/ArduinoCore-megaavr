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

/*****************************************************************************
 * INCLUDE
 *****************************************************************************/

#include <iomanip>

/*****************************************************************************
 * CTOR/DTOR
 *****************************************************************************/

template <typename T>
Register<T>::Register() 
: _reg_val (0)/*, 
  _observer(0)*/ { }

/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/

template <typename T>
T Register<T>::operator () () const { 
  return _reg_val;
}

template <typename T>
bool Register<T>::isBitSet(uint8_t const bit_pos) const { 
  T const bitmask = static_cast<T>(1<<bit_pos);
  return ((_reg_val & bitmask) == bitmask);
}

template <typename T>
bool Register<T>::isBitClr(uint8_t const bit_pos) const { 
  T const bitmask = static_cast<T>(1<<bit_pos);
  return ((_reg_val & bitmask) == 0);
}

template <typename T>
void Register<T>::operator = (uint8_t const val) {
   set(val, true);
}

template <typename T>
void Register<T>::operator |= (uint8_t const val) {
  set(_reg_val | val, true);
}

template <typename T>
void Register<T>::operator &= (uint8_t const val) { 
  set(_reg_val & val, true);
}
  
template <typename T>
void Register<T>::setBit(uint8_t const bit_pos) { 
  set(_reg_val | static_cast<T>(1<<bit_pos), true);
}

template <typename T>
void Register<T>::clrBit(uint8_t const bit_pos) { 
    set(_reg_val & static_cast<T>(~(1<<bit_pos)), true);
}

template <typename T>
void Register<T>::registerObserver(RegisterObserver<T> * observer) { 
  _observer_vect.push_back(observer);
}

template <typename T>
void Register<T>::set(uint8_t const val, bool const notify) { 
  _reg_val = val; 
  if(notify) notifyObserver();
}

/*****************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 *****************************************************************************/

template <typename T>
void Register<T>::notifyObserver() {
  for (RegisterObserver<T> * observer : _observer_vect) {
    observer->onChange(*this);
  }
};

/*****************************************************************************
 * PUBLIC FUNCTIONS
 *****************************************************************************/

template <typename T>
std::ostream & operator << (std::ostream & os, Register<T> const & reg) {
  os << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(reg()) << std::dec;
  return os;
}
