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

#include <test/port/megaavr/PORT_t.h>

PORT_t::PORT_t()
: _DIRSET_observer(DIR), 
  _DIRCLR_observer(DIR),
  _OUTSET_observer(OUT),
  _OUTCLR_observer(OUT) {
  DIRSET.registerObserver(&_DIRSET_observer);
  DIRCLR.registerObserver(&_DIRCLR_observer);
  OUTSET.registerObserver(&_OUTSET_observer);
  OUTCLR.registerObserver(&_OUTCLR_observer);
}

/*****************************************************************************
 * PUBLIC FUNCTIONS
 *****************************************************************************/

std::ostream & operator << (std::ostream & os, PORT_t const & port) {
  os << "\tDIR    = " << port.DIR    << std::endl
     << "\tDIRSET = " << port.DIRSET << std::endl
     << "\tDIRCLR = " << port.DIRCLR << std::endl
     << "\tOUT    = " << port.OUT    << std::endl
     << "\tOUTSET = " << port.OUTSET << std::endl
     << "\tOUTCLR = " << port.OUTCLR;
  return os;
}
