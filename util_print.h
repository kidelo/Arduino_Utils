/*
    Copyright (C) 2017-2018 Kidelo <kidelo@yahoo.com>

    This file is part of https://github.com/kidelo/KRobot_Utils

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    @brief This file provides a class for connecting multiple sink to a central print class

    *************************************************************************************

    EXAMPLE:
    -------
    #include <util_print.h>

    MultiPrint output;

    // add normals serial output
    output.add( &Serial    );

    // add other kind of "Print"
    output.add( &lcd       );
    output.add( &wifi      );
    output.add( &bluetooth );

    // print it to all sinks
    output.printf('ll');

    *************************************************************************************
*/

#ifndef KID_UTIL_PRINT_H_
#define KID_UTIL_PRINT_H_

#include <Print.h>

// **************************************************************************************

// forward print calls to multiple consumer
struct MultiPrint : public Print
{
  enum { MAX_FORWARD = 3 };

  // ctor
  MultiPrint()
  {
    // store at next free entry
    for( auto & p : m_fwd ) { p = 0; }
  }

  // add
  void add( Print * p )
  {
    // store at next free entry
    for( auto & to : m_fwd )
    {
      if ( !to )
      {
        to = p; break;
      }
    }

  } // end of add()

  // remove from output
  void remove( Print * p )
  {
    // store at next free entry
    for( auto & to : m_fwd )
    {
      if ( to == p ) to = 0;
    }

  } // end of remove()

  // normal write, forward to all consumers
  virtual size_t write(uint8_t c)
  {
    // for all
    for( auto fwd : m_fwd )
    {
      if ( fwd ) fwd->write( c );
    }

    // always consumed
    return 1;
 }

  // forwarder
  Print * m_fwd[MAX_FORWARD];
};

#endif // KID_UTIL_PRINT_H_

