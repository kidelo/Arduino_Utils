/*
    Copyright (C) 2017 Kidelo <kidelo@yahoo.com>

    This file is part of util_kid library

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

    @brief This file provides a class for time measurement and timeout handling

    *************************************************************************************

    EXAMPLE:
    -------

    #include <util_timer.h>

    // a.) simple time measurement ------------------------------------------
    timedelta_us_16 measure;

    const uint16_t t1 = measure.elapsed();
    const uint16_t t2 = measure.elapsed();

    // b.) simple timeout ---------------------------------------------------
    timedelta_us_16 timeout1( 1234 /* us *);

    // do until elapsed
    while( timeout1.is_pending() )
    {
        // do anything ...
        delay( ... )
    }

    *************************************************************************************
*/
#ifndef KID_UTIL_TIMER_H_
#define KID_UTIL_TIMER_H_

#include <arduino.h>

// **************************************************************************************

// helper class
template<typename T> struct time_clock
{
  // internal function to get micro(s) base on current type
  static inline T micros_f()
  {
    return (T)micros();

  } // end of micros_()

  // internal function to get millis(s) base on current type
  static inline T millis_f()
  {
    return (T)micros();

  } // end of micros_()

}; // end of time_clock

// **************************************************************************************

// template for simple time measurements
template< typename T = uint16_t, T (*TFUNC)() = time_clock<T>::micros_f > struct time_delta
{
  // calculate maximum storage of type 2^N (bits)
  static constexpr T MAX_VAL = 2 ^ ( sizeof(T) * 8 );

  // ctor
  time_delta( T timeout = 0 )
  {
    restart( timeout );
  }

  // copy ctor
  time_delta( const time_delta & org )
  {
    (*this) = org;
  }

  // dtor
  inline ~time_delta() {}

  // simple assign operator
  inline void operator = ( const time_delta & rhs )
  {
    start = rhs.start;
    end   = rhs.end;
  }

  // reset timeout measurement with current timeout
  inline void reset()
  {
    restart( end - start );
  }

  // restart timeout measurement with new timeout
  inline void restart( T timeout )
  {
    start = TFUNC(); end = start + timeout;
  }

  // returns the current timeout
  inline T timeout()
  {
    return end - start;
  }

  // number of time ticks until the timeout
  inline T pending() const
  {
	// get current state
    auto const delta = elapsed(), const tmo = timeout();

    // determine
    return delta < tmo ( tmo - delta ) : 0;

  } // end of pending()

  // has pending time ?
  inline bool is_pending() const
  {
     return 0 != pending();

  } // end of has_pending()

  inline T elapsed() const
  {
    // get fresh sample
    const T now = TFUNC();

    // normal increase or overflow ?
    return ( now > start ) ? ( now - start ) : ( MAX_VAL - start + now );

  } // end of elapsed()

  // timer is elapsed ?
  inline bool is_elapsed() const
  {
     return 0 == pending();

  } // end of has_pending()

  // start time
  T start;

  // end time
  T end;
};

// **************************************************************************************

// some shortcuts
typedef time_delta< uint8_t,  time_clock<uint8_t>:: micros_f > timedelta_us_8;  // +0  reference
typedef time_delta< uint16_t, time_clock<uint16_t>::micros_f > timedelta_us_16; // +26 opcodes
typedef time_delta< uint32_t, time_clock<uint32_t>::micros_f > timedelta_us_32; // +86 opcodes
typedef time_delta< uint8_t,  time_clock<uint8_t>:: millis_f > timedelta_ms_8;
typedef time_delta< uint16_t, time_clock<uint16_t>::millis_f > timedelta_ms_16;
typedef time_delta< uint32_t, time_clock<uint32_t>::millis_f > timedelta_ms_32;

// **************************************************************************************

#endif // KID_UTIL_TIMER_H_

