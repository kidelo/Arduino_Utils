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
    timedelta_us_16 timeout1( 1234 );
    
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
    return micros();

  } // end of micros_f()

  // internal function to get millis(s) base on current type
  static inline T millis_f()
  {
    return millis();

  } // end of millis_f()
  
}; // end of time_clock

// **************************************************************************************

// template for simple time measurements
template< typename T, T (*TFUNC)() > struct time_delta
{
  // calculate maximum storage of type 2^N (bits)
  static constexpr T MAX_VAL = ( 2 ^ ( sizeof(T) * 8 ) ) - 1;

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
    tmo   = rhs.tmo;
  }

  // reset timeout measurement with current timeout and
  // return elapsed time at point of call
  inline T reset()
  {
    // get raw values
    const T now     = time();
    const T elapsed = _elapsed( now );

    // set start time
    start = now;

    return elapsed;
  }

  // restart timeout measurement with new timeout
  inline void restart( T timeout )
  {
    start = time(); tmo = timeout;
  }

  // returns the current timeout
  inline T timeout() const
  {
    return timeout;
  }
  
  // number of time ticks until the timeout
  inline T pending() const
  {
	// get current state
    const T delta = elapsed();
    const T tmo   = timeout();

    // determine
    return delta < tmo ? ( tmo - delta ) : 0;

  } // end of pending()

  // has pending time ?
  inline bool is_pending() const
  {
     return 0 != pending();
  
  } // end of has_pending()

  // returns the elapsed time in time units
  inline T elapsed() const
  {
    // get time sample and forward
    return _elapsed( time() );

  } // end of elapsed()

  // timer is elapsed ?
  inline bool is_elapsed() const
  {
     return 0 == pending();
  
  } // end of has_pending()

  // calculate elapsed time based on a time value
  inline T _elapsed( T time ) const
  {
    // normal increase or overflow ?
    return ( time > start ) ? ( time - start ) : ( MAX_VAL - start + time );

  } // end of elapsed()

  // get time in ticks of time base
  inline static T time()
  {
    return TFUNC();
  }

  // start time
  T start;

  // timeout
  T tmo;

}; // end of time_delta

// **************************************************************************************

// some shortcuts
typedef time_delta< uint8_t,  time_clock<uint8_t>:: micros_f > timedelta_us_8;
typedef time_delta< uint16_t, time_clock<uint16_t>::micros_f > timedelta_us_16;
typedef time_delta< uint32_t, time_clock<uint32_t>::micros_f > timedelta_us_32;
typedef time_delta< uint8_t,  time_clock<uint8_t>:: millis_f > timedelta_ms_8;
typedef time_delta< uint16_t, time_clock<uint16_t>::millis_f > timedelta_ms_16;
typedef time_delta< uint32_t, time_clock<uint32_t>::millis_f > timedelta_ms_32;

// **************************************************************************************

#endif // KID_UTIL_TIMER_H_

