/*
    Copyright (C) 2017 Kidelo <kidelo@yahoo.com>

    This file is part of arduino_util_kid library

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

    @brief This file provides some helper functions
*/

#ifndef KID_UTIL_MATH_H_
#define KID_UTIL_MATH_H_

#include <stdint.h>

// **************************************************************************************

// clip value to min / max range
template<typename T> const T & clip_val( const T & in, const T & min, const T & max )
{
  if ( in < min )
  {
    return min;
  }
  else if ( in > max )
  {
    return max;
  }

  return in;

} // end of clip_val()

#endif // KID_UTIL_MATH_H_

