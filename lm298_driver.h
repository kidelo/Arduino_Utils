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

    @brief This file provides functions to control a car using 2x DC motors
           via a LM298 driver (http://www.st.com/en/motor-drivers/l298.html)
           each channel responsible for all wheels on side of a car
*/

// **************************************************************************************

#ifndef KID_LM298_DRIVER_H_
#define KID_LM298_DRIVER_H_

#include <arduino.h>
#include <util_timer.h>
#include <util_math.h>

// **************************************************************************************

// motor driver
struct LM298
{
  enum Direction
  {
    DRIVE_STOP      = 0,

    DRIVE_FORWARD   = '^',
    DRIVE_BACKWARD  = '_',
    DRIVE_LEFT      = '<',
    DRIVE_RIGHT     = '>',
  };

  // *********************************************************************************

  // defines control pins to LM298N
  enum
  {
    PIN_LEFT_DIR_1   = 10,   // direction control
    PIN_LEFT_DIR_2   = 4,    // direction control
    PIN_LEFT_PWM     = 6,    // PWM pin -> motor speed

    PIN_RIGHT_DIR_1  = 7,    // direction control
    PIN_RIGHT_DIR_2  = 8,    // direction control
    PIN_RIGHT_PWM    = 5,    // PWM pin -> motor speed
  };

  // EMPERICALLY VALUES, ADJUSTMENT NEEDED !!!!
  // depends on CAR SETUP, MOTORS TYPE, VOLTAGE VALUE AND UNDERGROUND .. and more !!!
  // current values are based on experiments by using this platform :
  //
  // http://osoyoo.com/2017/08/06/osoyoo-robot-car-diy-introduction/
  enum
  {
    // initial startup power for motor normal forward / backward
    POWER_GO_FWD_VAL       = 200,
    POWER_GO_FWD_MS        = 1,

    // initial startup power for motor normal left / right turn
    POWER_GO_TURN_VAL      = 255,
    POWER_GO_TURN_MS       = 5,

    // time to reach full speed ?
    TIME_FULL_SPEED_MS     = 500,

    // reverse speed for active brake
    BRAKE_SPEED_DIVIDER    = 2,

    // time constant -> original speed to time
    BRAKE_TIME_CONSTANT    = 2
  };

  // *********************************************************************************

  // drive forward with initial value
  inline void forward( uint8_t val )
  {
    start( DRIVE_FORWARD, val );
  }

  // drive backward
  inline void backward( uint8_t val )
  {
    start( DRIVE_BACKWARD, val );
  }

  // drive backward
  inline void turn_left( uint8_t val )
  {
    start( DRIVE_LEFT, val );
  }

  // turn right
  inline void turn_right( uint8_t val )
  {
    start( DRIVE_RIGHT, val );
  }

  // start drive in selected direction
  void start( Direction dir, uint8_t val )
  {
    // reset
    m_tmo.clear();

    // setup direction
    set_direction( dir );

    // on stop ?
    if ( !m_left || !m_right )
    {
      // direction change
      const bool lr = ( DRIVE_LEFT == dir ) || ( DRIVE_RIGHT == dir );

      // more motor power for a short time, fight against friction .. and more
      set_speed( lr ? POWER_GO_TURN_VAL: POWER_GO_FWD_VAL );
      delay    ( lr ? POWER_GO_TURN_MS : POWER_GO_FWD_MS  );
    }

    // command normal speed
    set_speed( val );

  } // end of start()

  // force brake and then stop of device
  void brake()
  {
    // drive forward or backward
    const bool drive = ( DRIVE_FORWARD == m_dir ) || ( DRIVE_BACKWARD == m_dir );

    // full speed reached  -> BREAK WITH MOTOR in REVERSE direction for a short time !
    if( drive && m_tmo.elapsed() > TIME_FULL_SPEED_MS )
    {
      // get opposite direction
      const auto opposite = ( DRIVE_FORWARD == m_dir ) ? DRIVE_BACKWARD : DRIVE_FORWARD;

      // get reverse speed ( determined empirically )
      const auto speed = get_speed() / 2;

      // start reverse direction
      start( opposite, speed );

      // wait ( determined empirically )
      delay( speed / 2 );
    }

    // now real stopping
    stop();

  } // end of brake()

  // force soft
  void stop()
  {
    // set maximum speed
    set_speed(255);

    // reset direction -> FAST MOTOR STOP -> ( HAND BRAKE )
    // see data sheet of LM298N, , page 6
    digitalWrite( PIN_LEFT_DIR_1,  LOW );
    digitalWrite( PIN_LEFT_DIR_2,  LOW );
    digitalWrite( PIN_RIGHT_DIR_1, LOW );
    digitalWrite( PIN_RIGHT_DIR_2, LOW );

    // hold time for transient motion
    delay(1);

    // force motor control off
    analogWrite( PIN_LEFT_PWM,  0 );
    analogWrite( PIN_RIGHT_PWM, 0 );

    // no speed
    m_left = m_right = 0; m_dir = DRIVE_STOP;

  } // end of stop()

  // change speed, speed = 0 -> stop / brake
  inline void set_speed( uint8_t val )
  {
    // speed
    if ( 0 == val )
    {
      stop();
    }
    else
    {
      speed_left ( val );
      speed_right( val );
    }

  } // end of speed()

  // get speed
  inline uint8_t get_speed() const
  {
    return m_left/2 + m_right/2;

  } // end of get_speed()

  // increase / decrease speed by value
  void speed_change( int8_t delta )
  {
    // fast out
    if ( DRIVE_STOP == m_dir ) return;

    // shrink to max
    const uint8_t val_l = (uint8_t) clip_val<int16_t>( ((int16_t)m_left)  + delta, 0, 255 );
    const uint8_t val_r = (uint8_t) clip_val<int16_t>( ((int16_t)m_right) + delta, 0, 255 );

    // one motor goes to stop -> brake
    if ( !val_l || !val_r )
    {
      brake();
    }
    else
    {
      // command new
      speed_left ( val_l );
      speed_right( val_r );
    }

  } // end of speed_change()

  // regulate speed for left side
  void speed_left( int16_t val )
  {
    // force write
    analogWrite( PIN_LEFT_PWM, val );

    // store
    m_left = val;

  } // end of speed_left()

  // regulate speed for right side
  void speed_right( uint8_t val )
  {
    // force write
    analogWrite( PIN_RIGHT_PWM, val );

    // store
    m_right = val;

  } // end of speed_right()

  // setup motor direction
  void set_direction( Direction dir )
  {
    // default is forward direction
    bool left_fwd  = true;
    bool right_fwd = true;

    switch( dir )
    {
      case DRIVE_BACKWARD:  left_fwd = false; right_fwd = false; break;
      case DRIVE_RIGHT:     left_fwd = false; right_fwd = true;  break;
      case DRIVE_LEFT:      left_fwd = true;  right_fwd = false; break;
      case DRIVE_FORWARD:
      default:
        break;
    }

    // ---------------
    // IN1/3  IN2/4 <- LM298 http://www.st.com/en/motor-drivers/l298.html
    // ---------------
    // HIGH , LOW   -> FORWARD
    // LOW  , HIGH  -> BACKWARD
    // HIGH , HIGH  -> FAST MOTOR STOP
    // LOW  , LOW   -> FAST MOTOR STOP
    digitalWrite( PIN_LEFT_DIR_1,   left_fwd  );
    digitalWrite( PIN_LEFT_DIR_2,  !left_fwd  );
    digitalWrite( PIN_RIGHT_DIR_1,  right_fwd );
    digitalWrite( PIN_RIGHT_DIR_2, !right_fwd );

    // remember last direction
    m_dir = dir;

  } // end of direction()

  // returns current direction
  inline Direction get_direction() const
  {
    return m_dir;

  } // end of get_direction()

  // last drive values
  uint8_t m_left  = 0;
  uint8_t m_right = 0;

  // last direction
  Direction m_dir = DRIVE_STOP;

  // timeout measurement
  timedelta_ms_16 m_tmo;
};

#endif // KID_LM298_DRIVER_H_

