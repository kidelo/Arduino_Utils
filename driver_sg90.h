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

    @brief Driver for SG-90, see http://www.towerpro.com.tw/product/sg90-7/
*/

// **************************************************************************************

#ifndef KID_SG90_DRIVER_H_
#define KID_SG90_DRIVER_H_

#include <arduino.h>
#include <util_timer.h>
#include <util_math.h>
#include <TaskScheduler.h>
#include <Servo.h>

// **************************************************************************************

struct SG90Servo : public Servo, protected Task
{
  enum
  {

    // yellow
    PIN_SERVO                        = 9,

    // ------------------------------------------

    // power of the SERVO motor after idle, no current
    POWER_OFF_IDLE_MS                = 2000,

    // ------------------------------------------
    //
    // for my device ....

    // mechanical adjustment correction for 0° correction
    // unaligned head connection to server axis
    SERVO_MECH_90_DEG_ADJUST_DEGREE  = -5,

    SERVO_MECH_0_DEG_PULSE_US        = 510,
    SERVO_MECH_180_DEG_PULSE_US      = 2300,

    // standard pulse width for servo motors
    SERVO_TIME_PULSE_WIDTH_MS          = 20,

    // initial response delay, servo will start turn at least after 3x pulse with
    SERVO_DELAY_INIT_RESPONSE_MS       = 3 * SERVO_TIME_PULSE_WIDTH_MS,

    // rotation speed for the servo
    SERVO_ROTATION_SPEED_180_DEG_MS    = 350,

    // extra fine adjustment delay for position around 0°
    SERVO_DELAY_FINE_ADJ_MS_PER_10DEG  = 5,
  };

  // ctor
  SG90Servo( Scheduler & scheduler ) : Task(POWER_OFF_IDLE_MS, 1, 0, &scheduler, false )
  {

  } // end of MyServo

  // init the thing
  bool init()
  {
    // test procedure
    set_angle( 0,  true );

    return true;
  }

  // power on the
  void on()
  {
    if ( !attached() )
    {
      attach(PIN_SERVO, SERVO_MECH_0_DEG_PULSE_US, SERVO_MECH_180_DEG_PULSE_US );
    }

    // disable automatic shutdown
    disable();

  } // end of n()

  // power off, the servo will standby by without power consumption
  void off()
  {
    // free me
    detach();
  }

  // is powered on ?
  inline bool is_on() const
  {
    return attached();

  } // end of is_on()

  // returns current angle
  int8_t get_angle()
  {
    return m_angle;

  } // end of get_angle()

  // current angle, wait for stable point
  void set_angle( int8_t angle, bool wait = false )
  {
    // adjust ( proportional, small error in absolute position but now linearized )
    int16_t angle_new = angle + ( SERVO_MECH_90_DEG_ADJUST_DEGREE / ( angle + 1 ));

    // clip
    if ( angle_new > +90 ) angle_new = +90;
    if ( angle_new < -90 ) angle_new = -90;

    // get current angle
    const int8_t angle_old = get_angle();

    // already controlled ?
    const bool is_on = attached();

    // required setup
    if ( !is_on )
    {
      // switch on
      on();
    }

    // command new value ( here 0..180° )
    write( angle_new + 90 );

    // estimate delta if powered off the initial position is unknown -> full range
    const uint8_t angle_delta = is_on ? ( angle_old > angle_new ? angle_old - angle_new : angle_new - angle_old ) : 180;

    // *******************************************************

    // calculate needed time to reached target position

    // a.) static response delay
    const uint16_t delay_ms_a = SERVO_DELAY_INIT_RESPONSE_MS;

    // b.) delta degree ( drive way distance from original to target position.. )
    const uint16_t delay_ms_b = ( ( (uint32_t)angle_delta * SERVO_ROTATION_SPEED_180_DEG_MS ) / 180 ) + 1;

    // c.) more accuracy for degrees near 0°
    const uint16_t delay_ms_c = ( ( ( 90 - (uint16_t)abs(angle_new) ) * SERVO_DELAY_FINE_ADJ_MS_PER_10DEG ) / 10 ) ;

    // overall delay
    const uint16_t delay_ms = delay_ms_a + delay_ms_b + delay_ms_c;

    // *******************************************************

    // store it
    m_angle = angle_new;

    // synchronous wait ?
    if ( wait )
    {
      // spend time
      ::delay( delay_ms );

      // force service control off
      off();
    }
    else
    {
      // enable the thing
      enable();

      // switch power off after idle time
      setdelay( delay_ms );
    }

    m_angle = angle;

  } // end of set_angle()

protected:

  // switch off after idle
  virtual void run()
  {
    // force off
    off();

  } // end of run()

  // last set value in °
  int8_t m_angle = 0;
};


#endif // KID_SG90_DRIVER_H_

