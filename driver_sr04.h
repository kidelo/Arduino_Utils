/*
    Copyright (C) 2018 Kidelo <kidelo@yahoo.com>

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

    @brief Driver for HC-SR04, see http://www.electroschematics.com/8902/hc-sr04-datasheet/
*/

// **************************************************************************************

#ifndef KID_HCSR04_DRIVER_H_
#define KID_HCSR04_DRIVER_H_

#include <arduino.h>
#include <util_timer.h>
#include <util_math.h>
#include <TaskScheduler.h>

// ********************************************************************************

// ultrasonic measurement for HC-SR04
struct UltraHCSR04 : protected Task
{
  enum
   {
     // ultrasonic echo feedback pin IRQ cabable (white)
     PIN_ECHO           = 3,

     // ultrasonic trigger pin (brown)
     PIN_TRIG           = 12,

     // convert round trip time from [us] to [cm], valid for 20°C
     DISTANCE_US_TO_CM  = 58,

     // -------------------------------------

     // maximum samples
     MAX_HIST_SAMPLES   = 16,

     // -------------------------------------

     // maximum timeout for measurement, process was retriggered
     // appr. 36 ms stated in data sheet
     TMO_ECHO_MAX_MS    = 50,

     // guard timeout between two measurements
     TMO_ECHO_GUARD_MS  = 20,

     // -------------------------------------

     // min / max for distance measurements
     MIN_DISTANCE_CM    = 2,
     MAX_DISTANCE_CM    = 400,

     // -------------------------------------

     // guard time out for stable distance measurement detection
     TMO_STABLE_GUARD   = TMO_ECHO_MAX_MS * MAX_HIST_SAMPLES,
   };

  // statistics
  struct t_stat
  {
    uint16_t min;       // min value
    uint16_t max;       // maximum value

    uint16_t mean_all;  // of all samples
    uint16_t mean_wo;   // of all samples without min / min max samples
  };

  // ctor, registers at scheduler
  UltraHCSR04( Scheduler & sched ) : Task( 1000, TASK_FOREVER, 0, &sched, false )
  {
    // clean me up
    clear();

    // set pin to interrupt
    pinMode( PIN_ECHO, INPUT  );
    pinMode( PIN_TRIG, OUTPUT );

    // force to low
    digitalWrite(PIN_TRIG,LOW);

    // normal
    time_delta_ms = TMO_ECHO_MAX_MS;

    // store singleton instance
    s_This = this;
  }

  // dtor
  virtual ~UltraHCSR04() {}

  // measurement is finished
  inline bool filled() const
  {
    return dist_full;

  } // end of filled()

  // clear old measurements
  inline void clear()
  {
    // clean up
    dist_last = dist_full = 0;

    // reset
    timer_last = TMO_STABLE_GUARD;

  } // end of clear()

  // trigger measurement once and than stop
  inline void measure()
  {
    set_period( 0, 0 );

  } // end of once()

  // set trigger period
  inline void set_period( uint16_t period, uint16_t start_delay = 0 )
  {
    // internal delta between end and start of next measurement
    time_delta_ms = period;

    // enable
    enableIfNot();

    // no start delay ...
    if ( !start_delay )
    {
      // force it
      trigger();
    }
    else
    {
      // setup initial wait
      restartDelayed( start_delay);
    }

  } // end of set_perdiod()

  // returns true when sensor got stable data in regular time interval
  bool is_stable() const
  {
    return timer_last < TMO_STABLE_GUARD;

  } // is_stable()

  // get last measurement
  uint16_t get_last() const
  {
    return dist_hist[ dist_last ];
  }

  // returns statistic of last measurements
  t_stat get_stat() const
  {
    t_stat tmp;

    uint32_t mean = 0;
    uint16_t  max = 0;
    uint16_t  min = (uint16_t)(-1);

    // get all
    for ( auto v : dist_hist )
    {
      // compare
      if ( v < min ) min = v;
      if ( v > max ) max = v;

      // accumulate
      mean += v;
    }

    // store
    tmp.min = min;
    tmp.max = max;

    // normalize
    tmp.mean_all = mean / MAX_HIST_SAMPLES;

    // remove min / max samples ( one times )
    mean -= min;
    mean -= max;

    // normalize
    tmp.mean_wo = mean / (MAX_HIST_SAMPLES-2);

    return tmp;

  } // end of get()

protected:

   // trigger measurement
  void trigger()
  {
    // reset, 2x event
    m_time = 0; m_cnt = 2;

    // attach callback to me ( static function ... template generates a individual one
    attachInterrupt( digitalPinToInterrupt( PIN_ECHO ), isr_callback, CHANGE );

    // trigger new measurement
    digitalWrite( PIN_TRIG, HIGH );

    // see datasheet
    delayMicroseconds( 10 );

    // start with high -> low transition
    digitalWrite( PIN_TRIG, LOW );

    // setup guard check
    restartDelayed( TMO_ECHO_MAX_MS );

  } // end of trigger()

  // callback from ISR
  static void isr_callback()
  {
    // cache ( avoid multiple volatile access )
    uint8_t cnt = m_cnt;

    // fast out, ignore interrupt
    if ( !cnt ) return;

    // detected pulse
    cnt--;

    // get current time now
    const uint32_t now = micros();

    // new measurement
    m_time = cnt > 0 ? now : now - m_time;

    // store back
    m_cnt = cnt;

    // inform
    if ( 0 == cnt )
    {
      // detach from ISR
      detachInterrupt( digitalPinToInterrupt( PIN_ECHO ) );

      // inform
      s_This->on_event();
    }
  }

  // call back when event has triggered ( is here in ISR )
  void on_event()
  {
    forceNextIteration();

  } // end of on_event()

  // default callback method
  virtual void run()
  {
    // has no new new sample of ultrasonic sound distance time
    // between our sensor an objects near and far
    if ( !m_cnt && !m_time )
    {
      // start next round of measurement
      trigger();

      // fast out
      return;
    }

    // distance in [cm] for time in [us] @ 20°C at normal air
    const uint16_t dist_cm = m_time/58;

    // buffer end reached ?
    bool end = false;

    // ignore out of range data -> sensor error ... to near, to fast
    if ( ( dist_cm >= MIN_DISTANCE_CM && dist_cm <= MAX_DISTANCE_CM ) )
    {
      // to next
      dist_last++;

      // buffer end reached ?
      const bool end = MAX_HIST_SAMPLES == dist_last;

      // overflow
      if ( end )
      {
        // jump to start
        dist_last = 0;

        // store required time to measure these samples
        timer_last = timer.reset();

        // storage is full
        dist_full = true;
      }

      // store it
      dist_hist[ dist_last ] = dist_cm;
    }

    // reset
    m_time = 0;

    // period or single shot ....
    if ( time_delta_ms || !end )
    {
      // enabled the thing again
      enable();

      // at least guard delay ( echo compensation )
      restartDelayed( max( (uint16_t)TMO_ECHO_GUARD_MS, time_delta_ms ) );
    }
    else
    {
      // disable scheduler
      disable();
    }

  } // end of on_event()

  // number of micros since of last action
  static volatile uint32_t m_time;

  // count down
  static volatile uint8_t m_cnt;

  // call be back
  static UltraHCSR04 * s_This;

  // last update
  timedelta_ms_16 timer;

  // 0 = one shot ( fill buffer ), min = TMO_ECHO_GUARD_MS, max ....
  uint16_t time_delta_ms;

  // last time required to fill sample buffer
  uint16_t timer_last;

  // distance history
  uint16_t dist_hist[MAX_HIST_SAMPLES];

  // last written entry
  uint8_t dist_last;

  // number of samples inside
  bool dist_full;

}; // end of class Ultra

// **************************************************************************************

// number of micros since of last action
volatile uint32_t UltraHCSR04::m_time = 0;
volatile uint8_t UltraHCSR04::m_cnt = 0;
UltraHCSR04 * UltraHCSR04::s_This  = 0;

// **************************************************************************************

#endif // KID_HCSR04_DRIVER_H_

