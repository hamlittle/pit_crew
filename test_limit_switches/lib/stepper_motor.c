/** \file stepper_motor.c
 *
 * \brief Enables interfacing with the GECKO G213V stepper motor driver.
 *
 * \author Hamilton Little
 *         hamilton.little@gmail.com
 *         Cal Poly ME UGRD class of 2014
 *
 * \copyright
 *       Copyright 2013 Hamilton Little
 *
 * \license \verbatim
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0\n
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. \endverbatim   */

/* Include Directives *********************************************************/

#include "stepper_motor.h"
#include <stdio.h>

/* Typedefs, Structs, and Enums ***********************************************/

/* Global Variables ***********************************************************/

/** \brief The first motor this library supports */
SM_t *motor0;

/** \brief The second motor this library supports */
SM_t *motor1;

/* Internal Function Prototypes ***********************************************/

static void SM_speed_ramp_init(SM_SRD_t *speed_ramp);
static void SM_timer_init(SM_t *motor, SM_timer_t timer);
static void SM_timer_OVF_handler(SM_t *motor);
static void SM_timer_start(SM_timer_t timer);
static void SM_timer_stop(SM_timer_t timer);
static void SM_timer_set_period(SM_timer_t, uint16_t period);
static void SM_step(SM_t *driver);
static unsigned long sqrt_Taylor(unsigned long x);

/* Function Definitions *******************************************************/

/** \brief Initializes this library.
 *
 * Sets up the necessary pins to interface with the Gecko G213V stepper motor
 * driver. Each driver requires 3 pins to interface with it:
 *    - DISABLE (Driven high to disable)
 *    - DIRECTION Sets the motor's direction
 *    - STEP steps once on rising edge
 *
 * The driver parameter returned is used as the first parameter to each of the
 * functions in this library.
 *
 * The enabled motor is in its default state of park, and ready for a movement
 * command as soon as this function returns.
 *
 * \param[out] motor The stepper motor driver to initialize
 * \param[in] port The port being used
 * \param[in] DISABLE_bm The DISABLE pin bm
 * \param[in] DIRECTION_bm The DIRECTION pin bm
 * \param[in] STEP_bm The STEP pin bm
 * \param[in] timer The timer to use for this stepper (TIMER0 or TIMER1) */
void SM_init(SM_t *motor, PORT_t *port, uint8_t DISABLE_bm,
             uint8_t DIRECTION_bm, uint8_t STEP_bm, SM_timer_t timer) {

   /* initialize the driver */
   motor->port = port;
   motor->DISABLE_bm = DISABLE_bm;
   motor->DIRECTION_bm = DIRECTION_bm;
   motor->STEP_bm = STEP_bm;
   motor->timer = timer;

   /* initialize IO pins */
   port->OUTSET = DIRECTION_bm; // set dir high as default
   port->OUTCLR = DISABLE_bm | STEP_bm;
   port->DIRSET = DIRECTION_bm | DISABLE_bm | STEP_bm; // set all as output

   SM_timer_init(motor, timer); // Must come first, as brake modifies the timer
   SM_brake(motor); // handles setting the speed_ramp field to defaults
}

/** \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerates
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param[in] motor the motor to move
 *  \param[in] step the number of steps to move
 *  \param[in] accel acceleration rate to use, in 0.01 rad/sec^2
 *  \param[in] decel decleration rate to use, in 0.01 rad/sec^2
 *  \param[in] speed max speed, in 0.01 rad/sec */
void SM_move(SM_t *motor,
             int16_t step, uint16_t accel, uint16_t decel, uint16_t speed) {
   /* Number of steps before we hit max speed. */
   uint16_t max_s_lim;
   /* Number of steps before we must start deceleration
    * (if accel does not hitmax speed). */
   uint16_t accel_lim;

   SM_SRD_t *speed_ramp = &(motor->speed_ramp);

   // Set direction from sign on step value.
   if (step < 0){
      speed_ramp->direction = BACKWARD;
      step = -step;
   }
   else{
      speed_ramp->direction = FORWARD;
   }

   // If moving only 1 step.
   if (step == 1){
      // Move one step...
      speed_ramp->accel_count = -1;
      // ...in DECEL state.
      speed_ramp->run_state = DECEL;
      // Just use a short delay
      speed_ramp->step_delay = T1_FREQ / 1000; //1ms delay

      // use dummy period as first period, has no effect on motor
      SM_timer_set_period(motor->timer, T1_FREQ / 100000); // 10us to first step
      SM_timer_start(motor->timer);
   }
   // Only move if number of steps to move is not zero.
   else if (step != 0){
      // Refer to documentation for detailed information about these
      // calculations.

      // Set max speed limit, by calc min_delay to use in timer.
      // min_delay = (alpha / tt)/ w
      speed_ramp->min_delay = A_T_x100 / speed;

      // Set accelration by calc the first (c0) step delay .
      // step_delay = 1/tt * sqrt(2*alpha/accel)
      // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000)
      // / (accel*100) )/10000
      speed_ramp->step_delay = (T1_FREQ_148 * sqrt_Taylor(A_SQ / accel))/100;

      // Find out after how many steps does the speed hit the max speed limit.
      // max_s_lim = speed^2 / (2*alpha*accel)
      max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);
      // If we hit max speed limit before 0,5 step it will round to 0.
      // But in practice we need to move atleast 1 step to get any speed at all.
      if (max_s_lim == 0){
         max_s_lim = 1;
      }

      // Find out after how many steps we must start deceleration.
      // n1 = (n1+n2)decel / (accel + decel)
      accel_lim = ((long)step*decel) / (accel+decel);
      // We must accelrate at least 1 step before we can start deceleration.
      if (accel_lim == 0){
         accel_lim = 1;
      }

      // Use the limit we hit first to calc decel.
      if (accel_lim <= max_s_lim){
         speed_ramp->decel_val = accel_lim - step;
      }
      else{
         speed_ramp->decel_val = -((long)max_s_lim*accel)/decel;
      }
      // We must decelrate at least 1 step to stop.
      if (speed_ramp->decel_val == 0){
         speed_ramp->decel_val = -1;
      }

      // Find step to start decleration.
      speed_ramp->decel_start = step + speed_ramp->decel_val;

      // If the maximum speed is so low that we dont need to go via accelration
      // state.
      if (speed_ramp->step_delay <= speed_ramp->min_delay){
         speed_ramp->step_delay = speed_ramp->min_delay;
         speed_ramp->run_state = RUN;
      }
      else{
         speed_ramp->run_state = ACCEL;
      }

      // Reset counter.
      speed_ramp->accel_count = 0;

      // use dummy period as first period, has no effect on motor
      SM_timer_set_period(motor->timer, T1_FREQ / 100000); // 10us to first step
      SM_timer_start(motor->timer);
   }
}

/** \brief Enables the stepper motor driver.
 *
 * Pulls the DISABLE pin low to enable the stepper motor driver. Any moves which
 * were occurring when the motor was disabled have been cancelled; this
 * function will not reinitiate a cancelled move.
 *
 * \param[in] motor the motor to enable (pull DISABLE low) */
void SM_enable(SM_t *motor) {
   motor->port->OUTCLR = motor->DISABLE_bm;
}

/** \brief Disables the stepper motor driver.
 *
 * Drives the DISABLE hi to disable the stepper motor driver. Additionally,
 * any moves which are ongoing will be cancelled. A cancelled move cannot be
 * reinitiated.
 *
 * Disabling is different from braking in that this function disables the
 * stepper motor driver, allowing the motor to freewheel (although spinning the
 * motor while it is connected to the stepper driver may fry the driver). When
 * the stepper motor is parked, via a call to SM_brake(), the current movement
 * will be canceled, and the full holding torque output of the motor is being
 * applied. However, the motor will be ready for a new movement at any time
 * while in the parked state, whereas the motor must be reenabled through a call
 * to SM_enable if it has been disabled before a new movement can be initiated.
 *
 * \param[in] motor the motor to disable (pull /DISABLE hi) */
void SM_disable(SM_t *motor) {
   SM_brake(motor);                         // stop motor, cancel move
   motor->port->OUTSET = motor->DISABLE_bm; // disable the driver
}

/** \brief Parks the motor in its current position.
 *
 * This is the default motor state while it is waiting for a new movement to be
 * initiated. The park function is implemented here so that an ongoing movement
 * can be cancelled immediately.
 *
 * In the parked state, the motor is outputting its full holding torque to
 * maintain its current position. Note that momentum in the motor may cause it
 * to move past the location it was when when the brake was initiated; this is
 * a powered brake, not an emergency brake;
 *
 * \param[in] motor the motor to brake */
void SM_brake(SM_t *motor) {
   SM_timer_stop(motor->timer);              // stop the motor
   SM_speed_ramp_init(&(motor->speed_ramp)); // reset speed ramp, cancels move
}

/* Internal Function Definitions **********************************************/

/** \brief Initializes a speed ramp.
 *
 * Sets all the values of the speed ramp to defaults, so that the motor starts
 * in a known state
 *
 * \param[in] speed_ramp the speed ramp to initialize */
static void SM_speed_ramp_init(SM_SRD_t *speed_ramp) {
   speed_ramp->run_state = STOP;
   speed_ramp->direction = FORWARD;
   speed_ramp->step_delay = 0;
   speed_ramp->decel_start = 0;
   speed_ramp->decel_val = 0;
   speed_ramp->min_delay = 0;
   speed_ramp->accel_count = 0;
   speed_ramp->last_accel_delay = 0;
   speed_ramp->step_count = 0;
   speed_ramp->rest = 0;
}

/** \brief Initializes the library timer 0 or 1 based on given timer parameter.
 *
 * Sets up the given timer for use with this library. This involves setting the
 * clock prescaler and mode, but the period and overflow interrupt enable are
 * left to the SM_move() and ISR for the corresponding timer.
 *
 * This function also handles mapping the appropriate speed ramp to the
 *
 * \param[in] timer The timer to initialize (TIMER0 or TIMER1)
 * \param[in] speed_ramp The speed ramp
 */
static void SM_timer_init(SM_t *motor, SM_timer_t timer) {
   if (timer == TIMER0) {
      TC0_ConfigWGM(&SM_TIMER0, TC_WGMODE_NORMAL_gc);
      TC0_ConfigClockSource(&SM_TIMER0, TC_CLKSEL_DIV64_gc);/* 32M/64 == 500K */
      motor0 = motor;
   }
   else {
      TC1_ConfigWGM(&SM_TIMER1, TC_WGMODE_NORMAL_gc);
      TC1_ConfigClockSource(&SM_TIMER1, TC_CLKSEL_DIV64_gc);/* 32M/64 == 500K */
      motor1 = motor;
   }

   motor->timer = timer;
}

static void SM_timer_OVF_handler(SM_t *motor) {
   uint16_t new_step_delay = 0;
   SM_SRD_t *speed_ramp = &(motor->speed_ramp);

   /* set next step delay */
   SM_timer_set_period(motor->timer, speed_ramp->step_delay);

   switch(speed_ramp->run_state) {
      case STOP:
         speed_ramp->step_count = 0;
         speed_ramp->rest = 0;

         /* stop the timer */
         SM_timer_stop(motor->timer);
         break;

      case ACCEL:
         SM_step(motor);
         ++(speed_ramp->step_count);
         ++(speed_ramp->accel_count);
         new_step_delay = speed_ramp->step_delay -
            (((2 * (long)speed_ramp->step_delay) + speed_ramp->rest)
             /(4 * speed_ramp->accel_count + 1));
         speed_ramp->rest =
            ((2 * (long)speed_ramp->step_delay) + speed_ramp->rest)
            %(4 * speed_ramp->accel_count + 1);

         // Check if we should start deceleration.
         if (speed_ramp->step_count >= speed_ramp->decel_start) {
            speed_ramp->accel_count = speed_ramp->decel_val;
            speed_ramp->run_state = DECEL;
         }
         // Check if we hit max speed.
         else if (new_step_delay <= speed_ramp->min_delay) {
            speed_ramp->last_accel_delay = new_step_delay;
            new_step_delay = speed_ramp->min_delay;
            speed_ramp->rest = 0;
            speed_ramp->run_state = RUN;
         }
         break;

      case RUN:
         SM_step(motor);
         ++(speed_ramp->step_count);
         new_step_delay = speed_ramp->min_delay;

         // Check if we should start deceleration.
         if (speed_ramp->step_count >= speed_ramp->decel_start) {
            speed_ramp->accel_count = speed_ramp->decel_val;
            // Start decelaration with same delay as accel ended with.
            new_step_delay = speed_ramp->last_accel_delay;
            speed_ramp->run_state = DECEL;
         }
         break;

      case DECEL:
         SM_step(motor);
         ++(speed_ramp->step_count);
         ++(speed_ramp->accel_count);
         new_step_delay = speed_ramp->step_delay -
            (((2 * (long)speed_ramp->step_delay) + speed_ramp->rest)
             /(4 * speed_ramp->accel_count + 1));
         speed_ramp->rest =
            ((2 * (long)speed_ramp->step_delay)+speed_ramp->rest)
            %(4 * speed_ramp->accel_count + 1);

         // Check if we at last step
         if (speed_ramp->accel_count >= 0){
            speed_ramp->run_state = STOP;
         }
         break;
   }

   speed_ramp->step_delay = new_step_delay;
}

/** \brief Sets the period of the given timer, and restarts it from 0.
 *
 * Restarts the timer from 0, incrementing by 1 every timer clock cycle, and
 * changes the period of the timer to the given value. This also clears any CC
 * channels on the timer.
 *
 * \param[in] timer the timer to set (TIMER0 or TIMER1)
 * \param[in] period the new timer period (TOP) */
static void SM_timer_set_period(SM_timer_t timer, uint16_t period) {
   if (timer == TIMER0) {
      TC_SetCount(&SM_TIMER0, 0x0000);
      TC_SetPeriod(&SM_TIMER0, period);
   }
   else {
      TC_SetCount(&SM_TIMER1, 0x0000);
      TC_SetPeriod(&SM_TIMER1, period);
   }
}

/** \brief Enables OVF interrupts on the given timer.
 *
 * As this library is interrupt driven, enabling these interrupts essentially
 * enables the given SM_move() command to be carried out by the motor.
 *
 * \param[in] timer the timer to start */
static void SM_timer_start(SM_timer_t timer) {
   if (timer == TIMER0) {
      TC0_SetOverflowIntLevel(&SM_TIMER0, TC_OVFINTLVL_HI_gc);
   }
   else {
      TC1_SetOverflowIntLevel(&SM_TIMER1, TC_OVFINTLVL_HI_gc);
   }

   /* enable hi level interrupts */
   PMIC.CTRL |= PMIC_HILVLEN_bm;
}

/** \brief Disables OVF interrupts on the given timer.
 *
 * As this library is interrupt driven, this effectively stops the motor. When
 * stopped, the stepper motor will go into its holding torque phase, unless the
 * motor driver is disabled.
 *
 * \param[in] timer the timer to stop. */
static void SM_timer_stop(SM_timer_t timer) {
   if (timer == TIMER0) {
      TC0_SetOverflowIntLevel(&SM_TIMER0, TC_OVFINTLVL_OFF_gc);
   }
   else {
      TC1_SetOverflowIntLevel(&SM_TIMER1, TC_OVFINTLVL_OFF_gc);
   }
}

/** \brief Moves the stepper motor one step in the given direction.
 *
 *  The stepper motor driver steps the motor according to the DIRECTION pin on
 *  the rising edge of the STEP pin. If the motor is moving in the opposite
 *  direction required, switch the A motor lead with B, and the /A motor lead
 *  with /B.
 *
 *  \param[in] driver The driver to step
 *  \param[in] direction The direction to move (FORWARD or BACKWARD) */
void SM_step(SM_t *motor) {
   PORT_t *port = motor->port;
   uint8_t DIRECTION_bm = motor->DIRECTION_bm;
   uint8_t STEP_bm = motor->STEP_bm;

   port->OUTCLR = STEP_bm;

   /* set the direction, swap motor leads if wrong */
   if (motor->speed_ramp.direction == FORWARD) {
      port->OUTSET = DIRECTION_bm;
   }
   else {
      port->OUTCLR = DIRECTION_bm;
   }

   /* step once */
   delay_us(2);
   port->OUTSET = STEP_bm;
}

/** \brief Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
static unsigned long sqrt_Taylor(unsigned long x) {
   register unsigned long xr;  // result register
   register unsigned long q2;  // scan-bit register
   register unsigned char f;   // flag (one bit)

   xr = 0;                     // clear result
   q2 = 0x40000000L;           // higest possible result bit
   do
   {
      if ((xr + q2) <= x)
      {
         x -= xr + q2;
         f = 1;                  // set flag
      }
      else{
         f = 0;                  // clear flag
      }
      xr >>= 1;
      if (f){
         xr += q2;               // test flag
      }
   } while (q2 >>= 2);          // shift twice
   if (xr < x){
      return xr +1;             // add for rounding
   }
   else{
      return xr;
   }
}

/* Interrupt Service Routines *************************************************/

/** \brief Timer 0 overflow ISR.
 *
 *  Increments/decrements the position of the stepper motor exept after last
 *  position, when it stops. A new step delay is calculated to follow the
 *  desired speed profile based on the accel/decel parameters from the initial
 *  SM_move() command.  */
ISR(SM_TIMER0_OVF_vect)
{
   SM_timer_OVF_handler(motor0);
}

/** \brief Timer 1 overflow ISR.
 *
 *  Increments/decrements the position of the stepper motor exept after last
 *  position, when it stops. A new step delay is calculated to follow the
 *  desired speed profile based on the accel/decel parameters from the initial
 *  SM_move() command.  */
ISR(SM_TIMER1_OVF_vect)
{
   SM_timer_OVF_handler(motor1);
}

