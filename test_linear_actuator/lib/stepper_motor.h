/** \defgroup SM_API Stepper Motor Library API */ /** @{ */
/** \file stepper_motor.h
 *
 * \brief Enables interfacing with the GECKO G213V stepper motor driver.
 *
 * This library simplifies interfacing with the Gecko G213V stepper motor
 * drivers. These drivers are used ot interface with the linear actuators on the
 * pit crew's machine. The Stepper motor drivers require 3 pins to interface
 * with them
 *    - /DISABLE (pulled low to disable)
 *    - DIRECTION Sets the motor's direction
 *    - STEP steps once on rising edge
 *
 * If the direction of the motor is the opposite of what is required for the
 * system, simply switch the A lead with the B lead, and the /A lead with the /B
 * lead on the motor port of the driver.
 *
 * The library is initialized through a call to SM_init(), which handles setting
 * up a port to communicate with the stepper motor driver, as well as the timer
 * interrupt used by this library to control the step pulses. Upon
 * initialization, the motor is disabled by pulling the /DISABLE pin low. In
 * order to move the motor, use the function SM_enable().
 *
 * A stepper motor movement is initiated by a call to SM_move(). An initial
 * calculation is made to determine the speed profile of the move before it is
 * executed. Once this set up has been performed, the timer for the motor is
 * started, and the move is carried out by the ISR which is vectored on the
 * timer's overflow, and the step pulse period is therefore determined by the
 * timer's period.
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
#ifndef _STEPPER_MOTOR_H_
#define _STEPPER_MOTOR_H_

/* Include Directives *********************************************************/

#include <avr/io.h>
#include "tc_driver.h"

/* Macro Defintions ***********************************************************/

#define SM_TIMER0 TCC0 ///< First timer this library supports
#define SM_TIMER1 TCC1 ///< Second timer this library supports

#define SM_TIMER0_OVF_vect TCC0_OVF_vect ///< First timer overflow IV
#define SM_TIMER1_OVF_vect TCC1_OVF_vect ///< Second timer overflow IV

#define T1_FREQ 4000000 ///< Motor timer frequency, modify as appropriate

#define SPR 400 ///< Steps per full revolution

/** \name Maths Constants */
///@{

#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*676)/1000)) // div by 100, scale by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

///@}

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief Direction of stepper motor travel */
typedef enum SM_direction { BACKWARD, FORWARD } SM_dir_t;

/** \brief The two timers this library supports */
typedef enum SM_timers { TIMER0, TIMER1 } SM_timer_t;

/** \brief The states a stepper motor can be in */
typedef enum SM_speed_ramp_state { STOP, ACCEL, DECEL, RUN } SM_SRD_state_t;

/** \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  It is initialized during a call to SM_move(), and when stepper motor is
 *  moving (timer interrupt running) data is read/updated when calculating the
 *  new step delays.
 */
typedef struct speed_ramp_data{
   SM_SRD_state_t run_state;  ///< What part of speed ramp we are in
   SM_dir_t direction;        ///< Stepper motor direction
   uint16_t step_delay;       ///< next timer delay period, == acc rate at start
   uint16_t decel_start;      ///< What step_pos to start deceleration
   int16_t decel_val;         ///< deceleration rate
   uint16_t min_delay;        ///< Minimum time delay (max speed)
   int16_t accel_count;       ///< used when acc/dec to calculate step_delay
   uint16_t last_accel_delay; ///< remember last accel delay for run state
   uint16_t step_count;       ///< Count the steps while moving
   uint16_t rest;             ///< remainder from new_step calcuation
} SM_SRD_t;

/** \brief Contains the static data needed to interface with a stepper motor
 * driver */
typedef struct stepper_motor_driver  {
   PORT_t *port;         ///< Stepper Motor Driver Control Port
   uint8_t DISABLE_bm;   ///< /DISABLE pin bm
   uint8_t DIRECTION_bm; ///< DIRECTION pin bm
   uint8_t STEP_bm;      ///< STEP pin bm
   SM_SRD_t speed_ramp;  ///< Speed ramp static data struct
   SM_timer_t timer;     ///< which timer to use for this motor (TIMER0, TIMER1)
} SM_t;

/* Function Prototypes ********************************************************/

/** \name Library Initializer */
///@{
void SM_init(SM_t *motor, PORT_t *port, uint8_t DISABLE_bm,
             uint8_t DIRECTION_bm, uint8_t STEP_bm, SM_timer_t timer);
///@}

void SM_move(SM_t *motor,
             int16_t step, uint16_t accel, uint16_t decel, uint16_t speed);
void SM_enable(SM_t *motor);
void SM_disable(SM_t *motor);
void SM_brake(SM_t *motor);


#endif /* end of include guard: _STEPPER_MOTOR_H_ */
/** @} */ /* End of \defgroup SM_API */
