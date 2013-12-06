/** \defgroup SM_API Stepper Motor Library API */ /** @{ */
/** \file stepper_motor.h
 *
 * \brief Enables interfacing with the GECKO G213V stepper motor driver.
 *
 * This library simplifies interfacing with the Gecko G213V stepper motor
 * drivers. These drivers are used ot interface with the linear actuators on the
 * pit crew's machine. The Stepper motor drivers require 3 pins to interface
 * with them
 *    - DISABLE (pulled low to disable)
 *    - DIRECTION Sets the motor's direction
 *    - STEP steps once on rising edge
 *
 * If the direction of the motor is the opposite of what is required for the
 * system, simply switch the A lead with the B lead, and the /A lead with the /B
 * lead on the motor port of the driver.
 *
 * The library is initialized through a call to SM_init(), which handles setting
 * up a port to communicate with the stepper motor driver, as well as the timer
 * interrupt used by this library to control the step pulses.
 *
 * A stepper motor movement is initiated by a call to SM_move(). An initial
 * calculation is made to determine the speed profile of the move before it is
 * executed. Once this set up has been performed, the timer for the motor is
 * started, and the move is carried out by the ISR which is vectored on the
 * timer's overflow, and the step pulse period is therefore determined by the
 * timer's period.
 *
 * Additionally, There are three states the motor can be put into that are
 * directly under the user's control. The disabled state turns off all output to
 * the motor from the motor driver, allowing it to freewheel (this will turn on
 * the RED eror LED on the driver). This will cancel any ongoing movement.
 * Disabling the motor is accomplished through the SM_disable() function.
 * Reenabling the motor is accomplished through a call to SM_enable(). Calling
 * SM_enable while the motor is enabled has no effect.
 *
 * The Park state is similar to the disable state, only it maintains power to
 * the motor, putting it into its holding torque state. This also cancels any
 * ongoing movements, and is initiated by a call to SM_brake(). However, this
 * state does not require a call to SM_enable() in order to start a new
 * movement; a new movement can be started at any time the motor is in the park
 * state, and the movement will begin immediately.
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
#include "stepper_motor.h"
#include "global.h"

/* Macro Defintions ***********************************************************/

#define SM_OFFSET 200 ///< dist b/w needle carriage and retaining ring

/** \name Limit Switch Definitions */
///@{

#define LS_PORT PORTC                      ///< Limit Switch port
#define LS_NEEDLE_PIN_bm PIN0_bm           ///< Needle Limit Switch Pin
#define LS_RING_PIN_bm   PIN1_bm           ///< Ring Limit Switch Pin

///@}

/** \name Stepper Motor Timer Definitions */
///@{

#define TIMER_NEEDLE TCC0 ///< Needle Carriage timer
#define TIMER_RING   TCC1 ///< Retaining Ring timer

#define TIMER_NEEDLE_OVF_vect TCC0_OVF_vect ///< Needle Carriage timer OVF IV
#define TIMER_RING_OVF_vect   TCC1_OVF_vect ///< Retaining Ring timer OVF IV

#define TIMER_FREQ 125000 ///< Both motors timer's frequency (Hz)

///@}

/** \name Maths Constants */
///@{

#define ALPHA (2*3.14159/SPR)                          // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*TIMER_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((TIMER_FREQ*0.676)/100)) // scale 0.676, div 100
#define A_SQ (long)(ALPHA*2*10000000000)               // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)                    // ALPHA*20000

///@}

/** \name Macro Defined Functions */
///@{

/** \brief Returns the state of the motor.
 *
 * The four states of the motor are defined in the SM_SRD_state_t enum.
 *
 * \param[in] _motor pointer to the motor to get the state (type SM_t*)
 *
 * \return The motor's state
 * \retval SM_STOP the motor is parked \sa SM_brake()
 * \retval SM_ACCEL the motor is accelerating to max speed
 * \retval SM_DECEL the motor is decelerating to rest
 * \retval SM_RUN the motor is running at max speed */
#define SM_get_motor_state(_motor) ((_motor)->speed_ramp.run_state)

/** \brief Homes the motor (sets the current position to 0).
 *
 * Homing is performed every time the linear actuator hits its limit switch.
 * However, the home position can additionally be changed by a call to this
 * function. This resets the current position of the motor to 0.
 *
 * \param[in] _motor pointer to the motor to home  */
#define SM_home(_motor) ((_motor)->position = 0)

///@}

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief Direction of stepper motor travel */
typedef enum SM_direction {
   SM_RETRACT, ///< Retract direction (towards home, negative step)
   SM_ENGAGE   ///< Engage direction (towards peach, positive step)
} SM_dir_t;

/** \brief The two timers this library supports */
typedef enum SM_timers {
   SM_TIMER_NEEDLE, ///< Timer for Needle Carriage actuator
   SM_TIMER_RING    ///< Timer for Retaining Ring actuator
} SM_timer_t;

/** \brief The states a stepper motor can be in */
typedef enum SM_speed_ramp_state {
   SM_STOP,  ///< Stopped (aka parked)
   SM_ACCEL, ///< Accelerating towards max speed
   SM_DECEL, ///< Decelerating to speed of 0
   SM_RUN    ///< Running at max speed
} SM_SRD_state_t;

/** \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  It is initialized during a call to SM_move(), and when stepper motor is
 *  moving (timer interrupt running) data is read/updated when calculating
 *  the
 *  new step delays.
 */
typedef struct speed_ramp_data{
   volatile SM_SRD_state_t run_state;  ///< What part of speed ramp we are in
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
   uint8_t DISABLE_bm;   ///< DISABLE pin bm
   uint8_t DIRECTION_bm; ///< DIRECTION pin bm
   uint8_t STEP_bm;      ///< STEP pin bm
   int16_t position;     ///< angular position of motor
   SM_SRD_t speed_ramp;  ///< Speed ramp static data struct
   SM_timer_t timer;     ///< which timer to use for this motor (TIMER0, TIMER1)
} SM_t;

/* Function Prototypes ********************************************************/

/** \name Library Initializer */
///@{
void SM_init(SM_t *motor, PORT_t *port, uint8_t DISABLE_bm,
             uint8_t DIRECTION_bm, uint8_t STEP_bm, SM_timer_t timer);
///@}

void SM_enable(SM_t *motor);
void SM_disable(SM_t *motor);
void SM_brake(SM_t *motor);
void SM_move(SM_t *motor,
             int16_t steps, uint16_t accel, uint16_t decel, uint16_t speed);
int16_t SM_get_position(SM_t *motor);



#endif /* end of include guard: _STEPPER_MOTOR_H_ */
/** @} */ /* End of \defgroup SM_API */
