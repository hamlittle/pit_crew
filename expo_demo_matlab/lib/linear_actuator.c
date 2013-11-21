/** \file linear_actuator.c
 *
 * \brief Linear Actuator wrapper library for the stepper motor library.
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

#include "linear_actuator.h"

/* Function Definitions *******************************************************/

/** \brief Initialize this library.
 *
 * Sets up the linear actuator, which acts as a wrapper for the stepper motor
 * library, by converting units of angular velocity and acceleration to linear
 * based on the pitch of the lead screw attached to the given motor.
 *
 * The timer parameters is one of SM_TIMER_NEEDLE or SM_TIMER_RING, as
 * appropriate.  This must be initialized correctly for the stepper motor
 * library to work correctly.
 *
 * The linear actuator is in the disabled state on initialization. In order to
 * move the actuator, use LA_enable().
 *
 * \param[out] actuator The actuator to initialize
 * \param[in] pitch Lead screw pitch (in 0.001 inches)
 * \param[in] port The port the actuator is hooked up to
 * \param[in] DISABLE_bm DISABLE pin bm
 * \param[in] DIRECTION_bm DISABLE pin bm
 * \param[in] STEP_bm STEP pin bm
 * \param[in] timer The stepper motor timer to use */
void LA_init(LA_t *actuator, uint16_t pitch, PORT_t *port,
             uint8_t DISABLE_bm, uint8_t DIRECTION_bm, uint8_t STEP_bm,
             SM_timer_t timer) {
   SM_init(&(actuator->motor), port, DISABLE_bm, DIRECTION_bm, STEP_bm, timer);
   actuator->pitch = pitch;
}


/** \brief Moves the linear actuator using the given velocity
 * profile.
 *
 * Angular steps, accel, decel, and speed are calculated using the pitch of the
 * given actuator, and the commands are sent to the stepper motor library to
 * enable the move.
 *
 * \param[in] actuator The actuator to move
 * \param[in] dist The distance to move (in 0.001 in)
 * \param[in] accel The acceleration rate (in 0.001 in/s^2)
 * \param[in] decel The deceleration rate (in 0.001 in/s^2)
 * \param[in] speed The max speed to move (in 0.001 in/s) */
void LA_move(LA_t *actuator, int16_t dist, uint16_t accel, uint16_t decel,
             uint16_t speed) {
   int16_t steps;
   uint16_t accel_ang, decel_ang, speed_ang;
   uint16_t pitch = actuator->pitch;

   steps = (int16_t)((int32_t)dist * SPR / pitch);
   speed_ang = (uint16_t)((uint32_t)speed * 200 * 314 / (pitch * 100));
   accel_ang = (uint16_t)((uint32_t)accel * 200 * 314 / (pitch * 100));
   decel_ang = (uint16_t)((uint32_t)decel * 200 * 314 / (pitch * 100));

   SM_move(&(actuator->motor), steps, accel_ang, decel_ang, speed_ang);
}

/** \brief Wrapper for SM_enable().
 *
 * Enables the given linear actuator. When a linear actuator is initialized, it
 * is in the disabled state. This must be called before the linear actuator can
 * initiate a move.
 *
 * \param[in] actuator The actuator the enable */
void LA_enable(LA_t *actuator) {
   SM_enable(&(actuator->motor));
}

/** \brief Wrapper for SM_disable();
 *
 * Disables the given linear actuator. When Disabled, the actuator will spin
 * freely, although this is never a good thing to do while it is hooked up to
 * the motor driver, as this may fry the drivers. So, dont spin the actuators
 * while they are still hooked up to the drivers.
 *
 * \param[in] actuator The actuator to disable */
void LA_disable(LA_t *actuator) {
   SM_disable(&(actuator->motor));
}

/** \brief Wrapper for SM_brake().
 *
 * Brakes the linear actuator, putting it in the stop (parked) state. In this
 * state, the motor is outputting its full holding torque. This is a powered
 * brake, in contrast with LA_disable, which allows the motor to spin freely.
 *
 * \param[in] actuator The actuator to brake (park) */
void LA_brake(LA_t *actuator) {
   SM_brake(&(actuator->motor));
}

/** \brief Returns the actuator's position.
 *
 * Position data returned is in units of 0.001in. The returned position is the
 * distance the actuator is away from its home position. The linear actuators
 * automatically home themselves when they hit the limit switches, however, the
 * home position can be changed by a call to LA_home().
 *
 * \param[in] actuator The actuator to get the position.
 *
 * \return Position of the motor since the last home */
int16_t LA_get_position(LA_t *actuator) {
   return (int16_t)((int32_t)(SM_get_position(&(actuator->motor)))
                    * actuator->pitch / SPR);
}

/** \brief Moves the actuator to its home position.
 *
 * This moves the actuator back to its home position, and homes the position to
 * 0.
 *
 * \param[in] acutator The actuator to move to home */
void LA_go_to_home(LA_t *actuator) {

   /* just move a distance we know the limit switch is within
    * motor will stop when it hits it */
   LA_move(actuator, -4000, 400, 400, 1600);
}
