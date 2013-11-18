/** \defgroup LA_API Linear Actuator Library API */ /** @{ */
/** \file linear_actuator.h
 *
 * \brief Linear Actuator wrapper library for the stepper motor library.
 *
 * \todo TODO document linear_actuator.h file
 *
 * \note For the linear actuator pitches of 0.5in for the needle carriage, and
 * 0.125 in for the retaining ring, the limits imposed on the distance,
 *   speed, and accel/decel values are:
 * <table>
 *    <tr>
 *       <th>Variable</th>
 *       <th>Needle</th>
 *       <th>Ring</th>
 *    </tr>
 *    <tr>
 *       <td>Distance</td>
 *       <td>-32K to 32K</td>
 *       <td>-20K to 20K</td>
 *    </tr>
 *    <tr>
 *       <td>Speed</td>
 *       <td>10 to 52K</td>
 *       <td>2 to 13K</td>
 *    </tr>
 *    <tr>
 *       <td>Accel/Decel</td>
 *       <td>57 to 52K</td>
 *       <td>15 to 13K</td>
 *    </tr>
 * </table>
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

#ifndef _LINEAR_ACTUATOR_H_
#define _LINEAR_ACTUATOR_H_

/* Include Directives *********************************************************/

#include "stepper_motor.h"

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief Linear Actuator used to drive the pit crew machine */
typedef struct linear_actuator  {
   SM_t motor;    ///< The motor driving the linear actuator
   uint16_t pitch; ///< Lead screw pitch, in 0.001 in
} LA_t;

/* Macro Defintions ***********************************************************/

/** \name Macro Defined Functions */
///@{

/** \brief Returns the state linear actuator.
 *
 * The four states of the actuator are defined in the SM_SRD_state_t enum.
 *
 * \param[in] _actuator pointer to the actuator to get the state (type LA_t*)
 *
 * \return The actuator's state
 * \retval SM_STOP the actuator is parked \sa SM_brake()
 * \retval SM_ACCEL the actuator is accelerating to max speed
 * \retval SM_DECEL the actuator is decelerating to rest
 * \retval SM_RUN the actuator is running at max speed */
#define LA_get_motor_state(_actuator) \
   (SM_get_motor_state(&((_actuator)->motor)))

///@}

/* Function Prototypes ********************************************************/

/** \name Library Initializer */
///@{

void LA_init(LA_t *actuator, uint16_t pitch, PORT_t *port,
             uint8_t DISABLE_bm, uint8_t DIRECTION_bm, uint8_t STEP_bm,
             SM_timer_t timer);

///@}

void LA_move(LA_t *actuator, int16_t dist, uint16_t accel, uint16_t decel,
             uint16_t speed);
void LA_enable(LA_t *actuator);
void LA_disable(LA_t *actuator);
void LA_brake(LA_t *actuator);

#endif /* end of include guard: _LINEAR_ACTUATOR_H_ */
/** @} */
