/** \file test_linear_actuator.h
 *
 * \brief System software for testing the linear actuators.
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
 * limitations under the License. \endverbatim */

#ifndef _TEST_LINEAR_ACTUATOR_H_
#define _TEST_LINEAR_ACTUATOR_H_

/* Include Directives *********************************************************/

#include "board.h"
#include "avr_compiler.h"
#include "clksys_driver.h"
#include "pressure_sensor.h"
#include "usart_bc.h"
#include "stepper_motor.h"

/* Macro Definitions **********************************************************/

#define SM_port PORTC            ///< stepper motor control port
#define SM_DISABLE_bm PIN0_bm    ///< stepper motor /DISABLE pin
#define SM_DIRECTION_bm PIN1_bm  ///< stepper motor DIRECTION pin
#define SM_STEP_bm PIN4_bm       ///< stepper motor STEP pin
#define SM_TIMER TIMER0          ///< stepper motor timer to use

#define BRAKE_PIN_vect PORTR_INT0_vect ///< Brake pin interrupt vector

/* Static Messages ************************************************************/
/** \name Help Message */
///@{

/** \brief Motor interface help message */
const char *help_message =
"\n\r--------------------------------------------------------------\n\r"
"Atmel AVR446 - Linear speed control of stepper motor\n\r\n\r"
"?  - Show help\n\ra [data] - Set acceleration (range: 71 - 32000)\n\r"
"d [data] - Set deceleration (range: 71 - 32000)\n\r"
"s [data] - Set speed (range: 12 - motor limit)\n\r"
"m [data] - Move [data] steps (range: -64000 - 64000)\n\r"
"move [steps] [accel] [decel] [speed]\n\r"
"         - Move with all parameters given\n\r"
"<enter>  - Repeat last move\n\r\n\r"
"acc/dec data given in 0.01*rad/sec^2 (100 = 1 rad/sec^2)\n\r"
"speed data given in 0.01*rad/sec (100 = 1 rad/sec)\n\r"
"--------------------------------------------------------------\n\r";

///@}

/* Typedefs, Enums, and Structs ***********************************************/

/* Function Prototypes ********************************************************/

/** \name Board Setup Functions */
///@{
static void setup_clocks(void);
static void setup_LEDs(void);
static void setup_switches(uint8_t switch_mask);
static void setup_pressure_sensor(PS_t *pressure_sensor);
static void setup_USART_BC(void);
static void setup_motor(void);
///@}

static void show_help_message(void);
static void show_motor_data(int16_t position, uint16_t acceleration,
                            uint16_t deceleration, uint16_t speed,
                            int16_t steps);

#endif /* end of include guard: _TEST_LINEAR_ACTUATOR_H_ */

