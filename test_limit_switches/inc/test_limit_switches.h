/** \file test_limit_switches.h
 *
 * \brief System software for testing the linear actuators with limit switches.
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

#ifndef _TEST_LIMIT_SWITCH_H_
#define _TEST_LIMIT_SWITCH_H_

/* Include Directives *********************************************************/

#include "board.h"
#include "avr_compiler.h"
#include "clksys_driver.h"
#include "pressure_sensor.h"
#include "usart_bc.h"
#include "linear_actuator.h"

/* Macro Definitions **********************************************************/

/** \name Stepper Motor Definitions */
///@{

#define LA_port PORTA            ///< stepper motor control port

#define LA_NEEDLE_PITCH        500             ///< needle act pitch (0.5in)
#define LA_NEEDLE_DISABLE_bm   PIN3_bm         ///< needle act /DISABLE pin
#define LA_NEEDLE_DIRECTION_bm PIN4_bm         ///< needle act DIRECTION pin
#define LA_NEEDLE_STEP_bm      PIN5_bm         ///< needle act STEP pin
#define LA_NEEDLE_TIMER        SM_TIMER_NEEDLE ///< needle act timer to use

#define LA_RING_PITCH        125             ///< ring act pitch (0.125in)
#define LA_RING_DISABLE_bm   PIN0_bm         ///< ring act /DISABLE pin
#define LA_RING_DIRECTION_bm PIN1_bm         ///< ring act DIRECTION pin
#define LA_RING_STEP_bm      PIN2_bm         ///< ring act STEP pin
#define LA_RING_TIMER        SM_TIMER_RING   ///< ring act timer to use

#define BRAKE_PIN_vect PORTR_INT0_vect ///< Brake pin interrupt vector

///@}

/* Static Messages ************************************************************/
/** \name Help Message */
///@{

/** \brief Motor interface help message */
const char *help_message =
"\n\r--------------------------------------------------------------\n\r"
"Atmel AVR446 - Linear speed control of stepper motor\n\r\n\r"
"?  - Show help\n\ra [data] - Set acceleration (range: 57 to 52K, 2 to 13K)\n\r"
"d [data] - Set deceleration (range: 57 to 52K, 2 to 13K)\n\r"
"s [data] - Set speed (range: 10 to 52K, 2 to 13K)\n\r"
"m [data] - Move [data] steps (range: -32K to 32K, -20K to 20K)\n\r"
"move [steps] [accel] [decel] [speed]\n\r"
"         - Move with all parameters given\n\r"
"<enter>  - Repeat last move\n\r\n\r"
"acc/dec data given in 0.001*in/sec^2 (1000 = 1 in/sec^2)\n\r"
"speed data given in 0.001*in/sec (1000 = 1 in/sec)\n\r"
"move data given in 0.001*in (1000 = 1 in\n\r"
"--------------------------------------------------------------\n\r";

///@}

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief The command line instruction types */
typedef enum command_types {
   STEP,   ///< Move the given number of steps
   MOVE,   ///< Move with all the information given
   ACCEL,  ///< Set acceleration
   DECEL,  ///< Set deceleration
   SPEED,  ///< Set max speed
   REPEAT, ///< repeat last move
   HELP,   ///< Show help menu
   NONE    ///< No command retrieved
} command_t;

/* Function Prototypes ********************************************************/

/** \name Board Setup Functions */
///@{
static void setup_clocks(void);
static void setup_LEDs(void);
static void setup_switches(uint8_t switch_mask);
static void setup_pressure_sensor(PS_t *pressure_sensor);
static void setup_USART_BC(void);
static void setup_linear_actuators(LA_t *needle_actuator, LA_t *ring_actuator);
///@}

static void show_help_message(void);
static void show_motor_data(int16_t position, uint16_t acceleration,
                            uint16_t deceleration, uint16_t speed,
                            int16_t steps);
static command_t parse_command(int16_t *steps, uint16_t *accel, uint16_t *decel,
                               uint16_t *speed);
static uint8_t find_next_param(char *command_buffer, uint8_t start_position);

#endif /* end of include guard: _TEST_LIMIT_SWITCH_H_ */

