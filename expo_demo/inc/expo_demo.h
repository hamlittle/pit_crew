/** \file expo_demo.h
 *
 * \brief System software for demoing at the senior project expo.
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

/** \name State Machine Definitions */
///@{

#define LA_NEEDLE_SPEED 1000        ///< needle carriage max speed, 0.001 in/s
#define LA_NEEDLE_ACCEL 400         ///< needle carriage accel, 0.001 in/s^2
#define LA_NEEDLE_DECEL 400         ///< needle carriage decel, 0.001 in/s^2
#define LA_NEEDLE_ENGAGE_DIST 1000  ///< needle carriage engage travel, 0.001 in
#define LA_NEEDLE_DETECT_DIST 500   ///< needle carriage engage travel, 0.001 in
/// needle carriage retract travel, 0.001 in
#define LA_NEEDLE_RETRACT_DIST (-LA_NEEDLE_ENGAGE_DIST-LA_NEEDLE_DETECT_DIST))

#define LA_RING_SPEED 1000        ///< needle carriage max speed, 0.001 in/s
#define LA_RING_ACCEL 400         ///< needle carriage accel, 0.001 in/s^2
#define LA_RING_DECEL 400         ///< needle carriage decel, 0.001 in/s^2
#define LA_RING_RETAIN_DIST 500  ///< needle carriage engage travel, 0.001 in
/// needle carriage retract travel, 0.001 in
#define LA_RING_RELEASE_DIST (-LA_RING_RETAIN_DIST)
///@}

/** \name Safety Switch Defininitions */
///@{

#define SS_PORT PORTC     ///< safety switch port
#define SS_PIN_bm PIN4_bm ///< safety swtich pin

/** \brief Check if the safety switch is open
 *
 * The safety switch is closed if the safety switch pin is high. When the
 * switch opens, the 5v signal is no longer connected to the pin, and the
 * safety switch pin will read low. */
#define SS_OPEN() (!(SS_PORT.IN & SS_PIN_bm))

///@}

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
#define BRAKE_PIN_bm PIN6_bm           ///< Switch 6, translates to PR[0]

///@}

/** \name Macro Defined Functions */
///@{

/** \brief Returns the current state of the machine.
 *
 * The valid states of the machine are enumerated in PC_STATE_t.
 *
 * \param[in] machine The machine to get the state of
 *
 * \return The current state of the machine */
#define get_state(_machine) ((_machine)->state)

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
   STEP1,   ///< Move the given number of steps
   STEP2,   ///< Move the given number of steps
   MOVE1,   ///< Move with all the information given
   MOVE2,   ///< Move with all the information given
   ACCEL1,  ///< Set acceleration
   ACCEL2,  ///< Set acceleration
   DECEL1,  ///< Set deceleration
   DECEL2,  ///< Set deceleration
   SPEED1,  ///< Set max speed
   SPEED2,  ///< Set max speed
   CAL,     ///< calibrate the pressure sensor
   SCAN,    ///< scan the sensor
   HELP,    ///< Show help menu
   NONE     ///< No command retrieved
} command_t;

/** \brief The different states for the machine.
 *
 * The system is modeled as a finite state machine. These are the states the
 * machine can be in. */
typedef enum pit_crew_state {
   IDLE,      ///< Idling (home state)
   RETAIN,    ///< Retaining Ring descending
   ENGAGE,    ///< Needles approaching peach
   DETECT,    ///< Checking for pits
   DISENGAGE, ///< Needles retracting
   RELEASE,   ///< Retaining ring releasing peach
   PASS,      ///< Tell the user the result
   STOP       ///< Lid is removed
} PC_STATE_t;

/** \brief Pit Crew machine.
 *
 * Contains the static data for all of the various hardware the board must
 * interface with, as well as the current state of the machine */
typedef struct pit_crew_machine  {
   PC_STATE_t state;
   LA_t needle_carriage;
   LA_t retaining_ring;
   PS_t pressure_sensor;
} PC_t;

/* Function Prototypes ********************************************************/

/** \name Board Setup Functions */
///@{
static void setup_clocks(void);
static void setup_LEDs(void);
static void setup_switches(uint8_t switch_mask);
static void setup_safety_switch(void);
static void setup_pressure_sensor(PS_t *pressure_sensor);
static void setup_USART_BC(void);
static void setup_linear_actuators(LA_t *needle_actuator, LA_t *ring_actuator);
///@}

INLINE void set_state(PC_t *machine, PC_STATE_t state);
static void show_help_message(void);
static void show_motor_data(int16_t position, uint16_t acceleration,
                            uint16_t deceleration, uint16_t speed,
                            int16_t steps);
static command_t parse_command(int16_t *steps1, uint16_t *accel1,
                               uint16_t *decel1, uint16_t *speed1,
                               int16_t *steps2, uint16_t *accel2,
                               uint16_t *decel2, uint16_t *speed2);
static uint8_t find_next_param(char *command_buffer, uint8_t start_position);

#endif /* end of include guard: _TEST_LIMIT_SWITCH_H_ */

