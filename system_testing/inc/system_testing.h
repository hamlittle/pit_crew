/** \file system_testing.h
 *
 * \brief System software for the Pit Crew Peach Pit Detection Machine.
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
#include "global.h"

/* Macro Definitions **********************************************************/

/** \name Default Manual Operation Settings */ /** @{ */
#define LA_NEEDLE_DFLT_SPEED 2000  ///< needle carriage default max speed
#define LA_NEEDLE_DFLT_ACCEL 1000  ///< needle carriage default accel
#define LA_NEEDLE_DFLT_DECEL 1000  ///< needle carriage default decel

#define LA_RING_DFLT_SPEED 2000  ///< needle carriage default max speed
#define LA_RING_DFLT_ACCEL 1000  ///< needle carriage default accel
#define LA_RING_DFLT_DECEL 1000  ///< needle carriage default decel
/** @} */

/** \name Safety Switch Defininitions */ /** @{ */
#define SS_PORT PORTC     ///< safety switch port
#define SS_PIN_bm PIN4_bm ///< safety swtich pin

/** \brief Check if the safety switch is open
 *
 * The safety switch is set up as a pull up, and the machine frame is grounded,
 * so when the pin goes low, the lid is closed, and when the pin is high, the
 * lid is off
 *
 * \return True if the switch is open (safety cover removed), otherwise false */
#define SS_OPEN() (SS_PORT.IN & SS_PIN_bm)
/** @} */

/** \name Retaining Ring Sensor Definitions */ /** @{ */
#define RS_PORT PORTC   ///< Retaining Ring Sensor Port

/// Retaining Ring Sensor Pin Mask Group
#define RS_PIN_gm (PIN5_bm | PIN6_bm | PIN7_bm)

/** \brief Checks if the peach has been retained.
 *
 * When all three pins connected to the contact sensor read hi, the peach is
 * properly retained.
 *
 * \return True if peach is retained, otherwise false */
#define RETAINED() ((RS_PORT.IN & RS_PIN_gm) == RS_PIN_gm)
/** @} */

/** \name Linear Actuator Hardware Definitions */ /** @{ */
#define LA_port PORTA ///< stepper motor control port

#define LA_NEEDLE_DISABLE_bm   PIN3_bm         ///< needle act /DISABLE pin
#define LA_NEEDLE_DIRECTION_bm PIN4_bm         ///< needle act DIRECTION pin
#define LA_NEEDLE_STEP_bm      PIN5_bm         ///< needle act STEP pin
#define LA_NEEDLE_TIMER        SM_TIMER_NEEDLE ///< needle act timer to use

#define LA_RING_DISABLE_bm   PIN0_bm       ///< ring act /DISABLE pin
#define LA_RING_DIRECTION_bm PIN1_bm       ///< ring act DIRECTION pin
#define LA_RING_STEP_bm      PIN2_bm       ///< ring act STEP pin
#define LA_RING_TIMER        SM_TIMER_RING ///< ring act timer to use
/** @} */

/** \name Linear Actuator Run Settings */
/** @{ */

#define LA_OFFSET 500 ///< Offset when needles are at the top of the ring

#define LA_RING_RETAIN_DEPTH 1350 ///< max distance ring goes to retain peach
#define LA_RING_RETAIN_ACCEL 1000 ///< retaining ring retain accel
#define LA_RING_RETAIN_DECEL 1000 ///< retaining ring retain decel
#define LA_RING_RETAIN_SPEED 2000 ///< retaining ring retain speed
#define LA_RING_RETAIN_OFFSET 10  ///< retaining ring retain depth padding

#define LA_NEEDLE_ENGAGE_SPEED 4000 ///< needle carriage engage speed
#define LA_NEEDLE_ENGAGE_ACCEL 2000 ///< needle carriage engage accel
#define LA_NEEDLE_ENGAGE_DECEL 2000 ///< needle carriage engage decel
#define LA_NEEDLE_ENGAGE_OFFSET 10  ///< needle carriage engage depth padding

#define LA_NEEDLE_CHECK_DEPTH 600      ///< needle carriage check depth
#define LA_NEEDLE_CHECK_SPEED 500      ///< needle carriage check speed
#define LA_NEEDLE_CHECK_ACCEL 500      ///< needle carriage check accel
#define LA_NEEDLE_CHECK_DECEL 6000     ///< needle carriage check decel
#define LA_NEEDLE_CHECK_SPEED_SLOW 100 ///< needle carriage check speed matlab
#define LA_NEEDLE_CHECK_ACCEL_SLOW 50  ///< needle carriage check accel matlab
#define LA_NEEDLE_CHECK_DECEL_SLOW 50  ///< needle carriage check decel matlab
#define LA_NEEDLE_CHECK_OFFSET 10      ///< needle carriage check depth padding

#define BRAKE_PIN_vect PORTR_INT0_vect ///< Brake pin interrupt vector
#define BRAKE_PIN_bm PIN6_bm           ///< Switch 6, translates to PR[0]
/** @} */

/** \name Default Pressure Sensor Thresholds */ /** @{ */
#define PS_ABS_THRESHOLD_DFLT 1100  ///< absolute threshold default
#define PS_DELTA_THRESHOLD_DFLT 600 ///< delta threshold default
/** @} */


/** \name Macro Defined Functions */ /** @{ */
/** \brief Returns the current state of the machine.
 *
 * The valid states of the machine are enumerated in PC_state_t.
 *
 * \param[in] _machine The machine to get the state of
 *
 * \return The current state of the machine, one of the PC_state_t states */
#define get_state(_machine) ((_machine)->state)
/** @} */

/** \name Static Messages */
/** @{ */
/** \brief Help message */
const char *help_message =
"\n\r--------------------------------------------------------------\n\r"
"Pit Crew Peach Pit Detector\n\r\n\r"
"?  - Show help\n\r"
"m<1,2> [dist]  - Move distance    (range: -32K to 32K)\n\r"
"a<1,2> [accel] - Set acceleration (range: 57 to 52K)\n\r"
"d<1,2> [decel] - Set deceleration (range: 57 to 52K)\n\r"
"s<1,2> [speed] - Set speed        (range: 10 to 52K)\n\r"
"move<1,2> [steps] [accel] [decel] [speed] - move with given parameters\n\r"
"c - calibrate pressure sensor\n\r"
"p - read pressure sensor\n\r"
"at - set pressure sensor absolute threshold\n\r"
"dt - set pressure sensor delta threshold\n\r"
"run - perform full system run\n\r"
"runm - perform full system run, print data for matlab script\n\r"
"<cr> - stop the current operation\n\r"
"1 is the needle carriage, 2 is the retaining ring, and + is down\n\r"
"acc/dec data given in 0.001*in/sec^2 (1000 = 1 in/sec^2)\n\r"
"speed data given in 0.001*in/sec (1000 = 1 in/sec)\n\r"
"move data given in 0.001*in (1000 = 1 in)\n\r"
"--------------------------------------------------------------\n\r";

/** \brief Needle Carriage name */
const char *LA_needle_name = "Needle Carriage";

/** \brief Retaining Ring name */
const char *LA_ring_name = "Retaining Ring";
/** @} */

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief The command line instruction types
 *
 * The following commands are recognized by the machine at the terminal
 * interface, and are further documented in the Introduction:
 * + 'm1' - move needle carriage (units 0.001 in, + is towards peach)
 * + 'm2' - move retaining ring (units: 0.001in, + is towards peach)
 * + 'a1' - set needle carriage accel (units: 0.001in/s^2)
 * + 'a2' - set retaining ring accel (units: 0.001in/s^2)
 * + 'd1' - set needle carriage decel (units: 0.001in/s^2)
 * + 'd2' - set retaining ring decel (units: 0.001in/s^2)
 * + 's1' - set needle carriage max speed (units: 0.001in/s)
 * + 's2' - set retaining ring max speed (units: 0.001in/s)
 * + 'move1 [steps] [accel] [decel] [speed]' move needle carriage with given
 * + 'move2 [steps] [accel] [decel] [speed]' move retaining ring with given
 * + 'c' - calibrate pressure sensor, print out calibration data
 * + 'p' - scan sensor, and print out calibration compensated results
 * + 'at' - set pressure sensor absolute threshold
 * + 'dt' - set pressure sensor delta threshold
 * + 'run' - perform a full system run
 * + 'runm' - perform a full system run, print sensor data for matlab
 * + 'h' - home both needle carriage and retaining ring
 * + '?' - print the help message
 * + '\n' - stop the current operation in place (set to idle state)
 *
 * The only commands which are not self explanatory are the 'run' and 'runm'
 * commands. The 'run' command starts a full system run. This includes the
 * automatic retention of the peach by the retaining ring, engaging the needles,
 * checking for pits, disengaging the needles, releasing the peach, and finally
 * passing or rejecting the peach based on whether any pits were found.
 *
 * The 'runm' command similarly performs a full system run. However, at the
 * check stage, the needles are moved much more slowly. This is because while
 * the peach is being checked, the data from the pressure sensor is continuously
 * printed out to the terminal. This allows for the accompanying matlab script
 * to process the values being printed, render each set of pressure sensor data
 * as a surface graph, and play them back as a video. This is simply for show,
 * so the user can see what the pressure sensor "saw" as it checked the peach,
 * but is also useful for debugging purposes when setting threshold values for
 * the system.
 *
 * Additionally, documentation for the purpose of the 'at' and 'dt' commands can
 * be found in the PS_check() function. */
typedef enum command_types {
   STEP1,           ///< Move needle carriage
   STEP2,           ///< Move the retaining ring
   ACCEL1,          ///< Set needle carriage acceleration
   ACCEL2,          ///< Set retaining ring acceleration
   DECEL1,          ///< Set needle carriage deceleration
   DECEL2,          ///< Set retaining ring deceleration
   SPEED1,          ///< Set needle carriage max speed
   SPEED2,          ///< Set retaining ring max speed
   MOVE1,           ///< Move needle carriage with given speed profile
   MOVE2,           ///< Move retaining ring with given speed profile
   CAL,             ///< calibrate the pressure sensor
   SCAN,            ///< scan the sensor
   ABS_THRESHOLD,   ///< set the absolute threshold
   DELTA_THRESHOLD, ///< set the delta threshold
   RUN,             ///< run the full system, print out sensor readings
   RUN_MATLAB,      ///< run for matlab
   HOME,            ///< reset both motors to home
   BRAKE,           ///< stop both motors
   HELP,            ///< Show help menu
   NONE             ///< No command retrieved
} command_t;

/** \brief The different states for the machine.
 *
 * The system is modeled as a finite state machine. These are the states the
 * machine can be in. The definition of each state is elaborated in this file's
 * documentation. */
typedef enum pit_crew_state {
   IDLE,      ///< Idling (home state)
   RETAIN,    ///< Retaining Ring descending
   ENGAGE,    ///< Needles approaching peach
   CHECK,     ///< Checking for pits
   DISENGAGE, ///< Needles retracting
   RELEASE,   ///< Retaining ring releasing peach
   PASS,      ///< Tell the user the result
   STOP       ///< Lid is removed
} PC_state_t;

/** \brief Pit Crew machine.
 *
 * Contains the static data for all of the various hardware the board must
 * interface with, as well as the current state of the machine */
typedef struct pit_crew_machine  {
   PC_state_t state;     ///< the current state of the system
   LA_t needle_carriage; ///< the system's needle carriage
   LA_t retaining_ring;  ///< the system's retaining ring
   PS_t pressure_sensor; ///< the system's pressure sensor
} PC_t;

#endif /* end of include guard: _TEST_LIMIT_SWITCH_H_ */

