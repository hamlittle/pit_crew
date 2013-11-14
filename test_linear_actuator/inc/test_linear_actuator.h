/** \file test_linear_actuator.h
 *
 * \brief System software for testing the pressure sensor and finding the
 * threshold values.
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

/* Macro Definitions **********************************************************/

#define COMPENSATION_SWITCH_bm PIN4_bm ///< Start compensation scan switch
#define SCAN_SWITCH_bm PIN5_bm         ///< Start full sensor scan switch

/* Typedefs, Enums, and Structs ***********************************************/

/* Function Prototypes ********************************************************/

/** \name Board Setup Functions */
///@{
void setup_clocks(void);
void setup_LEDs(void);
void setup_switches(uint8_t switch_mask);
void setup_pressure_sensor(PS_t *pressure_sensor);
void setup_USART_BC(void);
///@}

#endif /* end of include guard: _TEST_LINEAR_ACTUATOR_H_ */

