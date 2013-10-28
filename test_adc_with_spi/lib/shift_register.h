/** \file shift_register.h
 * \brief Shift Register driver.
 *
 * Simplifies setting up and communicating with the 74HC595 shift register.
 * SR_init() must be called before any of the other functions in this library.
 * SR_send_byte() performs a single byte transfer, but does not block while the
 * byte is being transferred. Additionally, SR_send_byte() does not perform any
 * bus arbitration, so it is up to the user to ensure that the bus is free, and
 * that any timing requirements are not broken
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
 * limitations under the License.\endverbatim */

/* Include Directives *********************************************************/
#include "spi_driver.h"

/* Library Definitions ********************************************************/

/** \brief Contains the static data needed to communicate with the shift
 * register.
 *
 * This struct is used as the first parameter to all of the SR_* functions in
 * this library */
typedef struct shift_register_struct  {
   PORT_t *port;             ///< Shift Register port
   SPI_Master_t *SPI_master; ///< SPI to use, allocated in SR_init()
} SR_t;

#define SPI_SS_PINCTRL PIN4CTRL ///< PINCTRL register for /SS PIN

/* Function Prototypes ********************************************************/

void SR_init(SR_t *shift_reg, PORT_t *port, SPI_t *module, bool lsb_first);
void SR_send_byte(SR_t *shift_reg, uint8_t byte);
