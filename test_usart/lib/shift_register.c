/** \file shift_register.c
 *
 * \brief Shift Register driver implementation, see shift_register.h for full
 * documentation.
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

/* Include Directives *********************************************************/
#include "shift_register.h"

/* Function Definitions *******************************************************/

/** \brief Initializes the shift register.
 *
 * All library functions in this library take the shift_register given here as
 * their first parameter. This function initializes it for future communication
 * with the shift register. This library uses the SPI library to communicate
 * with the shift register, set up in single byte transfer, non-interrupt mode
 * at 15MHz with no bus arbitration (only 1 device is allowed on SPI bus).
 *
 * \note The shift register's L_CLOCK pin must be tied to the SPI /SS pin on the
 * port passed as a parameter to this function. L_CLOCK latches the output
 * buffer when clocked high.
 *
 * \param[out] shift_reg the shift register to initialize
 * \param[in] port the port to use to communicate with the shift register
 * \param[in] module the SPI module to use  (one of SPIC..SPIF)
 * \param[in] lsb_first true for transmission order lsb->msb */
void SR_init(SR_t *shift_reg, PORT_t *port, SPI_t *module, bool lsb_first) {

   /* initialize shift_reg fields */
   shift_reg->port = port;
   shift_reg->SPI_master = malloc(sizeof(SPI_Master_t));

   /* Init SS (L_CLOCK) pin as output with push/pull configuration. L_CLOCK
    * latches the output buffer when clocked high, so the default state for
    * the pin is hi. */
   port->DIRSET = SPI_SS_bm;
   port->SPI_SS_PINCTRL = PORT_OPC_TOTEM_gc;
   port->OUTSET = SPI_SS_bm;

   /* Initialize the MCU as an SPI master on the given port. */
   SPI_MasterInit(shift_reg->SPI_master, // SPI_master_t struct to use
                  module,                   // SPI module to use
                  port,                     // SPI port to use
                  lsb_first,                // lsb_first = false
                  SPI_MODE_0_gc,            // data on leading rising edge
                  SPI_INTLVL_OFF_gc,        // SPI interrupt priority = off
                  true,                     // clock2x = true
                  SPI_PRESCALER_DIV4_gc);   // division scaler (SCK=15MHz)
}

/** \brief sends the given byte to the shift register.
 *
 * In the pit crew application, the shift register is used to select the
 * channels on each of the multiplexers. However, the order of the channel
 * select bits is left up to the user of this library, ie. the user is
 * responsible for tracking and creating the channel select masks.
 *
 *
 * \param[in] shift_reg shift register to communicate with \sa{ SR_init() }
 * \param[in] byte the byte to transmit
 *
 * \note lsb first or msb first operation is set during SR_init(). */
void SR_send_byte(SR_t *shift_reg, uint8_t byte) {
   /* clearing /SS (L_CLOCK) pin before the transfer maintains comptability
    * with the SPI protocol, although it is not necessary to use the shift
    * register, which latches its ouput buffer when the pin is driven high */
   shift_reg->port->OUTCLR = SPI_SS_bm;
   SPI_MasterTransceiveByte(shift_reg->SPI_master, byte);
   shift_reg->port->OUTSET = SPI_SS_bm;
   return;
}
