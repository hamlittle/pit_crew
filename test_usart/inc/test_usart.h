/** \file test_adc_with_spi.h
 *
 * \brief Tests the ADC, verifying we can communicate with it, and its
 * conversion results are reasonable, using the shift_register and adc
 * libraries.
 *
 * ### Test Functionality
 * Channel 15 of MPy is routed to the VINN1 pin of the ADC. ADC conversions are
 * first performed continuosly, and the upper 8 bits of the result are mapped to
 * the 8 LEDs onboard. This lasts for 5 (actually 37) seconds, then there is
 * a 5 second delay when the LEDs are turned off. Next, single conversion mode
 * is used to get conversion results, and again the upper 8 bits of the result
 * are mapped to the LEDs. Then, there is another 5 second delay, during which
 * the LEDs are turn off, and the loop repeats.
 *
 * \note The board wiring, including a voltage divider network, is documented in
 * the logbook.
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

#ifndef _TEST_SHIFT_REGISTER_H_
#define _TEST_SHIFT_REGISTER_H_

/* Include Directives *********************************************************/

#include <stdio.h>

#include "board.h"
#include "avr_compiler.h"
#include "clksys_driver.h"
#include "shift_register.h"
#include "adc.h"
#include "usart_driver.h"

/** \name Wiring Specific Definitions *****************************************/
///@{

#define SR_PORT       PORTF ///< Shift Register Port
#define SR_SPI_MODULE SPIF  ///< Shift Register SPI module to use (PORTF)

#define ADC_PORT         PORTC           ///< ADC port
#define ADC_CONVST_bm    PIN0_bm         ///< /CONVST pin (pull low to start)
#define ADC_EOC_bm       PIN1_bm         ///< /EOC pin (pulled low when ready)
#define ADC_SPI_MODULE   SPIC            ///< SPI module to use (PORTC)
#define ADC_SPI_INT_vect SPIC_INT_vect   ///< SPI interrupt vector
#define ADC_EOC_INT_VECT PORTC_INT0_vect ///< ADC /EOC interrupt vector

///@}

/* Static Definitions *********************************************************/
#define LINES_PER_MP 4 ///< 16 channels per mp [0..15] = 4 bits to select
#define NUM_MP       2 ///< Number of Multiplexers used
#define NUM_SR       1 ///< Number of Shift Registers used

/** \brief The two multiplexers being used. */
typedef enum multiplexer_select {
   MPx, ///< X-Position Multiplexer,
   MPy  ///< Y-Position Multiplexer
} mp_select_t;

/** \name Board Setup Functions ***********************************************/
///@{
void setup_clocks(void);
void setup_LEDs(void);
void setup_switches(uint8_t switch_mask);
void setup_ADC(PORT_t *port, SPI_t *module, uint8_t CONVST_bm,
               uint8_t EOC_bm, ADC_callback_t callback);
void setup_SR(SR_t *shift_reg, PORT_t *port, SPI_t *module, bool lsb_first);
///@}

void set_channel(mp_select_t mp_select, uint8_t channel, SR_t *shift_reg);
void show_result(uint8_t result);
void ADC_callback(uint16_t result);

#endif /* end of include guard: _TEST_SHIFT_REGISTER_H_ */

