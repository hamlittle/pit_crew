/** \file test_ext_boards.h
 *
 * \todo brief description
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

#ifndef _TEST_USART_H_
#define _TEST_USART_H_

/* Include Directives *********************************************************/

#include "board.h"
#include "avr_compiler.h"
#include "clksys_driver.h"
#include "shift_register.h"
#include "adc.h"
#include "usart_bc.h"

/* Macro Definitions **********************************************************/

#define LINES_PER_MP 4 ///< 16 channels per mp [0..15] = 4 bits to select
#define NUM_MP       2 ///< Number of Multiplexers used
#define NUM_SR       1 ///< Number of Shift Registers used

/** \name Wiring Specific Definitions */ ///@{
#define SPI_PORT     PORTF         ///< SPI bus port
#define SPI_MODULE   SPIF          ///< SPI bus module
#define SPI_SS0_bm   PIN2_bm       ///< SPI SS0 pin
#define SPI_SS1_bm   PIN3_bm       ///< SPI SS1 pin
#define SPI_INT_VECT SPIF_INT_vect ///< SPI bus interrupt vector

#define ADC0_CTRL_PORT    PORTD           ///< ADC0 control port
#define ADC0_CONVST_bm    PIN1_bm         ///< /CONVST pin (pull low to start)
#define ADC0_EOC_bm       PIN3_bm         ///< /EOC pin (pulled low when ready)
#define ADC0_EOC_INT_VECT PORTD_INT0_vect ///< ADC0 /EOC interrupt vector

#define ADC1_CTRL_PORT    PORTF           ///< ADC1 control port
#define ADC1_CONVST_bm    PIN1_bm         ///< /CONVST pin (pull low to start)
#define ADC1_EOC_bm       PIN0_bm         ///< /EOC pin (pulled low when ready)
#define ADC1_EOC_INT_VECT PORTF_INT0_vect ///< ADC1 /EOC interrupt vector
///@}

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief The two ADC's being used */
typedef enum ADC_selection {
   ADC0, ///< ADC0 is connected to MPx0
   ADC1  ///< ADC1 is connected to MPx1
} ADC_sel_t;

/** \brief The two multiplexers being used. */
typedef enum multiplexer_select {
   MPx, ///< X-Position Multiplexer,
   MPy  ///< Y-Position Multiplexer
} mp_select_t;

/* Function Prototypes ********************************************************/

/** \name Board Setup Functions */
///@{
void setup_clocks(void);
void setup_LEDs(void);
void setup_switches(uint8_t switch_mask);
void setup_ADC(void);
void setup_USART_BC(void);
///@}

void show_result(uint8_t result);

#endif /* end of include guard: _TEST_USART_H_ */

