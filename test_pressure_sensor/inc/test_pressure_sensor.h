/** \file test_pressure_sensor.h
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

#define NUM_MPX_CHANS 28 ///< Number of x-position channels
#define NUM_MPY_CHANS 24 ///< Number of y-position channels

#define COMPENSATION_SWITCH_bm PIN4_bm
#define SCAN_SWITCH_bm PIN5_bm

/** \brief The channels which can be selected on the x position multiplexer.
 *
 * MPx0 and MPx1 follow eachother, in that they will always be selecting the
 * same channel. This is just for convenience, and is not required by the
 * system.
 *
 * \note Because the ADC conversion result is 2 bytes, two bytes must be
 * transferred out on the SPI bus to control the MPx channel selected. In this
 * case, the first value sent out will be shifted out, and the second byte sent
 * will become the new channel selected. Therefore, each of the channel select
 * values are offset by from their index in this array by 1. This allows the
 * user to use MPx_channel[0] to select channel 0, as 0x00 is the second byte
 * transferred. The same goes for MPx_channel[1] to select channel 1, as 0x11
 * is the second byte transferred, and so on, for every channel on MPx0 and
 * MPx1. */
const uint8_t MPx_channels[(NUM_MPX_CHANS / 2) + 1] =
{ 0x00,
   0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
   0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD };

/** \brief The channels which can be selected on the y position multiplexer.
 *
 * Unlike MPx, MPy0 and MPy1 are independant, and only 1 should be turned on at
 * any given time. in order to facilitate this, When the channel being selected
 * is in the range of MPy0, MPy1 is set to channel 15, which is disconnected,
 * and vice versa.
 *
 * \note Because the ADC conversion result is 2 bytes, two bytes must be
 * transferred out on the SPI bus to control the MPy channel selected. In this
 * case, the first value sent out will be shifted out, and the second byte sent
 * will become the new channel selected. Therefore, each of the channel select
 * values are offset by from their index in this array by 1. This allows the
 * user to use MPx_channel[0] to select channel 0, as 0x00 is the second byte
 * transferred. The same goes for MPx_channel[1] to select channel 1, as 0x11
 * is the second byte transferred, and so on, for every channel on MPy0 and
 * MPy1. */
const uint8_t MPy_channels[NUM_MPY_CHANS + 1] =
{ 0x00,
   0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB,
   0x0F, 0x1F, 0x2F, 0x3F, 0x4F, 0x5F, 0x6F, 0x7F, 0x8F, 0x9F, 0xAF, 0xBF };

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

void scan_all(uint16_t readings[NUM_MPY_CHANS][NUM_MPX_CHANS]);
void print_results(uint16_t readings[NUM_MPY_CHANS][NUM_MPX_CHANS],
                   uint16_t compensations[NUM_MPY_CHANS][NUM_MPX_CHANS]);

#endif /* end of include guard: _TEST_USART_H_ */

