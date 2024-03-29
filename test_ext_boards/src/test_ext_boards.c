/** \file test_ext_boards.c
 *
 * \brief tests that the two boards developed for the project fucntion
 * correctly.
 *
 * Sets MPx and MPy to known channels, to verify we can set them correctly.
 * Continuously initiated single sample mode conversions on ADC0 and ADC1,
 * printing the result over the USART-USB gateway so the result can be confirmed
 * to be accurate.
 *
 * This verifies that we are able to correctly specifiy a channel, start
 * a conversion, and get the result.
 *
 * \note In order to change the channel on MPy (the second board), ADC0 must be
 * read first, so that when reading from ADC1, SR1 is updated. If we read from
 * ADC1 first, it would change the channel on MPy before ADC0 could get
 * a reading. The vice versa is also true: when we want to change the MPx
 * channel selected, we must read from ADC1 first, then ADC0, because the SPI
 * transfer during which we are updating SR0 occurs when we read ADC0.
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

#include "test_ext_boards.h"

/* Global Variables ***********************************************************/
/** \brief ADC0 connected to MPx0*/
ADC_ext_t adc0;

/** \brief ADC1 connected to MPx1*/
ADC_ext_t adc1;

/** \brief the ADC which is currently undergoing an SPI transfer to receive the
 * conversion result */
volatile ADC_sel_t current_ADC;

/* Function Definitions *******************************************************/

/** \brief Initialize and set cpu and periheral clocks.
 *
 * CPU clock frequencies set are:
 * -CPU: 32HMZ
 * -Peripheral Prescaling: NONE */
void setup_clocks(void) {

   // set 32MHZ oscillator as CPU clock source
   CLKSYS_Enable(OSC_RC32MEN_bm);                          // enable
   do { nop(); } while (!CLKSYS_IsReady(OSC_RC32MRDY_bm)); // wait til stable
   CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);   // select for CPU

   // disable all presacalers, until we decide otherwise
   CLKSYS_Prescalers_Config(CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);

   // set up external 32KHz oscillator (NOTE: first param is ignored)
   CLKSYS_XOSC_Config(OSC_FRQRANGE_04TO2_gc, false, OSC_XOSCSEL_32KHz_gc);

   // set internal 32KHz oscillator as source for DFLL and autocalibrate 32MHz
   CLKSYS_Enable(OSC_XOSCEN_bm);                          //enable
   do { nop(); } while (!CLKSYS_IsReady(OSC_XOSCRDY_bm)); // wait til stable
   CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, true); // true == ext 32KHz

   // disable unused oscillators (internal 2MHz and 32KHz oscillators)
   CLKSYS_Disable(OSC_RC2MEN_bm | OSC_RC32KEN_bm);
}

/** \brief Sets up the LEDS.
 *
 * Sets the pins on LED_PORT to inverted output, so setting the corresponding
 * pin will turn on the LED. Clear the corresponding pin to turn the given LED
 * off. */
void setup_LEDs(void) {
   LED_PORT.DIR = 0xff;                 // set all pins of port E to output
   PORTCFG.MPCMASK = 0xff;             // set for all pins on port E...
   LED_PORT.PIN0CTRL |= PORT_INVEN_bm;  // inverted output (set hi turns on led)
   LED_PORT.OUTCLR = 0xff;              // turn all leds off
}

/** \brief Sets up the switches.
 *
 * Switches on the board are normally open, and pull to ground when pushed.
 * Inverted input is enabled, so the IN register will read high if the switch is
 * pushed.
 *
 * \param[in] switch_mask which switches to enable. */
void setup_switches(uint8_t switch_mask) {
   uint8_t maskL = switch_mask & SWITCH_PORTL_MASK_gm;
   uint8_t maskH = switch_mask >> SWITCH_PORTH_OFFSET;

   SWITCH_PORTL.DIR &= ~maskL;
   PORTCFG.MPCMASK = maskL;
   SWITCH_PORTL.PIN0CTRL = PORT_OPC_PULLDOWN_gc | PORT_INVEN_bm;

   SWITCH_PORTH.DIR &= ~maskH;
   PORTCFG.MPCMASK = maskH;
   SWITCH_PORTL.PIN0CTRL = PORT_OPC_PULLDOWN_gc | PORT_INVEN_bm;
}

/** \brief Sets up the ADC.
 *
 * The values used to set up the ADC are set in test_ext_boards.h as macro
 * defined variables.
 *
 * \note The adc being initialized is a global variable, as this must be
 * accessed from the ISRs set up to coordinate the communication with the ADC,
 * as documented in the adc library.
 * \sa adc.h */
void setup_ADC() {
   ADC_init(&adc0, &ADC0_CTRL_PORT, ADC0_CONVST_bm, ADC0_EOC_bm,
            &SPI_PORT, &SPI_MODULE, SPI_SS0_bm);
   ADC_init(&adc1, &ADC1_CTRL_PORT, ADC1_CONVST_bm, ADC1_EOC_bm,
            &SPI_PORT, &SPI_MODULE, SPI_SS1_bm);
}

/** \brief Sets up the USART connection with the Board Controller.
 *
 * Redirects stdout to the UART channel connected to the Board Controller, so
 * that printf() calls will be transferred over this connection. The Board
 * Controller acts as a UART-USB bridge by default, so any transmissions on
 * this channel are translated and sent over the USB connection. This allows
 * any host machine connected to the USB line on the board to monitor these
 * transmissions using a Serial Port terminal program, such as RealTerm.
 *
 * This set up is performed by the usart_bc.h library */
void setup_USART_BC(void) {
   USART_BC_init();
}

/** \brief Shows upper 8 bits of ADC result on LEDS[0..7]
 *
 * \param[in] result the upper 8 bits of the ADC result */
void show_result(uint8_t result) {
   LED_PORT.OUT = result;
}

/** \brief main loop to run tests.
 *
 * The tests being run are described in the documentation of this file.
 *
 * \return never returns, test loop runs ad infinitum. */
int main(void) {
   uint8_t switch_mask = 0x00; // Input switches. Read hi when pressed
   uint16_t result = 0;
   uint16_t delay_counter = 0;

   const uint8_t MPx_channels[2] = { 0x00, 0x55 };
   const uint8_t MPy_channels[2] = { 0x00, 0x55 };

   /* see if this fixes things */
   PORTF.DIRSET = PIN4_bm;
   PORTF.OUTSET = PIN4_bm;

   /* call all of the setup_* functions */
   cli();
   setup_clocks();
   setup_LEDs();
   setup_switches(switch_mask);
   setup_ADC();
   setup_USART_BC();
   sei();

   /* signal debugging */
   PORTC.DIRCLR = PIN6_bm;
   PORTC.PIN6CTRL = PORT_OPC_PULLDOWN_gc;


   while (1) {

      ADC_set_output_data(&adc0, MPx_channels);
      result = ADC_sample_once(&adc0);
      if (PORTC.IN & PIN6_bm) {
         LED_PORT.OUT = 0xff;
      }
      else {
         LED_PORT.OUT = (uint8_t)(result>>4);
         printf("ADC0: %u\n", result);
         for (delay_counter = 0; delay_counter < 500; ++delay_counter) {
            delay_ms(1);
         }
      }

      ADC_set_output_data(&adc1, MPy_channels);
      result = ADC_sample_once(&adc1);
      if (PORTC.IN & PIN6_bm) {
         LED_PORT.OUT = 0xff;
      }
      else {
         LED_PORT.OUT = (uint8_t)(result>>4);
         printf("ADC1: %u\n", result);
         for (delay_counter = 0; delay_counter < 500; ++delay_counter) {
            delay_ms(1);
         }
      }
   }
}

/** \brief ADC /EOC interrupt vector (PORT.INT0)
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system.  */
ISR(ADC0_EOC_INT_VECT) {
   current_ADC = ADC0;
   ADC_EOC_interrupt_handler(&adc0);
}

/** \brief ADC /EOC interrupt vector (PORT.INT0)
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system.  */
ISR(ADC1_EOC_INT_VECT) {
   current_ADC = ADC1;
   ADC_EOC_interrupt_handler(&adc1);
}

/** \brief ADC SPI interrupt
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system. */
ISR(SPI_INT_VECT) {
   if (current_ADC == ADC0) {
      ADC_SPI_interrupt_handler(&adc0);
   }
   else {
      ADC_SPI_interrupt_handler(&adc1);
   }
}
