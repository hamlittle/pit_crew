/** \file test_pressure_sensor.c
 *
 * \brief System software for testing the pressure sensor and finding the
 * threshold values.
 *
 * Pressing switch 4 scans every channel and sets the compensation values. This
 * compensates for the fact that in the home position, some of the needles will
 * be applying a pressure to the sensor. This compensation value is subtracted
 * from the appropriate ADC readings before any analysis on them is performed.
 *
 * Pressing switch 5 initiates a scan of all the needles, and prints out the
 * most useful information about them. The printed information can be captured
 * using a terminal window set to read from the USB port the development board
 * is hooked up to. The information printed is:
 *    - Number of sensors showing a reading
 *    - Highest reading (range 0-2048)
 *    - Lowest reading (range 0-2048)
 *    - Average reading (range 0-2048)
 *
 * \note In order to change the channel on MPy (the second board), ADC0 must be
 * read first, so that when reading from ADC1, SR1 is updated. If we read from
 * ADC1 first, it would change the channel on MPy before ADC0 could get
 * a reading. The vice versa is also true: when we want to change the MPx
 * channel selected, we must read from ADC1 first, then ADC0, because the SPI
 * transfer during which we are updating SR0 occurs when we read ADC0.
 *
 * \note PD[1,3] are being used by ADC0, so these switches are not available to
 * be used on the development board. However, any of the other switches are fair
 * game.
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

#include "test_pressure_sensor.h"


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
   SWITCH_PORTL.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm;

   SWITCH_PORTH.DIR &= ~maskH;
   PORTCFG.MPCMASK = maskH;
   SWITCH_PORTL.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm;
}

/** \brief Initializes the pressure sensor library.
 *
 * This allows for easy interfacing with the sensor through the PS_* library
 * calls.
 *
 * \param[in] pressure_sensor The pressure sensor to initialize */
void setup_pressure_sensor(PS_t *pressure_sensor) {
   PS_init(pressure_sensor);
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

/** \brief main loop to run tests.
 *
 * The tests being run are described in the documentation of this file.
 *
 * \return never returns, test loop runs ad infinitum. */
int main(void) {
   uint8_t switch_mask = COMPENSATION_SWITCH_bm | SCAN_SWITCH_bm;
   bool compensation_switch_pushed = false;
   bool scan_switch_pushed = false;

   PS_t pressure_sensor;

   /* call all of the setup_* functions */
   cli();
   setup_clocks();
   setup_LEDs();
   setup_switches(switch_mask);
   setup_pressure_sensor(&pressure_sensor);
   setup_USART_BC();
   sei();

   /* signal debugging */
   PORTC.DIRCLR = PIN6_bm;
   PORTC.PIN6CTRL = PORT_OPC_PULLDOWN_gc;

   while (1) {

      if (PORTC.IN & PIN6_bm) {
         LED_PORT.OUT = 0xff;
      }
      else {
         LED_PORT.OUT = 0x00;
      }

      if ((SWITCH_PORTL.IN & COMPENSATION_SWITCH_bm)
          && !compensation_switch_pushed) {
         compensation_switch_pushed = true;

         LED_PORT.OUT = 0xff;
         PS_calibrate(&pressure_sensor);
         LED_PORT.OUT = 0x00;
         PS_print_compensation_buffer(&pressure_sensor);
      }
      else if (!(SWITCH_PORTL.IN & COMPENSATION_SWITCH_bm)) {
         compensation_switch_pushed = false;
      }

      if ((SWITCH_PORTL.IN & SCAN_SWITCH_bm)
          && !scan_switch_pushed) {
         scan_switch_pushed = true;

         LED_PORT.OUT = 0xff;
         PS_scan_all(&pressure_sensor);
         LED_PORT.OUT = 0x00;
         PS_print_scan_buffer(&pressure_sensor);
      }
      else if (!(SWITCH_PORTL.IN & SCAN_SWITCH_bm)) {
         scan_switch_pushed = false;
      }
   }
}
