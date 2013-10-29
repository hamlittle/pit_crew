/** \file test_adc_with_spi.c
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

/* Include Directives *********************************************************/

#include "test_usart.h"


/* Global Definitions *********************************************************/

/** \brief The ADC being used by the adc library */
ADC_ext_t adc;

static int uart_putchar(char c, FILE *stream);
static void uart_init (void);

static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL,
                                          _FDEV_SETUP_WRITE);

static int uart_putchar (char c, FILE *stream)
{
   if (c == '\n')
      uart_putchar('\r', stream);

   // Wait for the transmit buffer to be empty
   while ( !( USARTC0.STATUS & USART_DREIF_bm) );

   // Put our character into the transmit buffer
   USARTC0.DATA = c;

   return 0;
}


// Init USART.  Transmit only (we're not receiving anything)
// We use USARTC0, transmit pin on PC3.
// Want 9600 baud. Have a 2 MHz clock. BSCALE = 0
// BSEL = ( 2000000 / (2^0 * 16*9600)) -1 = 12
// Fbaud = 2000000 / (2^0 * 16 * (12+1))  = 9615 bits/sec
static void uart_init (void)
{
   // Set the TxD pin high - set PORTC DIR register bit 3 to 1
   PORTC.OUTSET = PIN3_bm;

   // Set the TxD pin as an output - set PORTC OUT register bit 3 to 1
   PORTC.DIRSET = PIN3_bm;

   // Set baud rate & frame format
   USART_Format_Set(&USARTC0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc,
                    false);
   USART_Baudrate_Set(&USARTC0, (uint16_t)12, (uint8_t)4);
   USART_SetMode(&USARTC0, USART_CMODE_ASYNCHRONOUS_gc);
   USART_Tx_Enable(&USARTC0);

   // Set mode of operation
   /* USARTC0.CTRLA = 0; // no interrupts please */
   /* USARTC0.CTRLC = 0x03; // async, no parity, 8 bit data, 1 stop bit */

   // Enable transmitter only
   /* USARTC0.CTRLB = USART_TXEN_bm; */
}
/* function definitions *******************************************************/

/** \name Board Setup Functions ***********************************************/
///@{

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

/** \brief Sets up the Shift Register.
 *
 * Shift register is wired to PORTF. /SS pin is wired to L_CLOCK on SPIF (clock
 * hi to latch output buffer of Shift Register).  This function initializes the
 * SR_t struct needed for the shift_register library, which handles
 * communicating with the shift register in a transparent way.
 *
 * \param[out] shift_reg the shift register to initialize
 * \param[in] port the port to use to communicate with the shift register
 * \param[in] module the SPI module to use  (one of SPIC..SPIF)
 * \param[in] lsb_first true for transmission order lsb->msb */
void setup_SR(SR_t *shift_reg, PORT_t *port, SPI_t *module, bool lsb_first) {
   SR_init(shift_reg, port, module, lsb_first);
}

/** \brief Sets up the ADC
 *
 * ADC is wired to PORTC on the board. /EOC is PC[1], and /CONVST is PC[0].
 * This function initializes the ADC_ext_t struct needed for the ADC library,
 * which handles conversion timing and conversion result communication.
 *
 * \note The adc being initialized is a global variable, as this must be
 * accessed from the ISRs set up to coordinate the communication witht the ADC,
 * as documented in the adc library.
 * \sa adc.h
 *
 * \param[in] port the port the ADC is wired to
 * \param[in] module the SPI module to use to communicate with the ADC
 * \param[in] CONVST_bm bitmask for the /CONVST pin on the given port
 * \param[in] EOC_bm bitmask for the /EOC pin on the given port
 * \param[in] callback ADC continuous mode callback function to register */
void setup_ADC(PORT_t *port, SPI_t *module, uint8_t CONVST_bm, uint8_t EOC_bm,
               ADC_callback_t callback) {
   ADC_init(&adc, port, module, CONVST_bm, EOC_bm);
   ADC_register_continuous_callback(&adc, callback);
}

///@}

/** \brief Selects the multiplexer channel.
 *
 * Convenience wrapper around SR_send_byte() which tracks the channel selected
 * on the x and y multiplexers.
 *
 * \param[in] mp_select which multiplexer to set channel
 * \param[in] channel the channel to select
 * \param[in] shift_reg the shift register used to select MP channels */
void set_channel(mp_select_t mp_select, uint8_t channel, SR_t *shift_reg) {
   static uint8_t x_channel = 0;
   static uint8_t y_channel = 0;
   uint8_t combined_channel_select;

   if (mp_select == MPx) {
      x_channel = channel;
   }
   else {
      y_channel = channel;
   }
   combined_channel_select = ((y_channel & 0x0F) << 4) | (x_channel & 0x0F);

   SR_send_byte(shift_reg, combined_channel_select);
}

/** \brief Shows upper 8 bits of ADC result on LEDS[0..7]
 *
 * \param[in] result the upper 8 bits of the ADC result */
void show_result(uint8_t result) {
   LED_PORT.OUT = result;
}

/** \brief ADC callback function.
 *
 * Called by the adc library whenever a conversion completes in continuous
 * mode. Here, we show the upper 8 bits of the conversion result on the LEDs.
 *
 * \param[in] result the latest conversion result */
void ADC_callback(uint16_t result) {
   if (PORTD.IN & PIN4_bm) {
      LED_PORT.OUT = 0xFF;
   }
   else {
      LED_PORT.OUT = (uint8_t)(result >> 4);
   }
}

/** \brief main loop to run tests.
 *
 * The tests being run are described in the documentation of this file.
 *
 * \return never returns, test loop runs ad infinitum. */
int main(void) {
   uint8_t switch_mask = 0x00; // Input switches. Read hi when pressed
   uint16_t result = 0;
   float result_f = 0.0;
   uint16_t counter;
   SR_t shift_reg;

   /* call all of the setup_* functions */
   cli();
   setup_clocks();
   setup_LEDs();
   setup_switches(switch_mask);
   setup_SR(&shift_reg, &SR_PORT, &SR_SPI_MODULE, false);
   setup_ADC(&ADC_PORT, &ADC_SPI_MODULE, ADC_CONVST_bm, ADC_EOC_bm,
             ADC_callback);
   uart_init();
   sei();

   set_channel(MPx, 15, &shift_reg);
   set_channel(MPy, 15, &shift_reg);

   stdout = &mystdout;

   /* signal debugging */
   PORTD.DIRCLR |= PIN4_bm;
   PORTD.PIN4CTRL = PORT_OPC_PULLDOWN_gc;

   while (1)
   {
      if (ADC_ready(&adc)) {
         // get a new conversion result
         result = ADC_sample_once(&adc);
      }

      if (PORTD.IN & PIN4_bm) {
         LED_PORT.OUT = 0xFF;
      }
      else {
         LED_PORT.OUT = (uint8_t)(result >> 4);
         result_f = result * 5.0 / 2048;
         printf("%u ", result);
      }
   }

}

/** \brief ADC /EOC interrupt vector (PORT.INT0)
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system.  */
ISR(ADC_EOC_INT_VECT) {
   ADC_EOC_interrupt_handler(&adc);
}

/** \brief ADC SPI interrupt
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system. */
ISR(ADC_SPI_INT_vect) {
   ADC_SPI_interrupt_handler(&adc);
}
