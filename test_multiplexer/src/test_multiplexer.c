/** \file test_multiplexer.c
 *
 *
 * \brief Tests the multiplexer input switching
 *
 * functionality Use switch 1 to cycle through the various multiplexer inputs.
 * LEDS[0..3] give the binary encoding of the selected channel on the
 * multiplexer. When a +3.3v signal is present on the selected channel,
 * LEDS[4..7] will be lit. Do not use a +5v source, the xmega is not 5v
 * tolerant. S[0..3] go to GPIO[0..3], and SIG goes to GPIO7. Multiplexer can
 * be powered by either +3.3v or +5v.
 *
 * \author Hamilton Little - hamilton.little@gmail.com
 * Developed for the Pit Crew Team Senior Project, Cal Poly Fall 2013 Code used
 * in this project draws from the example projects for the XMEGAA1 board and
 * the drivers for the XMEGA series, distributed by the Atmel Corporation. The
 * author would like to thank Atmel for their work in developing and supporting
 * the XMEGA chip series, as well as for the sample code and tutorials they
 * provide for the chip */

/**** include directives ******************************************************/

/* contains some macros to make the code more similar for both compilers. Look
 * into the file for more details. Includes <avr/io.h>, which will pull correct
 * atmel header so long as -mmcu is defined on the compile line. */
#include "avr_compiler.h"

/* The board.h header file defines which IO ports peripherals like Switches and
 * LEDs are connected to. The header file is configured for use with XMEGA-A1
 * Xplained by default. */
#include "board.h"

/**** function prototypes *****************************************************/

void setup_channel_select_pins(void);
void setup_leds(void);
void setup_switches(void);
void setup_signal_pin(void);
void setup(void);
void show_channel(int8_t channel);
void set_channel(int8_t channel);
void show_signal(uint8_t signal_present);

/**** function definitions ****************************************************/

/** \brief Sets up the channel select pins.
 *
 * GPIO pins [0..3] on header J3 are used to select multiplexer channel. */
void setup_channel_select_pins(void) {
   CHANNEL_PORT.DIR |= CHANNEL_PINS_bm;   // set channel select pins to out
   CHANNEL_PORT.OUTCLR = CHANNEL_PINS_bm; // set to channel 0 initially
}

/** \brief Sets up the LEDS.
 *
 * LEDS[0..3] read out the binary value of the current channel selection */
void setup_leds(void) {
   LEDPORT.DIR = 0xff;                 // set all pins of port E to output
   PORTCFG.MPCMASK = 0xff;             // set for all pins on port E...
   LEDPORT.PIN0CTRL |= PORT_INVEN_bm;  // inverted output (set hi turns on led)
   LEDPORT.OUTCLR = 0xff;              // turn all leds off
}

/** \brief Sets up the switches.
 *
 * Switch 7 cycles through multiplexer chanels 0..15 */
void setup_switches(void) {
   SWITCHPORTH.DIR &= ~CYCLE_SWITCH_bm;   // set the switch 7's pin as input
   SWITCHPORTH.PIN1CTRL |= PORT_OPC_PULLUP_gc;
}

/** \brief Sets up the signal reading pin
 *
 * GPIO pin 4 on header J3 is used to read the value on the signal pin of the
 * multiplexer */
void setup_signal_pin(void) {
   SIGNAL_PORT.DIR &= ~SIGNAL_PIN_bm; // set the signal pin as an input
   SIGNAL_PORT.PIN4CTRL |= PORT_OPC_PULLDOWN_gc;
}

/** \brief Calls all the setup_* functions.
 *
 * should be called from main to set up the board. */
void setup(void) {
   setup_channel_select_pins();
   setup_leds();
   setup_switches();
   setup_signal_pin();
}

/** \brief Sets LEDS[0..3] to the binary value of the selected channel.
 *
 * \param channel which channel is currently selected */
void show_channel(int8_t channel) {
   LEDPORT.OUTCLR = CHANNEL_LEDS_bm;
   LEDPORT.OUTSET = channel;
}

/* \brief Selects the multiplexer channel.
 *
 * Sets the channel select pins to change which channel is being read by the
 * multiplexer
 *
 * \param channel which channel to select */
void set_channel(int8_t channel) {
   CHANNEL_PORT.OUTCLR = CHANNEL_PINS_bm;
   CHANNEL_PORT.OUTSET = channel & CHANNEL_PINS_bm;
}

/* \brief LEDS[4..7] will be lit if signal evaluates to true.
 *
 * \param signal_present whether a signal is present. */
void show_signal(uint8_t signal_present) {
   if (signal_present) {
      LEDPORT.OUTSET = SIGNAL_LEDS_bm;
   }
   else  {
      LEDPORT.OUTCLR = SIGNAL_LEDS_bm;
   }
}

int main(void) {
   int8_t channel = 0;
   bool pushed = 0;

   setup();

   while (1)
   {
      if (!pushed && (SWITCHPORTH.IN & CYCLE_SWITCH_bm) == 0) {
         if (channel == 15) {
            channel = -1;
         }
         set_channel(++channel);
         show_channel(channel);
         pushed = 1;
      }
      else if (pushed && (SWITCHPORTH.IN & CYCLE_SWITCH_bm)) {
         pushed = 0;
      }
      show_signal(SIGNAL_PORT.IN & SIGNAL_PIN_bm);
   }
}
