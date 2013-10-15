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

#include "test_shift_register.h"

/**** function definitions ****************************************************/
// see header file for documentation

void setup(void) {
   setup_leds();
   setup_SR_pins();
   setup_signal_pin();
   setup_switches();
}

void setup_leds(void) {
   LEDPORT.DIR = 0xff;                 // set all pins of port E to output
   PORTCFG.MPCMASK = 0xff;             // set for all pins on port E...
   LEDPORT.PIN0CTRL |= PORT_INVEN_bm;  // inverted output (set hi turns on led)
   LEDPORT.OUTCLR = 0xff;              // turn all leds off
}

void setup_SR_pins(void) {
   SR_PORT.DIR |= SR_PINS_gm; // set SR pins as outputs
   SR_PORT.OUTCLR = 0xff;     // set to 0 initially
   return;
}

void setup_signal_pin(void) {
   SIGNAL_PORT.DIR &= ~SIGNAL_PIN_bm; // set the signal pin as an input
   SIGNAL_PORT.SIGPINCTRL |= PORT_OPC_PULLDOWN_gc;
}

void setup_switches(void) {
   SWITCHPORTH.DIR &= ~CYCLE_SWITCH_bm;   // set the switch 7's pin as input
   SWITCHPORTH.CHANSELSWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
}


void show_channel(int8_t channel) {
   LEDPORT.OUTCLR = CHANNEL_LEDS_gm;
   LEDPORT.OUTSET = channel;
}

void set_channel(int8_t channel) {
   static int8_t chan_pins = 4;
   int8_t chan_counter;
   int8_t chan_pin_mask = 0x08;

   SR_PORT.OUTCLR = SR_PINS_gm;

   for (chan_counter = 0; chan_counter < chan_pins; ++chan_counter) {
      // set order: msb->lsb
      if (channel & (chan_pin_mask >> chan_counter)) {
         SR_PORT.OUTSET = SR_SER_IN_PIN_bm;
      }
      else {
         SR_PORT.OUTCLR = SR_SER_IN_PIN_bm;
      }
      SR_PORT.OUTSET = SR_CLOCK_PIN_bm; // clock high to shift in value
      SR_PORT.OUTCLR = SR_CLOCK_PIN_bm; // clock low to set next value
   }

   SR_PORT.OUTSET = SR_L_CLOCK_PIN_bm; // toggle L_CLOCK...
   SR_PORT.OUTCLR = SR_L_CLOCK_PIN_bm; // to dump the output latch
}

void show_signal(uint8_t signal_present) {
   if (signal_present) {
      LEDPORT.OUTSET = SIGNAL_LEDS_gm;
   }
   else  {
      LEDPORT.OUTCLR = SIGNAL_LEDS_gm;
   }
}

int main(void) {
   int8_t channel = 0;
   bool pushed = 0;


   setup();

   set_channel(0);
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
