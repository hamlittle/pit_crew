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

#include "test_adc.h"

/**** function definitions ****************************************************/
// see header file for documentation

void setup(void) {
   setup_clocks();
   setup_leds();
   setup_SR_pins();
   setup_signal_pins();
   setup_switches();
}

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

void setup_leds(void) {
   LEDPORT.DIR = 0xff;                 // set all pins of port E to output
   PORTCFG.MPCMASK = 0xff;             // set for all pins on port E...
   LEDPORT.PIN0CTRL |= PORT_INVEN_bm;  // inverted output (set hi turns on led)
   LEDPORT.OUTCLR = 0xff;              // turn all leds off
}

void setup_SR_pins(void) {
   SR_PORT.DIR |= SR_PINS_gm; // set SR pins as outputs
   SR_PORT.OUTCLR = 0xff;     // set to 0 initially
}

void setup_signal_pins(void) {
   SIGNAL_PORT.DIR &= ~(SIGNAL_PINS_gm); // set the signal pin as an input

   // pins low by default, signal when high
   SIGNAL_PORT.SIG0PINCTRL = (SIGNAL_PORT.SIG0PINCTRL & ~PORT_OPC_gm) |
      PORT_OPC_PULLDOWN_gc;
   SIGNAL_PORT.SIG1PINCTRL = (SIGNAL_PORT.SIG1PINCTRL & ~PORT_OPC_gm) |
      PORT_OPC_PULLDOWN_gc;
}

void setup_switches(void) {
   SWITCHPORTH.DIR &= ~CYCLE_SWITCHES_gm;   // set the sw[6..7] pin as input
   SWITCHPORTH.CHANSEL0SWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
   SWITCHPORTH.CHANSEL1SWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
}

void show_channel(mp_select_t mp_select, uint8_t channel) {
   static int8_t channel0 = 0;
   static int8_t channel1 = 0;

   if (mp_select == mp0) {
      channel0 = channel;
      LEDPORT.OUTCLR = CHANNEL0_LEDS_gm;
   }
   else {
      channel1 = channel;
      LEDPORT.OUTCLR = CHANNEL1_LEDS_gm;
   }
   LEDPORT.OUTSET = channel0 | channel1 << LEDPORT_CHAN1_off;
}

void set_channel(mp_select_t mp_select, uint8_t channel) {
   static int8_t chan_pins = 4;
   int8_t chan_counter;
   int8_t chan_pin_mask = 0x08;
   static uint8_t channel0 = 0;
   static uint8_t channel1 = 0;

   SR_PORT.OUTCLR = SR_PINS_gm;

   if (mp_select == mp0) {
      channel0 = channel;
   }
   else {
      channel1 = channel;
   }

   for (chan_counter = 0; chan_counter < chan_pins; ++chan_counter) {
      // set order: msb->lsb
      if (channel0 & (chan_pin_mask >> chan_counter)) {
         SR_PORT.OUTSET = SR_SER_IN_PIN_bm;
      }
      else {
         SR_PORT.OUTCLR = SR_SER_IN_PIN_bm;
      }
      SR_PORT.OUTSET = SR_CLOCK_PIN_bm; // clock high to shift in value
      SR_PORT.OUTCLR = SR_CLOCK_PIN_bm; // clock low to set next value
   }
   for (chan_counter = 0; chan_counter < chan_pins; ++chan_counter) {
      // set order: msb->lsb
      if (channel1 & (chan_pin_mask >> chan_counter)) {
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

void show_signal(mp_select_t mp_select, uint8_t signal_present) {
   static bool chan0_toggled = false;
   static bool chan1_toggled = false;

   // if signal high and not toggled, then toggle to show signal
   // if signal low and toggled, then revert to showing channel
   if (mp_select == mp0) {
      if (signal_present && !chan0_toggled) {
         LEDPORT.OUTTGL = CHANNEL0_LEDS_gm;
         chan0_toggled = true;
      }
      else if (!signal_present && chan0_toggled) {
         LEDPORT.OUTTGL = CHANNEL0_LEDS_gm;
         chan0_toggled = false;
      }
   }
   else {
      if (signal_present && !chan1_toggled) {
         LEDPORT.OUTTGL = CHANNEL1_LEDS_gm;
         chan1_toggled = true;
      }
      else if (!signal_present && chan1_toggled) {
         LEDPORT.OUTTGL = CHANNEL1_LEDS_gm;
         chan1_toggled = false;
      }
   }
}

int main(void) {
   uint8_t channel0 = 0;
   uint8_t channel1 = 0;
   bool chan0_pushed = 0;
   bool chan1_pushed = 0;

   setup();

   set_channel(mp0, channel0);
   set_channel(mp1, channel1);
   while (1)
   {
      if (!chan0_pushed && (SWITCHPORTH.IN & CYCLE_SWITCH0_bm) == 0) {
         if (channel0 == 15) {
            channel0 = -1;
         }
         set_channel(mp0, ++channel0);
         show_channel(mp0, channel0);
         chan0_pushed = 1;
      }
      else if (chan0_pushed && (SWITCHPORTH.IN & CYCLE_SWITCH0_bm)) {
         chan0_pushed = 0;
      }

      if (!chan1_pushed && (SWITCHPORTH.IN & CYCLE_SWITCH1_bm) == 0) {
         if (channel1 == 15) {
            channel1 = -1;
         }
         set_channel(mp1, ++channel1);
         show_channel(mp1, channel1);
         chan1_pushed = 1;
      }
      else if (chan1_pushed && (SWITCHPORTH.IN & CYCLE_SWITCH1_bm)) {
         chan1_pushed = 0;
      }

      show_signal(mp0, SIGNAL_PORT.IN & SIGNAL0_PIN_bm);
      show_signal(mp1, SIGNAL_PORT.IN & SIGNAL1_PIN_bm);
   }
}
