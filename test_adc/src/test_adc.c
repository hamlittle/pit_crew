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

/**** global variables ********************************************************/

volatile bool ADC_ready = false;
SPI_Master_t SPI_master;
SPI_DataPacket_t data_packet;
uint8_t dummy_data[ADC_CON_BYTES] = { 0x55, 0x55 }; // MOSI is not connected
uint8_t ADC_result_buffer[ADC_CON_BYTES];

/**** function definitions ****************************************************/
// see header file for documentation

void setup(void) {
   cli();
   setup_clocks();
   setup_leds();
   setup_SR_pins();
   setup_switches();
   setup_adc();
   sei();
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

void setup_switches(void) {
   SWITCHPORTH.DIR &= ~CYCLE_SWITCHES_gm;   // set the sw[6..7] pin as input
   SWITCHPORTH.CHANSEL0SWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
   SWITCHPORTH.CHANSEL1SWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
}

void setup_adc() {
   /* Init SS pin as output with push/pull configuration (set/cleared by MCU) */
   SPI_PORT.DIRSET = SPI_SS_PIN;
   SPI_PORT.SPI_SS_PINCTRL = PORT_OPC_TOTEM_gc;

   /* Set SS output to high (disable SPI) */
   SPI_PORT.OUTSET = SPI_SS_PIN;

   /* Initialize SPI master on port C. */
   SPI_MasterInit(&SPI_master,              // SPI_master_t struct to use
                  &SPI_module,             // SPI module to use (see board.h)
                  &SPI_PORT,               // SPI port to use (see board.h)
                  false,                   // lsb_first = false
                  SPI_MODE_2_gc,           // data on leading falling edge
                  SPI_INTLVL_MED_gc,       // SPI interrupt priority
                  false,                   // clock2x = false
                  SPI_PRESCALER_DIV64_gc); // division scaler (0.5MHz)

   /* enable /CONVST pin as output */
   ADC_PORT.DIRSET = ADC_CONVST_PIN;

   /* enable interrupts on /EOC for falling edge
    * (ADC pulls low to signal conversion ready) */
   ADC_PORT.DIRCLR = ADC_EOC_PIN;                  // EOC pin is input
   ADC_PORT.ADC_EOC_PINCTRL = PORT_ISC_FALLING_gc; // pulled low when ready
   ADC_PORT.INT0MASK |= ADC_EOC_PIN;               // enable EOC interupt mask
   ADC_PORT.INTCTRL = PORT_INT0LVL_LO_gc;          // enable LO level interrupts

   /* enable low and med level interrupts */
   PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
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

void show_result(uint8_t result) {
   LEDPORT.OUT = result;
}

int main(void) {
   uint8_t channel0 = 0;
   uint8_t channel1 = 15;

   /* call all of the setup_* functions */
   setup();

   /* enable starting ADC conversions */
   ADC_ready = true;

   set_channel(mp0, channel0);
   set_channel(mp1, channel1);

   SPI_MasterCreateDataPacket(&data_packet,      // the SPI packet
                              dummy_data,        // data to be sent
                              ADC_result_buffer, // result buffer
                              ADC_CON_BYTES,     // bytes to transceive
                              &SPI_PORT,          // used by next param
                              SPI_SS_PIN);       // /ss pin to use
   while (1)
   {
      /* map upper 8 bits of 12-bit result to LEDS this is the lower 4 bits of
       * the first byte, and the upper 4 bits of the second byte  */
      show_result((ADC_result_buffer[0] << 4) & (ADC_result_buffer[1] >> 4));

      if (ADC_ready) {
         /* start a new conversion, EOC interrupt will fire when finished */
         ADC_ready = false;
         ADC_PORT.OUTSET |= ADC_CONVST_PIN; // pulled low to start conversion
         ADC_PORT.OUTCLR |= ADC_CONVST_PIN;
      }

      /* Wait for transmission to complete, result saved in ADC_result_buffer */
      if (data_packet.complete) {
         ADC_ready = true;
      }
   }
}

/** \brief ADC PORT /EOC interrupt vector
 *
 * /EOC is pulled low when a conversion is ready. In order to get the
 * conversion result, we start a SPI transmission here, and check for the
 * complete field of the data packet parameter to
 * SPI_MasterInterruptTransceivePacket to be true in the main loop. */
ISR(ADC_EOC_INT_VECT) {
   /* start a transmission, returns after first byte successfully sent */
   uint8_t status;
   do {
      status = SPI_MasterInterruptTransceivePacket(&SPI_master, &data_packet);
   } while (status != SPI_OK);
}
