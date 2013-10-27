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

/** \brief Wait until /EOC is pulled low before new conversion started */
volatile bool ADC_ready = false;

/** \brief ADC SPI master mode struct */
SPI_Master_t ADC_SPI_master;

/** \brief ADC data packet (dummy data, ADC_MOSI is not connected) */
SPI_DataPacket_t ADC_SPI_data_packet;

/** \brief data to send to ADC (dummy data, ADC_MOSI is not connected) */
const uint8_t ADC_SPI_MOSI_data[ADC_CON_BYTES] = { 0x55, 0xAA };

/** \brief ADC conversion result buffer */
uint8_t ADC_result_buffer[ADC_CON_BYTES];

/** \brief Shift Register SPI master mode struct */
SPI_Master_t SR_SPI_master;

/** \brief Shift Register data packet ([0..3] is MP1, [4..7] is MP0) */
SPI_DataPacket_t SR_SPI_data_packet;

/** \brief Shift Register MISO buffer (unused, SR_MISO is not connected) */
uint8_t SR_SPI_MISO_buffer;

/**** function definitions ****************************************************/
// see header file for documentation

void setup(void) {
   cli();
   setup_clocks();
   setup_leds();
   setup_SR();
   setup_switches();
   setup_ADC();
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

void setup_SR(void) {
   /* Init SS (L_CLOCK) pin as output with push/pull configuration  */
   SR_SPI_PORT.DIRSET = SR_SPI_SS_PIN;
   SR_SPI_PORT.SR_SPI_SS_PINCTRL = PORT_OPC_TOTEM_gc;
   SR_SPI_PORT.OUTSET = SR_SPI_SS_PIN;

   /* Initialize SPI master on port C. */
   SPI_MasterInit(&SR_SPI_master,         // SPI_master_t struct to use
                  &SR_SPI_module,         // SPI module to use (see board.h)
                  &SR_SPI_PORT,           // SPI port to use (see board.h)
                  true,                   // lsb_first = true
                  SPI_MODE_0_gc,          // data on leading rising edge
                  SPI_INTLVL_OFF_gc,      // SPI interrupt priority
                  true,                   // clock2x = true
                  SPI_PRESCALER_DIV4_gc); // division scaler (15MHz)

   /* enable low and med level interrupts */
   PMIC.CTRL |= PMIC_MEDLVLEN_bm;
}

void setup_switches(void) {
   SWITCHPORTH.DIR &= ~CYCLE_SWITCHES_gm;   // set the sw[6..7] pin as input
   SWITCHPORTH.CHANSEL0SWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
   SWITCHPORTH.CHANSEL1SWCTRL |= PORT_OPC_PULLUP_gc; // hitting sw pulls to gnd
}

void setup_ADC() {
   /* Init SS pin as output with push/pull configuration (set/cleared by MCU) */
   ADC_SPI_PORT.DIRSET = ADC_SPI_SS_PIN;
   ADC_SPI_PORT.ADC_SPI_SS_PINCTRL = PORT_OPC_TOTEM_gc;
   ADC_SPI_PORT.OUTSET = ADC_SPI_SS_PIN;

   /* enable /CONVST pin as output */
   ADC_PORT.DIRSET = ADC_CONVST_PIN;
   ADC_PORT.OUTSET = ADC_CONVST_PIN;

   /* enable interrupts on /EOC for falling edge
    * (ADC pulls low to signal conversion ready) */
   ADC_PORT.DIRCLR = ADC_EOC_PIN;                  // EOC pin is input
   ADC_PORT.ADC_EOC_PINCTRL = PORT_ISC_FALLING_gc; // pulled low when ready
   ADC_PORT.INT0MASK |= ADC_EOC_PIN;               // enable EOC interupt mask
   ADC_PORT.INTCTRL = PORT_INT0LVL_LO_gc;          // enable LO level interrupts

   /* Initialize SPI master on port C. */
   SPI_MasterInit(&ADC_SPI_master,         // SPI_master_t struct to use
                  &ADC_SPI_module,         // SPI module to use (see board.h)
                  &ADC_SPI_PORT,           // SPI port to use (see board.h)
                  false,                   // lsb_first = false
                  SPI_MODE_2_gc,           // data on leading falling edge
                  SPI_INTLVL_MED_gc,       // SPI interrupt priority
                  false,                   // clock2x = false
                  SPI_PRESCALER_DIV16_gc); // division scaler (2MHz)

   /* SPI is full duplex only, so we have to create a dummy packet in order
    * to receive data from the ADC  */
   SPI_MasterCreateDataPacket(&ADC_SPI_data_packet, // dummy data packet
                              ADC_SPI_MOSI_data,    // dummy data
                              ADC_result_buffer,    // buffer to save conversion
                              ADC_CON_BYTES,        // bytes to transceive
                              &ADC_SPI_PORT,        // needed for next param
                              ADC_SPI_SS_PIN);      // /SS pin for ADC

   /* enable low and med level interrupts */
   PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
}

void set_channel(mp_select_t mp_select, uint8_t channel) {
   static uint8_t channel0 = 0;
   static uint8_t channel1 = 0;
   uint8_t status;

   if (mp_select == mp0) {
      channel0 = channel;
   }
   else {
      channel1 = channel;
   }
   const uint8_t combined_channel_select =
      ((channel1 & 0x0F) << 4) | (channel0 & 0x0F);


   SR_SPI_PORT.OUTCLR = SR_SPI_SS_PIN;
   SPI_MasterTransceiveByte(&SR_SPI_master, combined_channel_select);
   SR_SPI_PORT.OUTSET = SR_SPI_SS_PIN;
}

void show_result(uint8_t result) {
   LEDPORT.OUT = result;
}

int main(void) {
   /* call all of the setup_* functions */
   setup();

   /* enable starting ADC conversions */
   ADC_ready = true;

   set_channel(mp0, 0);
   set_channel(mp1, 15);

   /* signal debugging */
   PORTD.DIRCLR |= PIN4_bm;
   PORTD.PIN4CTRL = PORT_OPC_PULLDOWN_gc;

   while (1)
   {
      if (ADC_ready) {
         // start a new conversion, EOC interrupt will fire when finished
         ADC_ready = false;
         delay_us(1); // DO NOT REMOVE, this delay is critical to ADC timing
         ADC_PORT.OUTCLR = ADC_CONVST_PIN; // pulled low to start conversion
         ADC_PORT.OUTSET = ADC_CONVST_PIN;
      }

      // Wait for transmission to complete, result saved in ADC_result_buffer
      if (PORTD.IN & PIN4_bm) {
         LEDPORT.OUT = 0xFF;
      }
      else if (ADC_SPI_data_packet.complete) {
         LEDPORT.OUT = (ADC_result_buffer[0]<<4) | (ADC_result_buffer[1]>>4);
         ADC_ready = true;
      }

      /* if (SR_SPI_data_packet.complete) { */
      /* } */

   }
}

/** \brief ADC PORT /EOC interrupt vector
 *
 * /EOC is pulled low when a conversion is ready. In order to get the
 * conversion result, we start a SPI transmission here, and check for the
 * complete field of the data packet parameter to
 * SPI_MasterInterruptTransceivePacket to be true in the main loop. */
ISR(ADC_EOC_INT_VECT) {
   // start a transmission, returns after first byte successfully sent
   uint8_t status;

   do {
      status = SPI_MasterInterruptTransceivePacket(&ADC_SPI_master,
                                                   &ADC_SPI_data_packet);
   } while (status != SPI_OK);
}

/** \brief ADC SPI interrupt
 *
 * Calls SPI_MasterInterruptHandler, passing it the SPI_master_t struct used
 * for the SPI communication. The SPI_MasterInterruptHandler takes care of bus
 * arbitration, and transmitting sequential bytes in a packet until the whole
 * packet has been transferred. Transfer is complete when the SPI_DataPacket_t
 * used as a parameter to SPI_MasterCreateDataPacket has been fully
 * transmitted, in which case the complete field of the packet is set, which
 * can be checked in the main loop */
ISR(ADC_SPI_INT_vect) {
   SPI_MasterInterruptHandler(&ADC_SPI_master);
}
