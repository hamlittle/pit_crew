/** \file adc.c
 *
 * \brief ADC driver implementation, see adc.h for full documentation.
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
 * limitations under the License. \endverbatim   */

/* Include Directives *********************************************************/

#include "adc.h"

/* Global Variables ***********************************************************/

/** \brief data to send to ADC (dummy data, ADC_MOSI is not connected) */
const uint8_t ADC_SPI_dummy_data[ADC_CON_BYTES] = { 0x55, 0xAA };

/* Function Definitions *******************************************************/

/** \brief Initializes the ADC.
 *
 * All library functions in this library take the adc given here as their first
 * parameter. This function initializes it for future communication with the
 * ADC.  This library uses the SPI library to communicate with the ADC, set up
 * in interrupt driven packet transfer mode, at 2MHz clock frequency. This
 * library performs bus arbitration through the SPI_driver.c library, however,
 * it is still a good idea to limit the ADC as the only peripheral on the bus
 * for performance reasons.
 *
 * \param[out] adc the adc to initialize
 * \param[in] port the port the ADC is wired to
 * \param[in] module the SPI module to use to communicate with the ADC
 * \param[in] CONVST_bm bitmask for the /CONVST pin on the given port
 * \param[in] EOC_bm bitmask for the /EOC pin on the given port */
void ADC_init(ADC_ext_t *adc, PORT_t *port, SPI_t *module, uint8_t CONVST_bm,
              uint8_t EOC_bm) {

   /* initialize the adc */
   adc->port = port;
   adc->CONVST_bm = CONVST_bm;
   adc->SPI_master = malloc(sizeof(SPI_Master_t));
   adc->data_packet = malloc(sizeof(SPI_DataPacket_t));
   adc->ready = true;
   adc->continuous = false;
   adc->callback = NULL;

   /* Init SS pin as output with push/pull configuration (set/cleared by MCU) */
   port->DIRSET = SPI_SS_bm;
   port->SPI_SS_PINCTRL = PORT_OPC_TOTEM_gc;
   port->OUTSET = SPI_SS_bm;

   /* enable /CONVST pin as output */
   port->DIRSET = CONVST_bm;
   port->OUTSET = CONVST_bm;

   /* Initialize SPI master on port C, which also sets up SPI interrupt */
   SPI_MasterInit(adc->SPI_master,         // SPI_master_t struct to use
                  module,                  // SPI module to use (see board.h)
                  port,                    // SPI port to use (see board.h)
                  false,                   // lsb_first = false
                  SPI_MODE_2_gc,           // data on leading falling edge
                  SPI_INTLVL_MED_gc,       // SPI interrupt priority
                  false,                   // clock2x = false
                  SPI_PRESCALER_DIV16_gc); // division scaler (2MHz)

   /* SPI is full duplex only, so we have to create a dummy packet in order
    * to receive data from the ADC  */
   SPI_MasterCreateDataPacket(adc->data_packet,   // dummy data packet
                              ADC_SPI_dummy_data, // dummy data
                              adc->result_buffer, // buffer to save conversion
                              ADC_CON_BYTES,      // bytes to transceive
                              port,               // needed for next param
                              SPI_SS_bm);         // /SS pin for ADC

   /* enable interrupts on /EOC for falling edge
    * (ADC pulls low to signal conversion ready) */
   port->DIRCLR = EOC_bm;                  // EOC pin is input
   PORTCFG.MPCMASK = EOC_bm;
   port->PIN0CTRL = PORT_ISC_FALLING_gc; // pulled low when ready
   port->INT0MASK |= EOC_bm;               // enable EOC interupt mask
   port->INTCTRL = PORT_INT0LVL_LO_gc;          // enable LO level interrupts

   /* enable low and med level interrupts */
   PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
}

/** \brief Registers the callback function for continuous sampling mode
 * operation.
 *
 * When the ADC is in continuous sampling mode, this callback will be triggered
 * after each conversion completes. The callback format is given by
 * ADC_callback_t.
 *
 * \param[in] adc the adc to register the callback for
 * \param[in] callback the callback to register */
void ADC_register_continuous_callback(ADC_ext_t *adc, ADC_callback_t callback) {
   adc->callback = callback;
}

/** \brief Begins continuous ADC sampling and conversions.
 *
 * When operating in continuous mode, the ADC performs conversions indefinately
 * until an accompanying call to ADC_stop_continuous() or ADC_sample_once() is
 * made. A callback function must be registered with the given adc, through
 * ADC_register_continuous_callback(), which is called after each conversion
 * completes. A new conversion is then started when this callback function
 * returns. This is done in case it takes longer to return from the callback
 * function than it does to perform a conversion, which would cause the MCU to
 * enter an infinite loop
 *
 * \param[in] adc the ADC to begin sampling continuously.  */
void ADC_sample_continuous(ADC_ext_t *adc) {
   /* set flags for continuous mode */
   adc->continuous = true;
   adc->ready = false;

   /* start a conversion */
   adc->port->OUTCLR = adc->CONVST_bm;
   adc->port->OUTSET = adc->CONVST_bm;
}

/** \brief Stops continuous mode ADC sampling.
 *
 * If there is an ongoing conversion when this function is called, the ADC will
 * be allowed to finish its conversion, and the result will be available
 * through the ADC_get_last_result() macro when the ADC_ready() macro returns
 * true.
 *
 * \param[in] adc the ADC to stop continuous sampling mode */
void ADC_stop_continuous(ADC_ext_t *adc) {
   /* stop continuous conversions */
   adc->continuous = false;
}

/** \brief Samples the ADC once, blocking until conversion is complete.
 *
 * Calling this function while the ADC is in continuous sample mode will
 * disable continuous conversions, perform an additional conversion, and return
 * the result This is to prevent the function from busy waiting indefinetely
 * for the ADC to be ready for a new conversion, as conversions are being
 * continously initiated by the ADC_EOC_interrupt_handler() from the user's ISR
 * in continuous mode.
 *
 * \param[in] adc the ADC to start a conversion on
 *
 * \return conversion result */
uint16_t ADC_sample_once(ADC_ext_t *adc) {
   /* stop continuous conversions, if they are ongoing */
   adc->continuous = false;

   /* wait for an ongoing convrsion to finish */
   while (!adc->ready){
      nop();
   }

   /* start a conversion */
   adc->ready = false;
   /* delay_us(1); // if ADC stops working, try uncommenting this line */
   adc->port->OUTCLR = adc->CONVST_bm;
   adc->port->OUTSET = adc->CONVST_bm;

   /* wait for conversion to finish */
   while (!adc->ready){
      nop();
   }

   return ADC_get_last_result(adc);
}

/** \brief ADC SPI interrupt Handler.
 *
 * This function first passes control over to the SPI_MasterInterruptHandler(),
 * which takes care of bus arbitration and sequential byte transfers from the
 * ADC. When the SPI transfer is complete, the result is stored in a temporary
 * buffer, which will be overwritten on the next conversion.
 *
 * In continuous mode, the registered callback will be called so the user can
 * process the conversion result. When the callback returns, a new conversion
 * will be started. This is to prevent the case where it takes less time for
 * the callback to return than it does to perform a conversion.
 *
 * In single sample mode, the ADC is made ready for a new conversion, and the
 * result is returned from the call to ADC_sample_once(), which is blocking
 * until the SPI transfer with the ADC completes
 *
 * \note This function should be called from the ISR registered to trigger on
 * the SPI IRQ associated with the ADC, as described in this file's
 * documentation.
 *
 * \param[in] adc The ADC which   */
void ADC_SPI_interrupt_handler(ADC_ext_t *adc) {
   SPI_MasterInterruptHandler(adc->SPI_master);

   /* check if transmission is complete */
   if (SPI_MasterInterruptTransmissionComplete(adc->SPI_master)) {
      if (adc->continuous) {
         /* let the user know a sample is ready, conversion result is stored in
          * adc->result_buffer by SPI driver */
         adc->callback(ADC_get_last_result(adc));

         /* start a new conversion */
         /* delay_us(1); // if ADC stops working, try uncommenting this line */
         adc->port->OUTCLR = adc->CONVST_bm;
         adc->port->OUTSET = adc->CONVST_bm;
      }
      else {
         /* ADC_sample_once is busy waiting for the adc->ready flag to be set.
          * Conversion result is stored in adc->result_buffer by SPI driver. */
         adc->ready = true;
      }
   }
}

/** \brief End of Conversion interrupt handler.
 *
 * This function starts an SPI transfer with the ADC in order to retrieve the
 * conversion result, using the SPI driver library's
 * SPI_MasterInterruptTransceivePacket().
 *
 * \note This function should be called from the ISR registered to trigger on
 * the falling edge of EOC, using INT0 of the port associated with the ADC, as
 * described in this file's documentation.
 *
 * \param[in] adc The ADC which has signalled EOC */
void ADC_EOC_interrupt_handler(ADC_ext_t *adc) {
   uint8_t status;

   /* initiate a data transfer to get the conversion result */
   do {
      status = SPI_MasterInterruptTransceivePacket(adc->SPI_master,
                                                   adc->data_packet);
   } while (status != SPI_OK);
}
