/** \defgroup ADC_API ADC Library API */ /** @{ */
/** \file adc.h
 *
 * \brief Enables interfacing with the ADC, in continuous and single sampling
 * modes.
 *
 * This library simplifies interfacing with the AD7892ANZ1 Analog to Digital
 * Converter (ADC). ADC_init initializes the library, after which the ADC can be
 * run in continuous or single sampling modes. This library depends on two ISR's
 * which must be registered by the user, explained in the note below.
 *
 * \note This library relies on two ISR's which must be set up by the user: one
 * for the end of conversion (/EOC) pin on the ADC, and the second for the SPI
 * bus used to communicate with the ADC. Each interrupt source is configured in
 * ADC_init (/EOC is configured as INT0 on the given port, SPI is one of the
 * SPIx_INT_vect vectors, whichever corresponds to the SPI port associated with
 * the ADC), however, the user must register the ISR's for each and call
 * ADC_EOC_interrupt_handler() or ADC_SPI_interrupt_handler() appropriately
 * from within them, passing the ADC being used on that port, as in the
 * following examples:
 * \code
 *
 * // user code
 *
 * ISR(PORTx_INT0_vect) {
 *    ADC_EOC_interrupt_handler(adc);
 * }
 *
 * ISR(SPIx_INT_vect) {
 *    ADC_SPI_interrupt_handler(adc);
 * }
 * \endcode
 * where the \c \<x\> in \c PORTx_INT0_vect and \c SPIx_INT_vect are replaced by
 * the appropriate port letters.
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
#include "spi_driver.h"
#include "board.h"

/** \name Macro Defined Functions *********************************************/
///@{

/** \brief Returns the last ADC conversion result.
 *
 * \param[in] _adc the adc to get the result from

 * \return The last adc conversion result
 * */
#define ADC_get_last_result(_adc) ((((uint16_t)((_adc)->result_buffer[0]))<<8)\
                                   | (_adc)->result_buffer[1])

/** \brief Returns whether the ADC is ready for a new conversion.
 *
 * Initiating a conversion, through ADC_sample_once(), when the ADC is not
 * ready will block until it is. Use this function to prevent ADC_sample_once()
 * from busy waiting for the current conversion to finish.
 *
 * \param[in] _adc the adc to check the status of

 * \return true if the ADC is ready for a new conversion */
#define ADC_ready(_adc) ((_adc)->ready)

///@}

/** \name Callback Function ***************************************************/
///@{

/** \brief Callback function type definition.
 *
 * This library makes use of a callback function, which is called whenever
 * a conversion is completed while the adc is in continuous mode. The format of
 * the function pointer is a function which returns void, and is passed a single
 * uint16_t parameter, the latest conversion result.
 *
 * \note Care must be taken that the implemented callback is ISR safe, as it is
 * called from the ADC_SPI_interrupt_handler() in continuous conversion mode.
 *
 * \note ADC conversion result is a right justified, 12-bit result, where the
 * msb is the sign bit, but is not extended to the left. This means that, if the
 * result is negative, the conversion will still be padded on the left by
 * 4 leading 0's, then the sign bit, then the lower 11 bits are the magnitude
 * of the conversion.
 *
 * \param[in] result ADC conversion result */
typedef void (*ADC_callback_t)(uint16_t result);

///@}


/* Library Definitions ********************************************************/

#define ADC_CON_BYTES 2 ///< ADC conversion result is 2 bytes

/** \brief Contains the static data needed to communicate with the ADC.
 *
 * This struct is used as the first parameter to all of the ADC_* functions in
 * this library */
typedef struct ADC_external_struct  {
   PORT_t *control_port;                 ///< ADC control port
   uint8_t CONVST_bm;                    ///< CONVST pin bm
   PORT_t *SPI_port;                     ///< ADC SPI port
   uint8_t RFS_bm;                       ///< ADC SPI SS pin bm
   SPI_Master_t *SPI_master;    ///< SPI to use, alloc'd in ADC_init()
   SPI_DataPacket_t *data_packet; ///< dummy data packet to transmit
   uint8_t result_buffer[ADC_CON_BYTES]; ///< Conversion result buffer
   volatile bool ready;                  ///< false if conversion ongoing
   bool continuous;                      ///< operation mode flag
   ADC_callback_t callback;              ///< continuous mode callback
} ADC_ext_t;

/* Function Prototypes ********************************************************/

/** \name Library Initializers */
///@{
void ADC_init(ADC_ext_t *adc,
              PORT_t *control_port, uint8_t CONVST_bm, uint8_t EOC_bm,
              PORT_t *SPI_port, SPI_t *SPI_module, uint8_t RFS_bm);
void ADC_register_continuous_callback(ADC_ext_t *adc, ADC_callback_t callback);
///@}

void ADC_sample_continuous(ADC_ext_t *adc);
void ADC_stop_continuous(ADC_ext_t *adc);
uint16_t ADC_sample_once(ADC_ext_t *adc);
void ADC_set_output_data(ADC_ext_t *adc, const uint8_t *data);
void ADC_SPI_interrupt_handler(ADC_ext_t *adc);
void ADC_EOC_interrupt_handler(ADC_ext_t *adc);
/** @} */ /* End of \defgroup ADC_API */
