/** \file usart_bc.c
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

#include "usart_bc.h"

/* Macro Definitions **********************************************************/

#define USART_BC_MODULE USARTC0 ///< USART module connected to Board Controller

#define USART_BC_PORT       PORTC   ///< USART PORT wired to Board Controller
#define USART_BC_TXD_PIN_bm PIN3_bm ///< TxD pin bitmask
#define USART_BC_RXD_PIN_bm PIN2_bm ///< RxD pin bitmask

#define USART_BC_BSEL_9600   ((uint16_t)12) ///< BSEL for 9600 baud rate
#define USART_BC_BSCALE_9600 ((uint8_t)4)   ///< BSCALE for 9600 baud rate

#define USART_BC_ONE_STOP_BIT false ///< last parameter to USART_Format_Set()

#define USART_BC_RXC_vect USARTC0_RXC_vect ///< USART Receive Byte Complete IV
#define USART_BC_DRE_vect USARTC0_DRE_vect ///< USART Data Register Empty IV

#define LF 0x0A ///< Line Feed (\n) character ASCII code

/* Global Variables ***********************************************************/

/** \brief The USART channel which is initialized in interrupt mode to
 * communicate with the Board Controller */
USART_data_t USART_BC;

/** \brief Set to true when an LF has been received, indicating a new string has
 * been received. */
volatile uint8_t string_available;

/* Internal Function Prototypes ***********************************************/

static void redirect_stdout_to_BC(void);
static int USART_BC_putchar(char c, FILE *stream);

/* Global Variables ***********************************************************/

/** \brief Creates a FILE stream suitable for replacing stdout, which prints
 * characters to the connected Board Controller.
 *
 * To use, redefine stdout to be this FILE stream, like this:
 * \code stdout = &UART_BC_stdout; \endcode
 * This enables communicating with the Board Controller through the connected
 * UART channel (USARTC0), using the stdio.h library printf() functions, which
 * accepts all standard printf() flags.
 *
 * This set up is handled internally by a call to USART_BC_init(), however, the
 * same pattern implemented here could be modifed to change stdout to any FILE
 * stream.
 * */
static FILE USART_BC_stdout = FDEV_SETUP_STREAM (USART_BC_putchar, NULL,
                                                 _FDEV_SETUP_WRITE);

/* Function Definitions *******************************************************/

/** \brief Initializes the USART channel connected to the Board Controller for
 * transmit and receive, and redirects stdout to this channel.
 *
 * Sets up the USART channel connected to the Board Controller with the
 * following parameters:
 *    - asynchronous mode
 *    - 9600 baud rate
 *    - 8 bit frame size, no parity check, 1 stop bit, no flow control
 *
 * Once this library has been initialized, subsequent calls to printf() will be
 * transferred over the USART channel connected to the Board Controller, which
 * acts as UART-USB Bridge, so the communication can be monitored by a terminal
 * program running on a host connected over the USB port on the board.
 *
 * Optionally, the USART_data_t* returned by this function can be used directly
 * to send data over the USB line to a connected computer using the functions
 * provided in the ATMEL supplied usart_driver.c library.
 *
 * To check if data has been received over this channel, and to get the received
 * data, use the usart_driver.c library, with the return value from this
 * function as the USART_data_t* argument those library calls.
 *
 * \return Pointer to the USART initialized to communicate with the onboard
 * USART-USB gateway (receive and transmit enabled) */
void USART_BC_init(void) {
   /* The following order of operations is given by AVR XMEGA A Manual, section
    * 21.5, when using asynchronous communications:
    *    1. Set the TxD pin high
    *    2. Set the TxD pin as output, RxD pin as input
    *    3. Set the baud rate and frame format
    *    4. Set the mode of operation
    *    5. Enable the transmitter and/or receiver, depending on usage */
   USART_BC_PORT.OUTSET = USART_BC_TXD_PIN_bm;
   USART_BC_PORT.DIRSET = USART_BC_TXD_PIN_bm;
   USART_BC_PORT.DIRCLR = USART_BC_RXD_PIN_bm;
   USART_Baudrate_Set(&USART_BC_MODULE,
                      USART_BC_BSEL_9600,
                      USART_BC_BSCALE_9600);
   USART_Format_Set(&USART_BC_MODULE,
                    USART_CHSIZE_8BIT_gc,
                    USART_PMODE_DISABLED_gc,
                    USART_BC_ONE_STOP_BIT);
   USART_SetMode(&USART_BC_MODULE, USART_CMODE_ASYNCHRONOUS_gc);
   USART_InterruptDriver_Initialize(&USART_BC, &USART_BC_MODULE,
                                    USART_DREINTLVL_LO_gc);
   USART_RxdInterruptLevel_Set(&USART_BC_MODULE, USART_RXCINTLVL_LO_gc);
   USART_Tx_Enable(&USART_BC_MODULE);
   USART_Rx_Enable(&USART_BC_MODULE);

   /* enable low level interrupts */
   PMIC.CTRL |= PMIC_LOLVLEX_bm;

   /* enables printf() to send data over the USB line */
   redirect_stdout_to_BC();
}

/** \brief Returns whether a new string has been received from the Board
 * Controller.
 *
 * This function should be called before a call to USART_BC_get_string() to
 * check that there has been received since the last call to the get_string
 * function.
 *
 * \return If data is available in the software RX buffer
 * \retval 0 No new data available
 * \retval non-zero New string is available in software RX buffer */
bool USART_BC_RX_available(void) {
   return string_available;
}

/** \brief Flushes the RX buffer.
 *
 * This clears any data in the RX buffer. Any subsequent call to to the RX
 * functions in the usart_driver.c library will return uniquely new string
 * received since the last time this function was called. */
void USART_BC_flush_RX_buffer(void) {
   while (USART_RXBufferData_Available(&USART_BC)) {
      USART_RXBuffer_GetByte(&USART_BC);
   }

   string_available = 0;
}

/** \brief Returns the oldest string received over USB line, if any.
 *
 * Calls to this function should be prefaced by a call to
 * USART_BC_RX_available() to check if there is any new data receveied.
 * Received strings are Line Feed terminated (\n, 0x0A), which are replaced by
 * a null terminator when they are returned by this function.
 *
 * \param[out] buffer Where to save the string from the return buffer [size
 * must be GTE USART_RX_BUFFER_SIZE to prevent overflow]
 *
 * \return Whether there was any data in the software RX buffer to return
 * \retval 0 No new data in the software buffer
 * \retval non-zero buffer contains valid receive data */
bool USART_BC_get_string(char* buffer) {
   uint8_t counter = 0;
   char next_char = NUL;

   if (!string_available) {
      return false;
   }

   while (next_char != LF && USART_RXBufferData_Available(&USART_BC)) {
      next_char = USART_RXBuffer_GetByte(&USART_BC);
      *(buffer+(counter++)) = next_char;
   }

   /* if only LF received, counter will be 1 */
   if (counter < 2 || next_char != LF) {
      return false;
   }

   *(buffer+counter) = NUL; //replace LF with NUL terminator
   --string_available;
   return counter;
}

/* Internal Function Definitions **********************************************/

/** \brief helper method for USART_BC_init(), redirects stdout to Board
 * Controller USART PORT.
 *
 * Redefines stdout to UART_BC_stdout, which uses UART_BC_putchar() to print
 * characters to the UART channel connected to the Board Controller. This is
 * the function which allows subsequent printf() calls to be sent and
 * understood by the Board Controller for translation onto the USB line. */
static void redirect_stdout_to_BC(void) {
   stdout = &USART_BC_stdout;
}

/** \brief Transmits the given character to the Board Controller over UART.
 *
 * The success of this transmission is dependant on the settings initialized in
 * USART_BC_init(). The stream parameter passed in is the UART_BC_stdout stream
 * initialized in redirect_to_BC(), however, this function only supports
 * transmission over UART to the Board Controller, so it is ignored, but must
 * be present to conform to the putchar() format specified in stdio.h.
 *
 * \param[in] c the character to send
 * \param[in] stream the file stream to print to (ignored) */
static int USART_BC_putchar(char c, FILE *stream)
{
   if (c == '\n') {
      USART_BC_putchar('\r', stream);
   }

   /*    Add the byte to the interrupt driven transmit buffer.
    *    USART_TXBuffer_PutByte returns 0 if buffer is full, non-zero on succes
    *    */
   while (!USART_TXBuffer_PutByte(&USART_BC, c)) {
      nop();
   }

   return 0;
}

/* Interrupt Service Routines *************************************************/

/** \brief USART_BC receive byte complete interrupt
 *
 * Calls the interrupt handler from the ATMEL supplied usart_driver.c library,
 *    which handles saving the received byte in a buffer to make room for the
 *    next byte received.  */
ISR(USART_BC_RXC_vect)
{
   if (USART_RXComplete(&USART_BC) == LF) {
      ++string_available;
   }
}


/** \brief USART_BC Data Register Empty interrupt.
 *
 * Calls the interrupt handler from the ATMEL supplied usart_driver.c library,
 *    which handles saving the received byte in a buffer to make room for the
 *    next byte received. */
ISR(USART_BC_DRE_vect)
{
   USART_DataRegEmpty(&USART_BC);
}

