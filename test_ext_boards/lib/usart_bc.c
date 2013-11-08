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

/* Internal Function Prototypes ***********************************************/

void redirect_stdout_to_BC(void);
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

/** \brief Initializes the USART channel connected to the Board Controller, and
 *    redirects stdout to this channel.
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
 * program running on a host connected over the USB port on the board. */
void USART_BC_init(void) {
   /* The following order of operations is given by AVR XMEGA A Manual, section
    * 21.5, when using asynchronous communications:
    *    1. Set the TxD pin high
    *    2. Set the TxD pin as output
    *    3. Set the baud rate and frame format
    *    4. Set the mode of operation
    *    5. Enable the transmitter or receiver, depending on usage */
   USART_BC_PORT.OUTSET = USART_BC_TXD_PIN_bm;
   USART_BC_PORT.DIRSET = USART_BC_TXD_PIN_bm;
   USART_Baudrate_Set(&USART_BC_MODULE,
                      USART_BC_BSEL_9600,
                      USART_BC_BSCALE_9600);
   USART_Format_Set(&USART_BC_MODULE,
                    USART_CHSIZE_8BIT_gc,
                    USART_PMODE_DISABLED_gc,
                    USART_BC_ONE_STOP_BIT);
   USART_SetMode(&USART_BC_MODULE, USART_CMODE_ASYNCHRONOUS_gc);
   USART_Tx_Enable(&USART_BC_MODULE);

   redirect_stdout_to_BC();
}

/* Internal Function Definitions **********************************************/

/** \brief helper method for USART_BC_init(), redirects stdout to Board
 * Controller USART PORT.
 *
 * Redefines stdout to UART_BC_stdout, which uses UART_BC_putchar() to print
 * characters to the UART channel connected to the Board Controller. This is
 * the function which allows subsequent printf() calls to be sent and
 * understood by the Board Controller for translation onto the USB line. */
void redirect_stdout_to_BC(void) {
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
   if (c == '\n')
      USART_BC_putchar('\r', stream);

   // Wait for the transmit buffer to be empty
   while (!(USART_IsTXDataRegisterEmpty(&USART_BC_MODULE)));

   // Put our character into the transmit buffer
   USART_PutChar(&USART_BC_MODULE, c);

   return 0;
}
