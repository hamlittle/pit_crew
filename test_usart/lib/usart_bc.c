/** \file usart_bc.h
 *
 * \brief Sets up the USARTC0 communication with the board controller, enabling
 * printf() functionality to send characters over usb connection.
 *
 * This library intializes the UART channel connected to the Board Controller on
 * the XPLAINED A1 board. After a call to UART_BC_init(), the user can use the
 * stdio.h library printf() function to send characters over the UART channel to
 * the Board Controller, which accepts all standard flags and conversions. The
 * Board Controller, by default, acts as a UART to USB bridge, so any characters
 * sent to it using the printf() function will be transferred to the USB
 * connection. This allows for any machine connected to the usb port to monitor
 * the communications being sent, either by a terminal program or otherwise.
 *
 * Communication is set up with the following parameters:
 * - asynchronous mode
 * - 9600 baud rate
 * - 8 bit frame size, no parity, 1 stop bit, no flow control
 *
 * This library is blocking while a transfer is ongoing, so transmissions should
 * be kept as short as possible to avoid delays in user code.
 *
 * \todo enable interrupt mode UART transmissions, to avoid blocking while long
 * transmissions are ongoing.
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
static FILE UART_BC_stdout = FDEV_SETUP_STREAM (UART_BC_putchar, NULL,
                                                _FDEV_SETUP_WRITE);

/* Function Prototypes ********************************************************/

void redirect_stdout_to_BC(void );

/* Function Definitions *******************************************************/

static void USART_BC_init (void)
{
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

void redirect_stdout_to_BC(void) {
   stdout = &UART_BC_stdout;
}
