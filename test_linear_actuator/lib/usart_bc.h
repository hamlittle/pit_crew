/** \defgroup usart_bc_api USART-USB Board Controller API */ /** @{ */
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

#ifndef _USART_BC_H_
#define _USART_BC_H_

/* Include Directives *********************************************************/

#include <stdio.h>

#include "usart_driver.h"

/* Macro Definitions **********************************************************/

#define USART_BC_MODULE USARTC0 ///< USART module connected to Board Controller

#define USART_BC_PORT       PORTC   ///< USART PORT wired to Board Controller
#define USART_BC_TXD_PIN_bm PIN3_bm ///< TxD pin bitmask

#define USART_BC_BSEL_9600   ((uint16_t)12) ///< BSEL for 9600 baud rate
#define USART_BC_BSCALE_9600 ((uint8_t)4)   ///< BSCALE for 9600 baud rate

#define USART_BC_ONE_STOP_BIT false ///< last parameter to USART_Format_Set()

/* Function Prototypes ********************************************************/

/** \name Library Initializer */ ///@{
void USART_BC_init(void);
///@}

#endif /* end of include guard: _USART_BC_H_ */

/** @} */ /* end of \defgoup usart_bc_api */
