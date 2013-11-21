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
 * For reception, use the USART_BC_RX_available() function to check for received
 * data, and the USART_BC_get_string() function to return the last string
 * received, NULL terminated. Note that when sending data to the board
 * controller from a host connected on the USB line, the data sent must be
 * terminated by a Line Feed (\n, ASCII 0x0A), or this library may hang
 * indefinately when returning the last string received.
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

/** \brief The null terminator for C strings */
#define NUL 0x00

/* Function Prototypes ********************************************************/

/** \name Library Initializer */ ///@{
void USART_BC_init(void);
///@}

bool USART_BC_RX_available(void);
void USART_BC_flush_RX_buffer(void);
bool USART_BC_get_string(char* buffer);

#endif /* end of include guard: _USART_BC_H_ */

/** @} */ /* end of \defgoup usart_bc_api */
