/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief This file implements some macros that makes the IAR C-compiler and
 *        avr-gcc work with the same code base for the AVR architecture.
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 2772 $
 * $Date: 2009-09-11 12:40:26 +0200 (fr, 11 sep 2009) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef COMPILER_AVR_H
#define COMPILER_AVR_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef F_CPU
/*! \brief Define default CPU frequency, if this is not already defined. */
#define F_CPU 2000000UL
#endif

#if defined( __GNUC__ )

/*! \brief This macro will protect the following code from interrupts. */
#define AVR_ENTER_CRITICAL_REGION( ) uint8_t volatile saved_sreg = SREG; \

/*! \brief This macro must always be used in conjunction with
 * AVR_ENTER_CRITICAL_REGION
 *        so the interrupts are enabled again.
 */
#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;

/*! \brief Define the delay_us macro for GCC. */
#define delay_us( us )   ( _delay_us( us ) )

/*! \brief Define the delay_ms macro for GCC. */
#define delay_ms( ms )   ( _delay_ms( ms ) )

#define INLINE static inline

/*! \brief Define the no operation macro. */
#define nop()   do { __asm__ __volatile__ ("nop"); } while (0)

#else
#error Compiler not supported.
#endif

#endif

