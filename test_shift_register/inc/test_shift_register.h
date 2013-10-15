#ifndef _TEST_SHIFT_REGISTER_H_
#define _TEST_SHIFT_REGISTER_H_

/* The board.h header file defines which IO ports peripherals like Switches and
 * LEDs are connected to. The header file is configured for use with XMEGA-A1
 * Xplained by default. */
#include "board.h"

/* contains some macros to make the code more similar for both compilers. Look
 * into the file for more details. Includes <avr/io.h>, which will pull correct
 * atmel header so long as -mmcu is defined on the compile line. */
#include "avr_compiler.h"

/**** function prototypes *****************************************************/

/** \brief Calls all the setup_* functions.
 *
 * should be called from main to set up the board. */
void setup(void);

/** \brief Sets up the LEDS.
 *
 * LEDS[0..3] read out the binary value of the current channel selection.
 * LEDS[4..7] will light up if a high value is read on the signal pin,
 * otherwise they are all off. */
void setup_leds(void);

/** \brief Sets up the shift register pins.
 *
 * pinout:
 *    - GPIO_1 (PD0) = SER_IN
 *    - GPIO_2 (PD1) = Clock
 *    - GPIO_3 (PD2) = L_Clock  */
void setup_SR_pins(void);

/** \brief Sets up the signal reading pin.
 *
 * GPIO_4 (PD3) is used to read the value on the signal pin of
 * the multiplexer.   */
void setup_signal_pin(void);

/** \brief Sets up the switches.
 *
 * Switch 7 cycles through multiplexer chanels 0..15 */
void setup_switches(void);


/** \brief Sets LEDS[0..3] to the binary value of the selected channel.
 *
 * \param channel which channel is currently selected */
void show_channel(int8_t channel);

/** \brief Selects the multiplexer channel.
 *
 * This sets the shift register to select the given channel on the multiplexer.
 * pinout:
 *    - GPIO_1 (PD0) = SER_IN
 *    - GPIO_2 (PD1) = Clock
 *    - GPIO_3 (PD2) = L_Clock
 *
 * \param channel which channel to select   */
void set_channel(int8_t channel);

/** \brief LEDS[4..7] will be lit if signal evaluates to true.
 *
 * \param signal_present whether a signal is present.  */
void show_signal(uint8_t signal_present);

#endif /* end of include guard: _TEST_SHIFT_REGISTER_H_ */

