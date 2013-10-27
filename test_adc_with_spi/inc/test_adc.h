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
#include "clksys_driver.h"
#include "spi_driver.h"
#include "port_driver.h"

/**** function prototypes *****************************************************/

typedef enum multiplexer_select { mp0, mp1 } mp_select_t;

/** \brief Calls all the setup_* functions.
 *
 * should be called from main to set up the board.
 *
 * \param SPI_master reference to SPI_master_t to use */
void setup(void);

/** \brief Initialize and set cpu and periheral clocks.
 *
 * -CPU: 32HMZ
 * -Peripherals
 *    -none */
void setup_clocks(void);

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
void setup_SR(void);

/** \brief Sets up the switches.
 *
 * Switch 7 cycles through multiplexer chanels 0..15 */
void setup_switches(void);

/** \brief sets up the ADC
 *
 * Enables the SPI interface to communicate with the external ADC
 *
 * \param SPI_master the SPI_master_t to use
 *
 * \note Using and Analog Devices AD7892ANZ-1  */
void setup_ADC(void);

/** \brief Selects the multiplexer channel.
 *
 * This sets the shift register to select the given channel on the multiplexer.
 * pinout:
 *    - GPIO_1 (PD0) = SER_IN
 *    - GPIO_2 (PD1) = Clock
 *    - GPIO_3 (PD2) = L_Clock
 *
 * \param channel which channel to select   */
void set_channel(mp_select_t mp_select, uint8_t channel);

/** \brief Shows upper 8 bit of ADC result on LEDS[0..7]
 *
 * \param result the upper 8 bits of the ADC result */
void show_result(uint8_t result);

#endif /* end of include guard: _TEST_SHIFT_REGISTER_H_ */

