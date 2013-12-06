/** \file board.h
 *
 * \brief Convenience definitions for the XMEGAA1 XPLAINED development board. */
#ifndef __XMEGA_A1_XPLAINED__
#define __XMEGA_A1_XPLAINED__

#define LED_PORT PORTE ///< LEDs are wired to PORTE[0..7]

#define SWITCH_PORTL         PORTD      ///< Switch Port [0..5]
#define SWITCH_PORTH         PORTR      ///< Switch Port [6..7]
#define SWITCH_PORTL_MASK_gm 0x3F       ///< Pin 0-5 (on PORTD)
#define SWITCH_PORTH_MASK_gm 0x03       ///< Pin 0-1 (on PORTR)
#define SWITCH_PORTH_OFFSET  6         ///< Switch Port [6..7] offset

/** \brief Read the switch values.
 *
 * Converts the switch values from PORTD[1..5] and PORTR[1..2] into a single
 * 8-bit value, with PORTR values shifted into the two MSB (PORTR[2] is MSB),
 * and similarly, puts the values from PORTD into the lower 6 bits (with
 * PORTD[5] being the highest bit of the lowest 6 bits of the returned value).
 *
 * \return 8-bit value, each bit representing the corresponding switch value */
#define READ_SWITCHES ((SWITCH_PORTL.IN & SWITCH_PORTL_MASK_gm) | \
                       (SWITCH_PORTH.IN << SWITCH_PORTH_OFFSET))
#endif // __XMEGA_A1_XPLAINED__
