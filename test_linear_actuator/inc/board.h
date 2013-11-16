/*************************************************************************
 *
 *
 *              Board Definitions
 *
 *
 *************************************************************************/

#ifndef __XMEGA_A1_XPLAINED__
#define __XMEGA_A1_XPLAINED__

#define LED_PORT PORTE ///< LEDs are wired to PORTE[0..7]

#define SWITCH_PORTL         PORTD      ///< Switch Port [0..5]
#define SWITCH_PORTH         PORTR      ///< Switch Port [6..7]
#define SWITCH_PORTL_MASK_gm 0x3F       ///< Pin 0-5 (on PORTD)
#define SWITCH_PORTH_MASK_gm 0x03       ///< Pin 0-1 (on PORTR)
#define SWITCH_PORTH_OFFSET  6         ///< Switch Port [6..7] offset

#define LEDPORT_TIMER0 TCE0
#define LEDPORT_AWEX AWEXE

#define READ_SWITCHES ((SWITCH_PORTL.IN & SWITCH_PORTL_MASK_gm) | \
                       (SWITCH_PORTH.IN << SWITCH_PORTH_OFFSET))
#endif // __XMEGA_A1_XPLAINED__
