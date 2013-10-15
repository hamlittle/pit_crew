/*************************************************************************
 *
 *
 *              Board Definitions
 *
 *
 *************************************************************************/

#ifndef __XMEGA_A1_XPLAINED__
#define __XMEGA_A1_XPLAINED__


#define LEDPORT PORTE ///< Port the LEDS are wired to
#define CHANNEL_LEDS_gm 0x0f ///< LEDs used to show channel select
#define SIGNAL_LEDS_gm 0xf0 ///< LEDs used to show signal present

#define SR_PORT PORTD ///< Shift Register Port
#define SR_PINS_gm 0x07 ///< Shift Register Port pins: PortD[0..2]
#define SR_SER_IN_PIN_bm 0x01 ///< Shift Register SER_IN pin: PD0
#define SR_CLOCK_PIN_bm 0x02 ///< Shift Register CLOCK pin: PD1
#define SR_L_CLOCK_PIN_bm 0x04 ///< Shift Register L_CLOCK pin: PD2

#define SIGNAL_PORT PORTD ///< Signal port
#define SIGNAL_PIN_bm 0x08 ///< Signal pin mask: PORTD[3]
#define SIGPINCTRL PIN3CTRL ///< Signal pin CTRL sfr

#define SWITCHPORTL PORTD ///< Switch Port [0..5]
#define SWITCHPORTH PORTR ///< Switch Port [6..7]
#define CYCLE_SWITCH_bm 0x02 ///< Cycle channel select pin: PORTR[1]
#define CHANSELSWCTRL PIN1CTRL ///< Cycle channel select pin CTRL sfr
#define SWITCHPORTL_MASK_gc	(0x3F<<0) // Pin 0-5
#define SWITCHPORTH_MASK_gc	(0x03<<0) // Pin 0-1

#define LEDPORT_TIMER0 TCE0
#define LEDPORT_AWEX AWEXE

#define READ_SWITCHES ((SWITCHPORTL.IN & SWITCHPORTL_MASK_gc) | \
                       (SWITCHPORTH.IN << SWITCHPORTH_OFFSET))


// Pin 0-1 on PORTR are used to represent Switch 6-7,
// in order to position the bits rigth in a byte they
// need to be shifted with an offset
#define SWITCHPORTH_OFFSET 6

#define TESTPORT PORTC

#define SWITCHPORT_INT0_vect PORTD_INT0_vect
#define SWITCHPORTH_INT0_vect PORTR_INT0_vect

//ADC Defines
#define ntc_enable()	(PORTB.PIN3CTRL = ((PORTB.PIN3CTRL & ~PORT_OPC_gm) | \
                                           PORT_OPC_PULLDOWN_gc))

//USART Defines
#define USART USARTC0
#define USART_PORT PORTC

#define USART_RXC_vect USARTC0_RXC_vect
#define USART_DRE_vect USARTC0_DRE_vect

#define DMA_CH_TRIGSRC_USART_DRE_gc DMA_CH_TRIGSRC_USARTC0_DRE_gc
#define DMA_CH_TRIGSRC_USART_RXC_gc DMA_CH_TRIGSRC_USARTC0_RXC_gc

//TWI Defines
#define TWI TWIC
#define TWIPORT PORTC

#define PRPTWI PRPD //The port not used for TWI

#define TWI_TWIM_vect TWIC_TWIM_vect
#define TWI_TWIS_vect TWIC_TWIS_vect

#endif // __XMEGA_A1_XPLAINED__
