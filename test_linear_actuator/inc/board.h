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

#define READ_SWITCHES ((SWITCHPORTL.IN & SWITCHPORTL_MASK_gm) | \
                       (SWITCHPORTH.IN << SWITCHPORTH_OFFSET))

// Pin 0-1 on PORTR are used to represent Switch 6-7,
// in order to position the bits rigth in a byte they
// need to be shifted with an offset

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
