/*************************************************************************
 *
 *
 *              Board Definitions
 *
 *
 *************************************************************************/

#ifndef __XMEGA_A1_XPLAINED__
#define __XMEGA_A1_XPLAINED__


#define LEDPORT           PORTE ///< LED port
#define LEDPORT_CHAN1_off 4     ///< Multiplexer0 = LED[4..7]
#define CHANNEL0_LEDS_gm  PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm ///< chan 0 LEDs
#define CHANNEL1_LEDS_gm  PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm ///< chan 1 LEDs

#define SR_PORT           PORTD    ///< Shift Register Port
#define SR_SER_IN_PIN_bm  PIN0_bm  ///< Shift Register SER_IN pin: PD0
#define SR_CLOCK_PIN_bm   PIN1_bm  ///< Shift Register CLOCK pin: PD1
#define SR_L_CLOCK_PIN_bm PIN2_bm  ///< Shift Register L_CLOCK pin: PD2
#define SR_PINS_gm        SR_SER_IN_PIN_bm | SR_CLOCK_PIN_bm | SR_L_CLOCK_PIN_bm

/* Unused in this test, signal routed from MP1 to ADC Vin1 */
/* #define SIGNAL_PORT    PORTD    ///< Signal port */
/* #define SIGNAL0_PIN_bm 0x08     ///< Signal Pin 0: PORTD[3] */
/* #define SIGNAL1_PIN_bm 0x10     ///< Signal Pin 0: PORTD[4] */
/* #define SIGNAL_PINS_gm (SIGNAL0_PIN_bm | SIGNAL1_PIN_bm) */
/* #define SIG0PINCTRL    PIN3CTRL ///< Signal pin CTRL sfr */
/* #define SIG1PINCTRL    PIN4CTRL ///< Signal pin CTRL sfr */

#define SWITCHPORTL         PORTD      ///< Switch Port [0..5]
#define SWITCHPORTH         PORTR      ///< Switch Port [6..7]
#define SWITCHPORTL_MASK_gm 0x3F       ///< Pin 0-5 (on PORTD)
#define SWITCHPORTH_MASK_gm 0x03       ///< Pin 0-1 (on PORTR)
#define CYCLE_SWITCH0_bm    PIN0_bm    ///< Cycle 0 channel select pin: PORTR[0]
#define CYCLE_SWITCH1_bm    PIN1_bm    ///< Cycle 1 channel select pin: PORTR[1]
#define CYCLE_SWITCHES_gm   (CYCLE_SWITCH0_bm | CYCLE_SWITCH1_bm)

#define CHANSEL0SWCTRL      PIN0CTRL   ///< Cycle 0 channel select pin CTRL sfr
#define CHANSEL1SWCTRL      PIN1CTRL   ///< Cycle 1 channel select pin CTRL sfr

#define SPI_PORT       PORTC    ///< SPI port
#define SPI_SS_PIN     PIN4_bm     ///< SPI /ss pin: PC[4]
#define SPI_SS_PINCTRL PIN4CTRL ///< SPI /ss PINxCTRL sfr
#define SPI_module     SPIC     ///< SPI module to use (PORTC)
#define SPI_INT_vect   SPIC_INT_vect ///< SPI interrupt vector

#define ADC_PORT         PORTC           ///< ADC port
#define ADC_CONVST_PIN   PIN0_bm         ///< /CONVST pin (pull low to start)
#define ADC_EOC_PIN      PIN1_bm         ///< /EOC pin (pulled low when ready)
#define ADC_EOC_PINCTRL  PIN1CTRL        ///< /EOC pin CTRL sfr
#define ADC_EOC_INT_VECT PORTC_INT0_vect ///< ADC /EOC interrupt vector
#define ADC_CON_BYTES    2               ///< ADC conversion result size


#define LINES_PER_MP 4 /// 16 channels per mp [0..15] = 4 bits to select

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
