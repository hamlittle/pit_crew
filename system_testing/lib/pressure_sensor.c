/** \file pressure_sensor.c
 *
 * \brief Enables interfacing with the matrix pressure sensor.
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

#include "pressure_sensor.h"

/* Macro Definitions **********************************************************/
/** \name Wiring Specific Definitions */ ///@{
#define SPI_PORT     PORTF         ///< SPI bus port
#define SPI_MODULE   SPIF          ///< SPI bus module
#define SPI_SS0_bm   PIN2_bm       ///< SPI SS0 pin
#define SPI_SS1_bm   PIN3_bm       ///< SPI SS1 pin
#define SPI_INT_VECT SPIF_INT_vect ///< SPI bus interrupt vector

#define ADC0_CTRL_PORT    PORTD           ///< ADC0 control port
#define ADC0_CONVST_bm    PIN1_bm         ///< /CONVST pin (pull low to start)
#define ADC0_EOC_bm       PIN3_bm         ///< /EOC pin (pulled low when ready)
#define ADC0_EOC_INT_VECT PORTD_INT0_vect ///< ADC0 /EOC interrupt vector

#define ADC1_CTRL_PORT    PORTF           ///< ADC1 control port
#define ADC1_CONVST_bm    PIN1_bm         ///< /CONVST pin (pull low to start)
#define ADC1_EOC_bm       PIN0_bm         ///< /EOC pin (pulled low when ready)
#define ADC1_EOC_INT_VECT PORTF_INT0_vect ///< ADC1 /EOC interrupt vector
///@}

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief The channels which can be selected on the x position multiplexer.
 *
 * MPx0 and MPx1 follow eachother, in that they will always be selecting the
 * same channel. This is just for convenience, and is not required by the
 * system.
 *
 * \note Because the ADC conversion result is 2 bytes, two bytes must be
 * transferred out on the SPI bus to control the MPx channel selected. In this
 * case, the first value sent out will be shifted out, and the second byte sent
 * will become the new channel selected. Therefore, each of the channel select
 * values are offset by from their index in this array by 1. This allows the
 * user to use MPx_channel[0] to select channel 0, as 0x00 is the second byte
 * transferred. The same goes for MPx_channel[1] to select channel 1, as 0x11
 * is the second byte transferred, and so on, for every channel on MPx0 and
 * MPx1. */
const uint8_t MPx_channels[(NUM_PS_X_CHANS / 2) + 1] =
{ 0x00,
   0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77,
   0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 };

/** \brief The channels which can be selected on the y position multiplexer.
 *
 * Unlike MPx, MPy0 and MPy1 are independant, and only 1 should be turned on at
 * any given time. in order to facilitate this, When the channel being selected
 * is in the range of MPy0, MPy1 is set to channel 15, which is disconnected,
 * and vice versa.
 *
 * Additionally, the upper 4 bits are mapped to MPy0, and the lower 4 to MPy1,
 * because the ADC is set up to transfer from MSB->LSB, so we must mirror this
 * with the Multiplexers, and the upper 4 bits go to the lower multiplexer, and
 * vice versa.
 *
 * \note Because the ADC conversion result is 2 bytes, two bytes must be
 * transferred out on the SPI bus to control the MPy channel selected. In this
 * case, the first value sent out will be shifted out, and the second byte sent
 * will become the new channel selected. Therefore, each of the channel select
 * values are offset by from their index in this array by 1. This allows the
 * user to use MPx_channel[0] to select channel 0, as 0x00 is the second byte
 * transferred. The same goes for MPx_channel[1] to select channel 1, as 0x11
 * is the second byte transferred, and so on, for every channel on MPy0 and
 * MPy1. */
const uint8_t MPy_channels[NUM_PS_Y_CHANS + 1] =
{ 0x00,
   0x0F, 0x1F, 0x2F, 0x3F, 0x4F, 0x5F, 0x6F, 0x7F, 0x8F, 0x9F, 0xAF, 0xBF,
   0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB };

/** \brief The two ADC's being used */
typedef enum ADC_selection {
   ADC0, ///< ADC0 is connected to MPx0
   ADC1  ///< ADC1 is connected to MPx1
} ADC_sel_t;

/* Global Variables ***********************************************************/

/** \brief ADC0 connected to MPx0*/
static ADC_ext_t adc0;

/** \brief ADC1 connected to MPx1*/
static ADC_ext_t adc1;

/** \brief the ADC which is currently undergoing an SPI transfer to receive the
 * conversion result */
volatile ADC_sel_t current_ADC;

/* Function Prototypes ********************************************************/

static void sweep_sensor(uint16_t buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS],
                         uint16_t comp_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS]);
static void print_buffer(uint16_t buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS],
                         uint16_t check_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS]);
static uint16_t get_min(uint16_t *oversample_buffer);
static uint16_t get_max(uint16_t *oversample_buffer);

/* Function Definitions * *****************************************************/

/** \brief Initializes the library.
 *
 * Sets up the Multiplexers, Shift Registers, and ADCs needed to interface with
 * the pressure sensor.
 *
 * \note Don't forget to calibrate the sensor once before using the
 * PS_scan_all() function
 *
 * \param[in] pressure_sensor the pressure sensor to initialize */
void PS_init(PS_t *pressure_sensor) {
   uint8_t y_channel, x_channel;

   for (y_channel = 0; y_channel < NUM_PS_Y_CHANS; ++y_channel) {
      for (x_channel = 0; x_channel < NUM_PS_X_CHANS; ++x_channel) {
         pressure_sensor->compensation_buffer[y_channel][x_channel] = 0;
         pressure_sensor->scan_buffer[y_channel][x_channel] = 0;
         pressure_sensor->scan_buffer[y_channel][x_channel] = 0;
      }
   }

   ADC_init(&adc0, &ADC0_CTRL_PORT, ADC0_CONVST_bm, ADC0_EOC_bm,
            &SPI_PORT, &SPI_MODULE, SPI_SS0_bm);
   ADC_init(&adc1, &ADC1_CTRL_PORT, ADC1_CONVST_bm, ADC1_EOC_bm,
            &SPI_PORT, &SPI_MODULE, SPI_SS1_bm);
}

/** \brief Calibrates the pressure sensor for future scans.
 *
 * The compensation values generated are saved internally in the given
 * pressure_sensor parameter, and used in future PS_scan_all() calls. To get
 * direct access to the compensation buffer, use PS_get_compensation_buffer
 *
 * To see the results of the compensation buffer, use the
 * PS_get_compensation_buffer() macro.
 *
 * \note Scanning is performed by synchronizing the order of ADC conversions to
 * adjust the sensor element being scanned appropriately (to change MPy
 * channel, read from ADC1, and MPx channel is updated when reading from ADC0.
 * This is because both ADCs and both MPs are on the same SPI bus, but SS0
 * controls ADC0 and MPx, and SS1 controls ADC1 and MPy).
 *
 * \note This function is blocking while the calibration is being performed, and
 * should not be called during a time critical routine, as this function samples
 * all 672 pressure sensor elements.
 *
 * \param[in] pressure_sensor the pressure sensor to calibrate */
void PS_calibrate(PS_t *pressure_sensor) {
   sweep_sensor(pressure_sensor->compensation_buffer, NULL);

   uint8_t y_ndx, x_ndx;
   for (y_ndx = 0; y_ndx < NUM_PS_Y_CHANS; ++y_ndx) {
      for (x_ndx = 0; x_ndx < NUM_PS_X_CHANS; ++x_ndx) {
         pressure_sensor->check_buffer[y_ndx][x_ndx] = 0;
         pressure_sensor->scan_buffer[y_ndx][x_ndx] = 0;
      }
   }
}

/** \brief brief description
 *
 * The conversion result is saved internally in the pressure_sensor parameter,
 * and adjusted according to the last calibration performed on the pressure
 * sensor. To get access to the results, use PS_get_scan_buffer()
 *
 * \note Scanning is performed by synchronizing the order of ADC conversions to
 * adjust the sensor element being scanned appropriately (to change MPy
 * channel, read from ADC1, and MPx channel is updated when reading from ADC0.
 * This is because both ADCs and both MPs are on the same SPI bus, but SS0
 * controls ADC0 and MPx, and SS1 controls ADC1 and MPy).
 *
 * \note This function is blocking while the scan is being performed, and should
 * not be called during a time critical routine, as this function samples all
 * 672 pressure sensor elements.
 *
 * \param[in] pressure_sensor pressure sensor to scan and compensate */
void PS_scan_all(PS_t *pressure_sensor) {
   uint8_t y_ndx, x_ndx;
   for (y_ndx = 0; y_ndx < NUM_PS_Y_CHANS; ++y_ndx) {
      for (x_ndx = 0; x_ndx < NUM_PS_X_CHANS; ++x_ndx) {
         pressure_sensor->check_buffer[y_ndx][x_ndx] =
            pressure_sensor->scan_buffer[y_ndx][x_ndx];
      }
   }

   sweep_sensor(pressure_sensor->scan_buffer,
                pressure_sensor->compensation_buffer);

}

/** \brief Prints the results of a full sensor conversion scan.
 *
 * this function prints the results of a compensated full sensor scan sweep to
 * a terminal window using the USART-USB gateway on the development board.
 * This requires the USART-USB gateway to have been properly configured. The
 * order in which elements are printed matches the position of the reading on
 * the sensor, where <0,0> is the upper left corner of the sensor.
 *
 * The printed data are in the range of 0-2048, where a higher number indicated
 * a higher pressure on the sensor.
 *
 * \param[in] pressure_sensor the pressure sensor whose results to print */
void PS_print_scan_buffer (PS_t *pressure_sensor) {
   print_buffer(PS_get_scan_buffer(pressure_sensor),
                pressure_sensor->check_buffer);
}

/** \brief Prints the contents of the compensation buffer.
 *
 * This function prints the results of the compensation buffer to a terminal
 * window using the USART-USB gateway on the development board. This requires
 * the USART-USB gateway to have been properly configured. The order in which
 * elements are printed matches the position of the reading onthe sensor, where
 * <0,0> is the upper left corner of the senor.
 *
 * The printed data are in the range of 0-2048, where a higher number indicates
 * a higher pressure on the sensor. In reality, the values of the compensation
 * buffer should ideally all be 0, although they never are.
 *
 * \param[in] pressure_sensor whose compensation buffer to print */
void PS_print_compensation_buffer(PS_t *pressure_sensor) {
   print_buffer(PS_get_compensation_buffer(pressure_sensor), NULL);
}

/** \brief Checks the last scan of the sensor for any values above the given
 *    thresholds.
 *
 * This can be used as a convenience for seeing if anything hard has been
 * detected in the peach. The absolute threshold is the absolute value, which if
 * any value in the scan buffer exceeds, will trigger pit detection. The delta
 * threshold is the value by which no single element of the scan buffer can have
 * increased from the scan before the most recent without triggering a pit
 * detection.
 *
 * \param[in] pressure_sensor The pressure sensor to check
 * \param[in] abs_threshold The absolute threshold to check
 * \param[in] delta_threshold The change from last reading threshold
 *
 * \return true if any value was above the threshold, otherwise false */
bool PS_check(PS_t *pressure_sensor, uint16_t abs_threshold, uint16_t
              delta_threshold) {
   uint8_t x_ndx, y_ndx;

   for (y_ndx = PS_Y_MIN; y_ndx < PS_Y_MAX; ++y_ndx) {
      for (x_ndx = PS_X_MIN; x_ndx < PS_X_MAX; ++x_ndx) {
         if (pressure_sensor->scan_buffer[y_ndx][x_ndx] >= abs_threshold) {
            return true;
         }
      }
   }

   for (y_ndx = PS_Y_MIN; y_ndx < PS_Y_MAX; ++y_ndx) {
      for (x_ndx = PS_X_MIN; x_ndx < PS_X_MAX; ++x_ndx) {
         if (pressure_sensor->scan_buffer[y_ndx][x_ndx] >
             pressure_sensor->check_buffer[y_ndx][x_ndx]) {
            if (pressure_sensor->scan_buffer[y_ndx][x_ndx] -
                pressure_sensor->check_buffer[y_ndx][x_ndx]
                >= delta_threshold) {
               return true;
            }
         }
      }
   }

   return false;
}

/* Private Function Definitions ***********************************************/

/** \brief Sweeps the pressure sensor, and compensates the results.
 *
 * If performing a calibration sweep, pass NULL for the second parameter, and
 * no compensation will be performed.
 *
 * \param[in] buffer buffer to save the sensor sweep results
 * \param[in] comp_buffer compensation values to use, or NULL to not use any */
static void sweep_sensor(uint16_t buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS],
                         uint16_t comp_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS]) {
   uint8_t y_channel, x_channel;
   uint16_t oversample_buffer[OVERSAMPLE_SIZE];
   uint16_t result;
   uint16_t comp = ZERO_THRESHOLD;
   uint16_t min;
   uint16_t max;
   uint8_t oversample_ndx;

   /* start at position 0,0 */
   ADC_reset_channel(&adc0);
   ADC_reset_channel(&adc1);

   for (y_channel = 0; y_channel < NUM_PS_Y_CHANS; ++y_channel) {

      /* set the y channel, but don't use the conversion result */
      ADC_set_output_data(&adc1, MPy_channels+y_channel);
      ADC_sample_once(&adc1);

      for (x_channel = 0; x_channel < (NUM_PS_X_CHANS / 2); ++x_channel) {
         /* set the x_channel, but dont use the conversion result */
         ADC_set_output_data(&adc0, MPx_channels+x_channel);
         ADC_sample_once(&adc0);
         /* delay_ms(5); */

         comp = ZERO_THRESHOLD;

         for (oversample_ndx = 0; oversample_ndx < OVERSAMPLE_SIZE;
              ++oversample_ndx) {
            oversample_buffer[oversample_ndx] = ADC_sample_once(&adc1);
            if (oversample_buffer[oversample_ndx] & SIGN_BIT) {
               oversample_buffer[oversample_ndx] = 0;
            }
         }
         min = get_min(oversample_buffer);
         max = get_max(oversample_buffer);
         if ((max - min > OVERSAMPLE_THRESHOLD) && (comp_buffer != NULL)) {
            result = 0;
         }
         else {
            result = (max+min)/2;
         }

         if (comp_buffer != NULL) {
            comp = comp_buffer[y_channel][x_channel] + ZERO_THRESHOLD;
            if (comp > result) { /* if comp is gt result, set to 0  */
               result = 0;
               comp = ZERO_THRESHOLD;
            }
         }
         comp -=ZERO_THRESHOLD;
         buffer[y_channel][x_channel] = result-comp;

         for (oversample_ndx = 0; oversample_ndx < OVERSAMPLE_SIZE;
              ++oversample_ndx) {
            oversample_buffer[oversample_ndx] = ADC_sample_once(&adc0);
            if (oversample_buffer[oversample_ndx] & SIGN_BIT) {
               oversample_buffer[oversample_ndx] = 0;
            }
         }
         min = get_min(oversample_buffer);
         max = get_max(oversample_buffer);
         if ((max - min > OVERSAMPLE_THRESHOLD) && (comp_buffer != NULL)) {
            result = 0;
         }
         else {
            result = (max+min)/2;
         }

         if (comp_buffer != NULL) {
            comp = comp_buffer[y_channel][x_channel + (NUM_PS_X_CHANS/2)]
               + ZERO_THRESHOLD;
            if (comp > result) { /* if comp is gt result, set to 0  */
               result = 0;
               comp = ZERO_THRESHOLD;
            }
         }
         if (result & SIGN_BIT) {
            result = 0;
            comp = ZERO_THRESHOLD;
         }
         comp -= ZERO_THRESHOLD;
         buffer[y_channel][x_channel + (NUM_PS_X_CHANS)/2] = result-comp;
      }
   }
}

/** \brief Prints the contents of the given buffer.
 *
 * \note If this is the scan buffer, the results are already compensated.
 *
 * \param[in] buffer The pressure sensor scan buffer to print out
 * \param[in] check_buffer The check buffer to print out */
static void print_buffer(uint16_t buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS],
                         uint16_t check_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS]){
   int8_t y_channel, x_channel;

   for (y_channel = PS_Y_MIN; y_channel < PS_Y_MAX; ++y_channel) {
      printf("\n");
      for (x_channel = PS_X_MIN; x_channel < PS_X_MAX ; ++x_channel) {
         printf("%4u", buffer[y_channel][x_channel]);
         if (check_buffer != NULL) {
            if (buffer[y_channel][x_channel] >
                check_buffer[y_channel][x_channel]) {
               printf("(%3u)", buffer[y_channel][x_channel] -
                      check_buffer[y_channel][x_channel]);
            }
            else {
               printf("(  0)");
            }
         }
         printf(" ");
      }
   }
}

/** \brief Returns the min value from the given oversample_buffer.
 *
 * \param[in] oversample_buffer the oversample buffer to get the min value
 *
 * \return Minimum value of the oversample buffer */
static uint16_t get_min(uint16_t *oversample_buffer) {
   uint16_t min = *oversample_buffer;
   uint8_t ndx;

   for (ndx = 0; ndx < OVERSAMPLE_SIZE-1; ++ndx) {
      if (*(oversample_buffer+ndx+1) < *(oversample_buffer+ndx)) {
         min = *(oversample_buffer+ndx+1);
      }
   }

   return min;
}

/** \brief Returns the max value from the given oversample_buffer.
 *
 * \param[in] oversample_buffer the oversample buffer to get the max value
 *
 * \return maximum value of the oversample buffer */
static uint16_t get_max(uint16_t *oversample_buffer) {
   uint16_t max = *oversample_buffer;
   uint8_t ndx;

   for (ndx = 0; ndx < OVERSAMPLE_SIZE-1; ++ndx) {
      if (*(oversample_buffer+ndx+1) > *(oversample_buffer+ndx)) {
         max = *(oversample_buffer+ndx+1);
      }
   }

   return max;
}

/** \brief ADC /EOC interrupt vector (PORT.INT0)
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system.  */
ISR(ADC0_EOC_INT_VECT) {
   current_ADC = ADC0;
   ADC_EOC_interrupt_handler(&adc0);
}

/** \brief ADC /EOC interrupt vector (PORT.INT0)
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system.  */
ISR(ADC1_EOC_INT_VECT) {
   current_ADC = ADC1;
   ADC_EOC_interrupt_handler(&adc1);
}

/** \brief ADC SPI interrupt
 *
 * This interrupt source was configured in the call to ADC_init(), but must be
 * registered by the user to allow for multiple ADCs to be used in the
 * system. */
ISR(SPI_INT_VECT) {
   if (current_ADC == ADC0) {
      ADC_SPI_interrupt_handler(&adc0);
   }
   else {
      ADC_SPI_interrupt_handler(&adc1);
   }
}
