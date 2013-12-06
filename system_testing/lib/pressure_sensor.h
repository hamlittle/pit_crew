/** \defgroup pressure_sensor Pressure Sensor Library API */ /** @{ */
/** \file pressure_sensor.h
 *
 * \brief Enables interfacing with the matrix pressure sensor.
 *
 * This library simplifies interfacing with the Sensing Tex 672 element matrix
 * pressure sensor. Currently, only full scans of the pressure sensor are
 * enabled, there is no control over which elements are sampled.
 *
 * To use this library, First a calibration run must be initiated. Without
 * a calibration run, the results from this library cannot be trusted. Once the
 * pressure sensor has been calibrated, full sweep scans can be initiated. This
 * returns a scan buffer, which holds the results of the ADC conversion result
 * across each of the test resistors on board 1. The buffer is a matrix with the
 * same dimensions as the pressure sensor (24 y-position, 28 x-position
 * elements). The format of each element of the scan buffer is as follows, from
 * MSB to LSB
 *    - 4 leading 0's
 *    - 1 sign bit
 *    - 11 bit conversion value (range: 0-2048)
 *
 * This library uses 3 buffers when interfacing with the sensor. The
 * compensation buffer is populated by a call to PS_calibrate(). This buffer is
 * subtracted from the values read during a call to PS_scan_all(), and these
 * compensated values are saved in the scan buffer. Finally, the check buffer
 * holds the compensated values of the previous scan. These values are used when
 * checking that no value exceeds the delta threshold during calls to
 * PS_check().
 *
 * Additionally, the format of the each buffer (compensation, scan, and check)
 * are as defined as:
 *    \code uint16_t buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS] \endcode
 * where \code buffer[y_pos][x_pos] \endcode returns the buffer element value at
 * the given <x,y> position on the sensor.
 *
 * The compensation and scan buffers are saved in the pressure_sensor_t struct
 * passed into each of the functions in this library. The conversion result
 * buffer can be retrieved by using the PS_get_scan_buffer() macro, and the
 * compensation buffer can similarly be retrieved using the
 * PS_get_compensation_buffer() macro. Note that each of these macros expect
 * a pointer to the Pressure sensor struct as a parameter.
 *
 * This library relies on the USART_BC library, and assumes that the
 * user has enabled it through a call to USART_BC_init() before any of the print
 * functions in this library are called.
 *
 * This library depends on a few interrupts which are set up
 * internally. ADC0 and ADC1 /EOC interrupts, as well as the SPI interrupt used
 * to communicate with the ADCs and SRs are taken by this library for
 * coordinating full sensor scan sweeps. Care must be taken that these
 * interrupts are not used by other parts of the code when using this library
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

#ifndef _PRESSURE_SENSOR_H_
#define _PRESSURE_SENSOR_H_

/* Include Directives *********************************************************/

#include "adc.h"
#include "usart_bc.h"

/* Macro Definitions **********************************************************/

#define PS_X_MIN 9  ///< min value along x-axis of sensor to check
#define PS_X_MAX 19 ///< max value along x-axis of sensor to check
#define PS_Y_MIN 9  ///< min value along y_axis of sensor to check
#define PS_Y_MAX 19 ///< min value along y_axis of sensor to check

#define NUM_PS_X_CHANS 28 ///< Number of x-position channels
#define NUM_PS_Y_CHANS 24 ///< Number of y-position channels

/// any reading, after compensated, less than this threshold is set to 0
#define ZERO_THRESHOLD 40        ///< zero threshold, \sa PS_scan_all()
#define OVERSAMPLE_SIZE 5        ///< oversample rate, \sa PS_scan_all()
#define OVERSAMPLE_THRESHOLD 1000 ///< oversample threshold, \sa PS_scan_all()

#define SIGN_BIT (0x0001 << 11)  ///< Sign bit of ADC conversion result

/* Typedefs, Enums, and Structs ***********************************************/

/** \brief Contains the static data needed to interface with the pressure
 * sensor.
 *
 * This struct is used as the first parameter to all of the PS_* functions in
 * this library */
typedef struct pressure_sensor_struct  {
   /// Compensation buffer storage area
   uint16_t compensation_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS];
   /// Sweep Scan buffer, adjusted using compensation_buffer
   uint16_t scan_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS];
   /// Save values from last run here to check against
   uint16_t check_buffer[NUM_PS_Y_CHANS][NUM_PS_X_CHANS];
} PS_t;

/** \name Macro Defined Functions *********************************************/
///@{

/** \brief Returns the scan buffer from the last scan performed.
 *
 * Note that the actual scan buffer is returned, not a copy. For data integrity
 * purposes, this buffer should be treated as read only.
 *
 * \param[in] _ps The pressure sensor to get scan buffer from
 * \return The scan buffer from the last sweep of the sensor, calibrated using
 * the result of the last calibration */
#define PS_get_scan_buffer(_ps) ((_ps)->scan_buffer)

/** \brief Returns the compensation buffer from the last scan performed.
 *
 * Note that the actual compensation buffer is returned, not a copy. For data
 * integrity purposes, this buffer should be treated as read only.
 *
 * \param[in] _ps The pressure sensor to get compensation buffer from
 * \return the compensation buffer from the last sensor calibration */
#define PS_get_compensation_buffer(_ps) ((_ps)->compensation_buffer)

///@}

/* Function Prototypes ********************************************************/

/** \name Libray Setup Functions */
///@{
void PS_init(PS_t *pressure_sensor);
///@}

void PS_calibrate(PS_t *pressure_sensor);
void PS_scan_all(PS_t *pressure_sensor);
void PS_print_scan_buffer(PS_t *pressure_sensor);
void PS_print_compensation_buffer(PS_t *pressure_sensor);
bool PS_check(PS_t *pressure_sensor, uint16_t abs_threshold,
              uint16_t delta_threshold);

#endif /* end of include guard: _PRESSURE_SENSOR_H_ */
/** @} */ /* end of \defgoup pressure_sensor */
