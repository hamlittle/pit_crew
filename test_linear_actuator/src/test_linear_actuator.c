/** \file test_linear_actuator.c
 *
 * \brief System software for testing the linear actuators.
 *
 * This test follows directly from AVR446: Linear Speed Control of Stepper
 * Motors. This test is simply a port of the code provided with this App Note to
 * the Xmega Platform. On startup, a help menu is presented which explains how
 * to run the tests.
 *
 * \todo TODO test_linear_actuator.c documentation
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

/* Include Directives *********************************************************/

#include "test_linear_actuator.h"

/* Global Data ****************************************************************/

/* \brief The motor we are controlling */
SM_t motor;

/* Function Definitions *******************************************************/

/** \brief Initialize and set cpu and periheral clocks.
 *
 * CPU clock frequencies set are:
 * -CPU: 32HMZ
 * -Peripheral Prescaling: NONE */
static void setup_clocks(void) {

   // set 32MHZ oscillator as CPU clock source
   CLKSYS_Enable(OSC_RC32MEN_bm);                          // enable
   do { nop(); } while (!CLKSYS_IsReady(OSC_RC32MRDY_bm)); // wait til stable
   CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);   // select for CPU

   // disable all presacalers, until we decide otherwise
   CLKSYS_Prescalers_Config(CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);

   // set up external 32KHz oscillator (NOTE: first param is ignored)
   CLKSYS_XOSC_Config(OSC_FRQRANGE_04TO2_gc, false, OSC_XOSCSEL_32KHz_gc);

   // set internal 32KHz oscillator as source for DFLL and autocalibrate 32MHz
   CLKSYS_Enable(OSC_XOSCEN_bm);                          //enable
   do { nop(); } while (!CLKSYS_IsReady(OSC_XOSCRDY_bm)); // wait til stable
   CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, true); // true == ext 32KHz

   // disable unused oscillators (internal 2MHz and 32KHz oscillators)
   CLKSYS_Disable(OSC_RC2MEN_bm | OSC_RC32KEN_bm);
}

/** \brief Sets up the LEDS.
 *
 * Sets the pins on LED_PORT to inverted output, so setting the corresponding
 * pin will turn on the LED. Clear the corresponding pin to turn the given LED
 * off. */
static void setup_LEDs(void) {
   LED_PORT.DIR = 0xff;                 // set all pins of port E to output
   PORTCFG.MPCMASK = 0xff;             // set for all pins on port E...
   LED_PORT.PIN0CTRL |= PORT_INVEN_bm;  // inverted output (set hi turns on led)
   LED_PORT.OUTCLR = 0xff;              // turn all leds off
}

/** \brief Sets up the switches.
 *
 * Switches on the board are normally open, and pull to ground when pushed.
 * Inverted input is enabled, so the IN register will read high if the switch is
 * pushed.
 *
 * \param[in] switch_mask which switches to enable. */
static void setup_switches(uint8_t switch_mask) {
   uint8_t maskL = switch_mask & SWITCH_PORTL_MASK_gm;
   uint8_t maskH = switch_mask >> SWITCH_PORTH_OFFSET;

   SWITCH_PORTL.DIR &= ~maskL;
   PORTCFG.MPCMASK = maskL;
   SWITCH_PORTL.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm;

   SWITCH_PORTH.DIR &= ~maskH;
   PORTCFG.MPCMASK = maskH;
   SWITCH_PORTH.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm;}

   /** \brief Initializes the pressure sensor library.
    *
    * This allows for easy interfacing with the sensor through the PS_* library
    * calls.
    *
    * \param[in] pressure_sensor The pressure sensor to initialize */
   static void setup_pressure_sensor(PS_t *pressure_sensor) {
      PS_init(pressure_sensor);
   }

/** \brief Sets up the USART connection with the Board Controller.
 *
 * Redirects stdout to the UART channel connected to the Board Controller, so
 * that printf() calls will be transferred over this connection. The Board
 * Controller acts as a UART-USB bridge by default, so any transmissions on
 * this channel are translated and sent over the USB connection. This allows
 * any host machine connected to the USB line on the board to monitor these
 * transmissions using a Serial Port terminal program, such as RealTerm.
 *
 * This set up is performed by the usart_bc.h library */
static void setup_USART_BC(void) {
   USART_BC_init();
}

/// \todo TODO document setup_motor()
static void setup_motor() {
   SM_init(&motor, &SM_port, SM_DISABLE_bm, SM_DIRECTION_bm, SM_STEP_bm,
           SM_TIMER);
   SM_enable(&motor);
}

/** \brief main loop to run tests.
 *
 * The tests being run are described in the documentation of this file.
 *
 * \return never returns, test loop runs ad infinitum. */
int main(void) {
   uint8_t switch_mask = PIN6_bm;

   int16_t position = 0;
   int16_t steps = 1000;
   uint16_t acceleration = 100;
   uint16_t deceleration = 100;
   uint16_t speed = 800;

   PS_t pressure_sensor;

   bool cmd_ok = false;
   char command_buffer[USART_RX_BUFFER_SIZE];

   /* call all of the setup_* functions */
   cli();
   setup_clocks();
   setup_LEDs();
   setup_switches(switch_mask);
   setup_pressure_sensor(&pressure_sensor);
   setup_USART_BC();
   setup_motor();
   sei();

   /* shows the help menu */
   show_help_message();

   /* show the current state of the linear actuator */
   show_motor_data(position, acceleration, deceleration, speed, steps);

   while (1) {
      cmd_ok = false;
      // If a command is received, check the command and act on it.
      if (USART_BC_get_string(command_buffer)) {
         if (command_buffer[0] == 'm') {
            // Move with...
            if (command_buffer[1] == ' '){
               // ...number of steps given.
               steps = atoi((const char *)command_buffer+2);
               SM_move(&motor, steps, acceleration, deceleration, speed);
               position += steps;
               cmd_ok = true;
               printf("\n\r  ");
            }
            else if (command_buffer[1] == 'o'){
               if (command_buffer[2] == 'v'){
                  if (command_buffer[3] == 'e'){
                     // ...all parameters given
                     if (command_buffer[4] == ' '){
                        int i = 6;
                        steps = atoi((const char *)command_buffer+5);
                        while ((command_buffer[i]!=' ') &&
                               (command_buffer[i]!='\n')) {
                           i++;
                        }
                        i++;
                        acceleration = atoi((const char *)command_buffer+i);
                        while ((command_buffer[i]!=' ') &&
                               (command_buffer[i]!='\n')) {
                           i++;
                        }
                        i++;
                        deceleration = atoi((const char *)command_buffer+i);
                        while ((command_buffer[i]!=' ') &&
                               (command_buffer[i]!='\n')) {
                           i++;
                        }
                        i++;
                        speed = atoi((const char *)command_buffer+i);
                        SM_move(&motor, steps, acceleration,
                                deceleration, speed);
                        position += steps;
                        cmd_ok = true;
                        printf("\n\r  ");
                     }
                  }
               }
            }
         }
         else if (command_buffer[0] == 'a'){
            // Set acceleration.
            if (command_buffer[1] == ' '){
               acceleration = atoi((const char *)command_buffer+2);
               cmd_ok = true;
            }
         }
         else if (command_buffer[0] == 'd'){
            // Set deceleration.
            if (command_buffer[1] == ' '){
               deceleration = atoi((const char *)command_buffer+2);
               cmd_ok = true;
            }
         }
         else if (command_buffer[0] == 's'){
            if (command_buffer[1] == ' '){
               speed = atoi((const char *)command_buffer+2);
               cmd_ok = true;
            }
         }
         else if (command_buffer[0] == '\r'){  // hyper terminal sends \r\n
            SM_move(&motor, steps, acceleration, deceleration, speed);
            position += steps;
            cmd_ok = true;
         }
         else if (command_buffer[0] == '?'){
            show_help_message();
            cmd_ok = true;
         }

         // Send help if invalid command is received.
         if (!cmd_ok) {
            show_help_message();
         }
         else {
            while (motor.speed_ramp.run_state != STOP) {
               if (READ_SWITCHES & PIN6_bm) {
                  SM_brake(&motor);
                  printf("Motor parked\n");
               }
               printf("Running... Steps Left: %d\n",
                      steps - motor.speed_ramp.step_count);
               delay_ms(250);
            }

            printf("Done with move\n");
         }

         // Clear RXbuffer.
         USART_BC_flush_RX_buffer();

         show_motor_data(position, acceleration, deceleration, speed, steps);
      }//end if (cmd)

   }//end while (1)
}

/*! \brief Sends help message.
 *
 *  Outputs help message.
 */
static void show_help_message(void)
{
   printf("%s", help_message);
}

/*! \brief Sends out data.
 *
 *  Outputs the values of the data you can control by serial interface
 *  and the current position of the stepper motor.
 *
 *  \param acceleration Accelration setting.
 *  \param deceleration Deceleration setting.
 *  \param speed Speed setting.
 *  \param steps Position of the stepper motor.
 */
static void show_motor_data(int16_t position, uint16_t acceleration,
                            uint16_t deceleration, uint16_t speed,
                            int16_t steps) {
   printf("\n\r Motor pos: %d    a: %d    d: %d    s: %d    m: %d\n\r>",
          position, acceleration, deceleration, speed, steps);
}
