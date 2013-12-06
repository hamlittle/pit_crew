/** \file system_testing.c
 *
 * \brief Main project for testing the fully operational system
 *
 * This file contains the main peach pit detection algorithm implemented in the
 * main loop. The main loop is broken up as a finite state machine. Each state
 * is described in detail in the following sections. In order to run the system,
 * a computer should be connected to the XMEGA A1 development board through the
 * USB line, and a terminal program set to communicate with the board at 9600
 * baud, no parity, 1 stop bit, and no flow control should be started in order
 * to command the machine.
 *
 * The finite state machine implemented in the main loop is documented in
 * \ref index.
 *
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

/* Include Directives*********************************************************/

#include "system_testing.h"

/* Function Prototypes ********************************************************/

/** \name Board Setup Functions */
///@{
static void setup_clocks(void);
static void setup_LEDs(void);
static void setup_safety_switch(void);
static void setup_retain_sensor(void);
static void setup_pressure_sensor(PS_t *pressure_sensor);
static void setup_USART_BC(void);
static void setup_linear_actuators(LA_t *needle_actuator, LA_t *ring_actuator);
///@}

INLINE void set_state(PC_t *machine, PC_state_t state);
static void show_help_message(void);
static void show_motor_data(const char *name, int16_t position,
                            uint16_t acceleration, uint16_t deceleration,
                            uint16_t speed, int16_t dist);
static void show_threshold_data(uint16_t abs_threshold,
                                uint16_t delta_threshold);
static command_t parse_command(int16_t *steps1, uint16_t *accel1,
                               uint16_t *decel1, uint16_t *speed1,
                               int16_t *steps2, uint16_t *accel2,
                               uint16_t *decel2, uint16_t *speed2,
                               uint16_t *abs_threshold,
                               uint16_t *delta_threshold);
static uint8_t find_next_param(char *command_buffer, uint8_t start_position);


/* Function Definitions*******************************************************/

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

   // set internal 32KHz oscillator as source for DFLL and autocalibrate
   // 32MHz
   CLKSYS_Enable(OSC_XOSCEN_bm);                          //enable
   do { nop(); } while (!CLKSYS_IsReady(OSC_XOSCRDY_bm)); //wait til stable
   CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, true); //true == ext 32KHz

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

/** \brief Sets up the safety switch.
 *
 * The safety switch is the contact switch between the box and the and the
 * board the machine is moutned on. The machine will not run if the box is not
 * in contact with the board. This chan be checked via the macro-defined
 * function SS_OPEN() */
static void setup_safety_switch(void)  {
   SS_PORT.DIRCLR = SS_PIN_bm;
   PORTCFG.MPCMASK = SS_PIN_bm;
   SS_PORT.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

/** \brief Sets up the Retaining Ring sensor.
 *
 * All three pins on the retaining ring sensor will read high when the peach is
 * retained. To check for proper retention, use the macro defined RETAINED(). */
static void setup_retain_sensor(void) {
   RS_PORT.DIRCLR = RS_PIN_gm;
   PORTCFG.MPCMASK = RS_PIN_gm;
   RS_PORT.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
}

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

/** \brief Initializes the linear actuators.
 *
 * Calls into the linear_actuator.c library to initialize two to linear
 * actuators.  The pin values and timers used for each motor are defined in
 * this file's header file.
 *
 * \param[in] needle_actuator The needle motor to initialize
 * \param[in] ring_actuator The ring motor to initialize */
static void setup_linear_actuators(LA_t *needle_actuator, LA_t *ring_actuator) {
   LA_init(needle_actuator, LA_NEEDLE_PITCH, &LA_port, LA_NEEDLE_DISABLE_bm,
           LA_NEEDLE_DIRECTION_bm, LA_NEEDLE_STEP_bm, LA_NEEDLE_TIMER);
   LA_init(ring_actuator, LA_RING_PITCH, &LA_port, LA_RING_DISABLE_bm,
           LA_RING_DIRECTION_bm, LA_RING_STEP_bm, LA_RING_TIMER);
   LA_enable(needle_actuator);
   LA_enable(ring_actuator);
}

/** \brief main loop to run tests.
 *
 * The tests being run are described in the documentation of this file.
 *
 * \return never returns, test loop runs ad infinitum. */
int main(void) {
   PC_t machine;

   int16_t steps1 = 0;
   int16_t steps2 = 0;
   uint16_t accel1 = LA_NEEDLE_DFLT_ACCEL;
   uint16_t accel2 = LA_RING_DFLT_ACCEL;
   uint16_t decel1 = LA_NEEDLE_DFLT_DECEL;
   uint16_t decel2 = LA_RING_DFLT_DECEL;
   uint16_t speed1 = LA_NEEDLE_DFLT_SPEED;
   uint16_t speed2 = LA_RING_DFLT_SPEED;

   bool move_made = false;
   /* bool stop_printed = false; */

   command_t command;
   bool matlab = false;
   uint16_t abs_threshold = PS_ABS_THRESHOLD_DFLT;
   uint16_t delta_threshold = PS_DELTA_THRESHOLD_DFLT;

   /* call all of the setup_* functions */
   cli();
   setup_clocks();
   setup_LEDs();
   setup_safety_switch();
   setup_retain_sensor();
   setup_pressure_sensor(&(machine.pressure_sensor));
   setup_USART_BC();
   setup_linear_actuators(&(machine.needle_carriage),
                          &(machine.retaining_ring));
   set_state(&machine, IDLE);
   sei();

   /* shows the help menu */
   show_help_message();

   /* show the current state of the linear actuator */
   show_motor_data(LA_needle_name,
                   LA_get_position(&(machine.needle_carriage)),
                   accel1, decel1, speed1, steps1);
   show_motor_data(LA_ring_name,
                   LA_get_position(&(machine.retaining_ring)),
                   accel2, decel2, speed2, steps2);
   show_threshold_data(abs_threshold, delta_threshold);

   /* perform an initial pressure sensor calibration */
   PS_calibrate(&machine.pressure_sensor);

   /* home both of the linear actuators */
   LA_go_to_home(&machine.needle_carriage);
   LA_go_to_home(&machine.retaining_ring);

   while (1) {

      /* if (SS_OPEN()) { */
      /*    set_state(&machine, STOP); */
      /* } */

      switch (get_state(&machine)) {

         case IDLE:
            command = parse_command(&steps1, &accel1, &decel1, &speed1,
                                    &steps2, &accel2, &decel2, &speed2,
                                    &abs_threshold, &delta_threshold);
            matlab = false;

            switch(command) {
               case RUN_MATLAB:
                  matlab = true;
                  printf("movie_begin\n");

               case RUN:
                  printf("Beginning system run\n");
                  set_state(&machine, RETAIN);
                  move_made = false;
                  break;

               case HOME:
                  LA_go_to_home(&machine.needle_carriage);
                  LA_go_to_home(&machine.retaining_ring);
                  printf("\n>");
                  break;

               case BRAKE:
                  LA_brake(&machine.needle_carriage);
                  LA_brake(&machine.retaining_ring);
                  printf("\n>");
                  break;

               case STEP2:
               case MOVE2:
                  LA_move(&machine.retaining_ring, steps2, accel2, decel2,
                          speed2);
                  printf("\n>");
                  break;

               case STEP1:
               case MOVE1:
                  LA_move(&machine.needle_carriage, steps1, accel1, decel1,
                          speed1);
                  printf("\n>");
                  break;

               case ACCEL1:
               case DECEL1:
               case SPEED1:
               case ACCEL2:
               case DECEL2:
               case SPEED2:
               case ABS_THRESHOLD:
               case DELTA_THRESHOLD:
                  printf("\n>");
                  break;

               case CAL:
                  PS_calibrate(&(machine.pressure_sensor));
                  PS_print_compensation_buffer(&(machine.pressure_sensor));
                  printf("\n>");
                  break;

               case SCAN:
                  PS_scan_all(&(machine.pressure_sensor));
                  PS_print_scan_buffer(&(machine.pressure_sensor));
                  printf("\n>");
                  break;

               case HELP:
                  show_help_message();
                  show_motor_data(LA_needle_name,
                                  LA_get_position(&(machine.needle_carriage)),
                                  accel1, decel1, speed1, steps1);
                  show_motor_data(LA_ring_name,
                                  LA_get_position(&(machine.retaining_ring)),
                                  accel2, decel2, speed2, steps2);
                  show_threshold_data(abs_threshold, delta_threshold);
                  printf("\n>");
                  break;

               default: // do nothing, also catches none command
                  break;

            }
            break;

         case RETAIN:
            if (!move_made) {
               printf("Retaining peach\n");
               LA_move(&machine.retaining_ring,
                       LA_RING_RETAIN_DEPTH,
                       LA_RING_RETAIN_ACCEL, LA_RING_RETAIN_DECEL,
                       LA_RING_RETAIN_SPEED);
               move_made = true;
            }
            if (parse_command(&steps1, &accel1, &decel1, &speed1, &steps2,
                              &accel2, &decel2, &speed2, &abs_threshold,
                              &delta_threshold)
                == BRAKE) {

               set_state(&machine, STOP);
            }
            if (RETAINED()) {
               LA_brake(&machine.retaining_ring);
               printf("Peach retained\n");
               set_state(&machine, ENGAGE);
               move_made = false;
            }
            if ((LA_get_position(&machine.retaining_ring)
                 > LA_RING_RETAIN_DEPTH-LA_RING_RETAIN_OFFSET)
                && !RETAINED()) {
               printf("No Peach found: resetting\n");
               set_state(&machine, RELEASE);
               move_made = false;
            }
            break;

         case ENGAGE:
            if (!move_made) {
               PS_calibrate(&machine.pressure_sensor);
               printf("Engaging Needles\n");
               LA_move(&machine.needle_carriage,
                       LA_get_position(&machine.retaining_ring) - LA_OFFSET,
                       LA_NEEDLE_ENGAGE_ACCEL, LA_NEEDLE_ENGAGE_DECEL,
                       LA_NEEDLE_ENGAGE_SPEED);
               move_made = true;
            }

            if (parse_command(&steps1, &accel1, &decel1, &speed1, &steps2,
                              &accel2, &decel2, &speed2, &abs_threshold,
                              &delta_threshold)
                == BRAKE) {

               set_state(&machine, STOP);
            }
            if (LA_get_position(&machine.needle_carriage) >=
                LA_get_position(&machine.retaining_ring)
                - LA_OFFSET - LA_NEEDLE_ENGAGE_OFFSET) {

               set_state(&machine, CHECK);
               move_made = false;
            }
            break;

         case CHECK:
            if (!move_made) {
               printf("Checking Peach\n");
               LA_brake(&machine.needle_carriage);
               if (matlab) {
                  LA_move(&machine.needle_carriage,
                          LA_NEEDLE_CHECK_DEPTH, LA_NEEDLE_CHECK_ACCEL_SLOW,
                          LA_NEEDLE_CHECK_DECEL_SLOW,
                          LA_NEEDLE_CHECK_SPEED_SLOW);
               }
               else {
                  LA_move(&machine.needle_carriage,
                          LA_NEEDLE_CHECK_DEPTH, LA_NEEDLE_CHECK_ACCEL,
                          LA_NEEDLE_CHECK_DECEL, LA_NEEDLE_CHECK_SPEED);
               }
               move_made = true;
            }

            if (LA_get_position(&machine.needle_carriage) <
                LA_get_position(&machine.retaining_ring)
                - LA_OFFSET + LA_NEEDLE_CHECK_DEPTH - LA_NEEDLE_CHECK_OFFSET) {

               if (parse_command(&steps1, &accel1, &decel1, &speed1, &steps2,
                                 &accel2, &decel2, &speed2, &abs_threshold,
                                 &delta_threshold)
                   == BRAKE) {

                  set_state(&machine, STOP);
               }
               else {
                  PS_scan_all(&machine.pressure_sensor);

                  if (PS_check(&machine.pressure_sensor, abs_threshold,
                               delta_threshold)) {
                     set_state(&machine, DISENGAGE);
                     printf("\n\n\n\n   !!!!!!PIT!!!!!!\n\n\n\n");
                  }
                  if (matlab) {
                     printf("pressure_sensor_begin\n");
                     PS_print_scan_buffer(&machine.pressure_sensor);
                     printf("pressure_sensor_end\n");
                  }
               }
            }
            else {
               set_state(&machine, DISENGAGE);
               printf("\n\n\n\n             NO PIT\n\n\n\n");
            }
            break;

         case DISENGAGE:
            if (matlab) {
               printf("movie_end\n");
            }
            printf("Disengaging Needles\n");
            LA_go_to_home(&machine.needle_carriage);
            PS_print_scan_buffer(&machine.pressure_sensor);
            set_state(&machine, RELEASE);
            break;

         case RELEASE:
            printf("Releasing Peach\n");
            LA_go_to_home(&machine.retaining_ring);
            set_state(&machine, IDLE);
            break;

         case PASS:
            printf("Passing/Rejecting Peach\n");
            break;

         case STOP:
            /* if (!stop_printed) { */
            /*    printf("Safety Lid Removed: Operation Stopped\n"); */
            /*    stop_printed = true; */
            /* } */
            printf("Machine stopped\n");
            LA_brake(&machine.needle_carriage);
            LA_brake(&machine.retaining_ring);
            set_state(&machine, IDLE);
            /* if (!SS_OPEN()) { */
            /*    printf("Safety Lid Replaced: Resetting Machine\n"); */
            /*    LA_go_to_home(&machine.needle_carriage); */
            /*    LA_go_to_home(&machine.retaining_ring); */
            /*    set_state(&machine, IDLE); */
            /*    stop_printed = false; */
            /* } */
            break;
      }
   }
}

/** \brief Sets the new state of the machine.
 *
 * Convenience method for setting the state of the machine to a new state.
 * Not
 * implemented as a a macro so we ca force the compiler to type check the
 * parameters.
 *
 * \param[in] machine The machine to set the state
 * \param[in] state The new state to set */
INLINE void set_state(PC_t *machine, PC_state_t state) {
   machine->state = state;
}

/** \brief Shows the help message
 *
 *  Outputs help message.
 */
static void show_help_message(void)
{
   printf("%s", help_message);
}

/** \brief Shows the data for the given motor.
 *
 *  Outputs the values of the data you can control by serial interface
 *  and the current position of the stepper linear actuator.
 *
 *  \param[in] name Name of the motor
 *  \param[in] position Current position of the motor
 *  \param[in] acceleration acceleration setting
 *  \param[in] deceleration Deceleration setting
 *  \param[in] speed Speed setting
 *  \param[in] dist distance from last move
 */
static void show_motor_data(const char *name, int16_t position,
                            uint16_t acceleration, uint16_t deceleration,
                            uint16_t speed, int16_t dist) {

   printf("\n"); // do not remove, solves unknown bug
   printf("%s: pos: %d    accel: %u    decel: %u    speed: %u    last_dist: %d",
          name, position, acceleration, deceleration, speed, dist);
}

/** \brief Shows the threshold data.
 *
 * \param[in] abs_threshold the absolute threshold
 * \param[in] delta_threshold the delta threshold */
static void show_threshold_data(uint16_t abs_threshold,
                                uint16_t delta_threshold) {
   printf("\n"); // do not remove, solves unknown bug
   printf("abs: %u    delta: %u\n", abs_threshold, delta_threshold);
}

/** \brief Parses the UART buffer for a new command, and sets the parameter to
 * the given parsed value, if applicable and command is valid.
 *
 * Once the command has been parsed, any remaining data in the buffer is
 * flushed.
 *
 * \param[out] steps1 Needle carriage move distance
 * \param[out] accel1 Needle carriage move accel
 * \param[out] decel1 Needle carriage move decel
 * \param[out] speed1 Needle carriage move speed
 * \param[out] steps2 Retaining Ring move distance
 * \param[out] accel2 Retaining Ring move accel
 * \param[out] decel2 Retaining Ring move decel
 * \param[out] speed2 Retaining Ring move speed
 * \param[out] abs_threshold absolute threshold to use
 * \param[out] delta_threshold delta theshold to use
 *
 * \return The parsed command type, one of the command_t commands */
static command_t parse_command(int16_t *steps1, uint16_t *accel1,
                               uint16_t *decel1, uint16_t *speed1,
                               int16_t *steps2, uint16_t *accel2,
                               uint16_t *decel2, uint16_t *speed2,
                               uint16_t *abs_threshold,
                               uint16_t *delta_threshold) {
   char command_buffer[USART_RX_BUFFER_SIZE];
   command_t command = NONE;
   uint8_t command_ndx = 0;

   // If a command is received, check the command and act on it.
   if (USART_BC_get_string(command_buffer))
   {
      if (command_buffer[command_ndx] == 'r') {
         ++command_ndx;
         if (command_buffer[command_ndx++] == 'u') {
            if (command_buffer[command_ndx++] == 'n') {
               if (command_buffer[command_ndx++] == 'm') {
                  command = RUN_MATLAB;
               }
               else {
                  command = RUN;
               }
            }
         }
      }
      else if (command_buffer[command_ndx] == 'a') {
         ++command_ndx;
         if (command_buffer[command_ndx++] == 't') {
            if (command_buffer[command_ndx++] == ' '){
               // ...new threshold given
               *abs_threshold = atoi((const char *)command_buffer+command_ndx);
               command = ABS_THRESHOLD;
            }
         }
      }
      else if (command_buffer[command_ndx] == 'd') {
         ++command_ndx;
         if (command_buffer[command_ndx++] == 't') {
            if (command_buffer[command_ndx++] == ' '){
               // ...new threshold given
               *delta_threshold = atoi((const char *)
                                       command_buffer+command_ndx);
               command = DELTA_THRESHOLD;
            }
         }
      }
      else if (command_buffer[command_ndx] == 'h') {
         command = HOME;
      }
      else if (command_buffer[command_ndx] == 'm') {
         ++command_ndx;
         // Move with...
         if (command_buffer[command_ndx] == '1') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' '){
               // ...number of steps given.
               *steps1 = atoi((const char *)command_buffer+command_ndx);
               command = STEP1;
            }
         }
         else if (command_buffer[command_ndx] == '2') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' '){
               // ...number of steps given.
               *steps2 = atoi((const char *)command_buffer+command_ndx);
               command = STEP2;
            }
         }
         else if (command_buffer[command_ndx++] == 'o'){
            if (command_buffer[command_ndx++] == 'v'){
               if (command_buffer[command_ndx++] == 'e'){
                  // ...all parameters given
                  if (command_buffer[command_ndx] == '1') {
                     ++command_ndx;
                     if (command_buffer[command_ndx++] == ' ') {
                        *steps1 =
                           atoi((const char *)command_buffer+command_ndx);

                        command_ndx =
                           find_next_param(command_buffer, ++command_ndx);
                        *accel1 =
                           atoi((const char *)command_buffer+command_ndx);

                        command_ndx =
                           find_next_param(command_buffer, ++command_ndx);
                        *decel1 =
                           atoi((const char *)command_buffer+command_ndx);

                        command_ndx =
                           find_next_param(command_buffer, ++command_ndx);
                        *speed1 =
                           atoi((const char *)command_buffer+command_ndx);

                        command = MOVE1;
                     }
                  }
                  else if (command_buffer[command_ndx] == '2') {
                     ++command_ndx;
                     if (command_buffer[command_ndx++] == ' ') {
                        *steps2 =
                           atoi((const char *)command_buffer+command_ndx);

                        command_ndx =
                           find_next_param(command_buffer, ++command_ndx);
                        *accel2 =
                           atoi((const char *)command_buffer+command_ndx);

                        command_ndx =
                           find_next_param(command_buffer, ++command_ndx);
                        *decel2 =
                           atoi((const char *)command_buffer+command_ndx);

                        command_ndx =
                           find_next_param(command_buffer, ++command_ndx);
                        *speed2 =
                           atoi((const char *)command_buffer+command_ndx);

                        command = MOVE2;
                     }
                  }
               }
            }
         }
      }
      else if (command_buffer[command_ndx] == 'a'){
         ++command_ndx;
         // Set acceleration.
         if (command_buffer[command_ndx] == '1') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' ') {
               *accel1 = atoi((const char *)command_buffer+command_ndx);
               command = ACCEL1;
            }
         }
         else if (command_buffer[command_ndx] == '2') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' '){
               *accel2 = atoi((const char *)command_buffer+command_ndx);
               command = ACCEL2;
            }
         }
      }
      else if (command_buffer[command_ndx] == 'd'){
         ++command_ndx;
         // Set deceleration.
         if (command_buffer[command_ndx] == '1') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' '){
               *decel1 = atoi((const char *)command_buffer+command_ndx);
               command = DECEL1;
            }
         }
         else if (command_buffer[command_ndx] == '2') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' '){
               *decel2 = atoi((const char *)command_buffer+command_ndx);
               command = DECEL2;
            }
         }
      }
      else if (command_buffer[command_ndx] == 's') {
         ++command_ndx;
         if (command_buffer[command_ndx] == '1') {
            ++command_ndx;
            if (command_buffer[command_ndx++] == ' ') {
               *speed1 = atoi((const char *)command_buffer+command_ndx);
               command = ACCEL1;
            }
         }
         if (command_buffer[command_ndx++] == '2') {
            if (command_buffer[command_ndx] == ' ') {
               ++command_ndx;
               *speed2 = atoi((const char *)command_buffer+command_ndx);
               command = ACCEL2;
            }
         }
      }
      else if (command_buffer[command_ndx] == 'c') {
         command = CAL;
      }
      else if (command_buffer[command_ndx] == 'p') {
         command = SCAN;
      }
      else if (command_buffer[command_ndx] == '?') {
         command = HELP;
      }
      else {
         command = BRAKE;
      }

      // Clear RXbuffer.
      USART_BC_flush_RX_buffer();
   }//end if (cmd)

   return command;
}

/** \brief Finds the next param in the given command buffer string.
 *
 * \param[in] command_buffer the string to search through for the next param
 * \param[in] start_position the index value to start at in the string
 *
 * \return index of next parameter in the string */
static uint8_t find_next_param(char *command_buffer, uint8_t start_position)
{
   uint8_t ndx = start_position;

   while ((command_buffer[ndx]!=' ') &&
          (command_buffer[ndx]!='\n')) {
      ++ndx;
   }
   return ++ndx;
}
