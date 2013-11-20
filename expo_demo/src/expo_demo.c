/** \file expo_demo.c
 *
 * \brief System software for demoing at the senior project expo.
 *
 * This test follows directly from AVR446: Linear Speed Control of Stepper
 * LA_needles. This test is simply a port of the code provided with this App
 * Note to
 * the Xmega Platform. On startup, a help menu is presented which explains how
 * to run the tests.
 *
 * \todo TODO test_limit_switches.c documentation
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

#include "expo_demo.h"

/* Global Data ****************************************************************/


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
   SWITCH_PORTH.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_INVEN_bm;
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
   SS_PORT.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
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

/** \brief Initializes the stepper motors.
 *
 * Calls into the stepper_motor.c library to initialize two to linear
 * actuators.  The pin values and timers used for each motor are defined in
 * this file's header file.
 *
 * \param[in] needle_motor The needle motor to initialize
 * \param[in] ring_motor The ring motor to initialize */
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
   uint8_t switch_mask = 0x00;

   int16_t steps1 = 1000;
   int16_t steps2 = 1000;
   uint16_t accel1 = 100;
   uint16_t accel2 = 100;
   uint16_t decel1 = 100;
   uint16_t decel2 = 100;
   uint16_t speed1 = 800;
   uint16_t speed2 = 800;

   uint16_t delay_ndx = 0;

   LA_t LA_needle;
   LA_t LA_ring;

   command_t command;
   bool info_shown = false;
   uint16_t steps_left = 0;

   /* call all of the setup_* functions */
   cli();
   setup_clocks();
   setup_LEDs();
   setup_switches(switch_mask);
   setup_safety_switch();
   setup_pressure_sensor(&machine.pressure_sensor);
   setup_USART_BC();
   setup_linear_actuators(&machine.needle_carriage,
                          &machine.retaining_ring);
   set_state(&machine, IDLE);
   sei();

   /* shows the help menu */
   show_help_message();

   /* show the current state of the linear actuator */
   show_motor_data(LA_get_position(&machine.needle_carriage),
                   accel1, decel1, speed1, steps1);
   show_motor_data(LA_get_position(&machine.retaining_ring),
                   accel2, decel2, speed2, steps2);

   while (1) {

      switch(get_state(&machine)) {

         case IDLE:
            if (!info_shown){
               show_motor_data(LA_get_position(&machine.needle_carriage),
                               accel1, decel1, speed1, steps1);
               show_motor_data(LA_get_position(&machine.retaining_ring),
                               accel2, decel2, speed2, steps2);
               printf("Ready for command\n");
               info_shown = true;
            }
            command = parse_command(&steps1, &accel1, &decel1, &speed1,
                                    &steps2, &accel2, &decel2, &speed2);

            switch(command) {
               case STEP2:
               case MOVE2:
                  LA_move(&machine.retaining_ring, steps2, accel2, decel2,
                          speed2);
                  info_shown = false;
                  break;

               case STEP1:
               case MOVE1:
                  LA_move(&machine.needle_carriage, steps1, accel1, decel1,
                          speed1);
                  info_shown = false;
                  break;

               case ACCEL1:
               case DECEL1:
               case SPEED1:
               case ACCEL2:
               case DECEL2:
               case SPEED2:
                  info_shown = false;
                  break;

               case CAL:
                  PS_calibrate(&machine.pressure_sensor);
                  PS_print_compensation_buffer(&machine.pressure_sensor);
                  break;

               case SCAN:
                  PS_scan_all(&machine.pressure_sensor);
                  PS_print_scan_buffer(&machine.pressure_sensor);
                  break;

               case HELP:
                  show_help_message();
                  info_shown = false;
                  break;

               default: // do nothing, also catches none command
                  break;

            }
            break;

            /*    case RETAIN: */

            /*          break; */

            /*    case ENGAGE: */

            /*       break; */

            /*    case DETECT: */

            /*       break; */

            /*    case DISENGAGE: */

            /*       break; */

            /*    case RELEASE: */

            /*       break; */

            /*    case PASS: */

            /*       break; */

            /*    case STOP: */

            /*       break; */

      }




      /*  */
      /*       switch(command = parse_command(&steps, &accel, &decel,
       *       &speed))
       *       { */
      /*  */
      /*          case STEP: */
      /*             LA_move(&LA_needle, steps, accel, decel, speed); */
      /*             LA_move(&LA_ring, steps, accel, decel, speed); */
      /*             position += steps; */
      /*             printf("\n\n"); */
      /*             break; */
      /*  */
      /*          case MOVE: */
      /*             LA_move(&LA_needle, steps, accel, decel, speed); */
      /*             LA_move(&LA_ring, steps, accel, decel, speed); */
      /*             position += steps; */
      /*             printf("\n\n"); */
      /*             break; */
      /*  */
      /*          case ACCEL: */
      /*          case DECEL: */
      /*          case SPEED: */
      /*             printf("\n\n"); */
      /*             break; */
      /*  */
      /*          case REPEAT: */
      /*             LA_move(&LA_needle, steps, accel, decel, speed); */
      /*             LA_move(&LA_ring, steps, accel, decel, speed); */
      /*             position += steps; */
      /*             printf("\n\n"); */
      /*             break; */
      /*  */
      /*          case HELP: */
      /*             show_help_message(); */
      /*             break; */
      /*  */
      /*          case NONE: */
      /*             break; */
      /*  */
      /*          default: */
      /*             show_help_message(); */
      /*             break; */
      /*       } */
      /*  */
      /*       if ((command != HELP) && (command != NONE)) { */
      /*          while (LA_get_motor_state(&LA_needle) != SM_STOP) { */
      /*             if (READ_SWITCHES & PIN6_bm) { */
      /*                LA_brake(&LA_needle); */
      /*                LA_brake(&LA_ring); */
      /*                printf("motors parked\n"); */
      /*             } */
      /*             if (steps > 0) { */
      /*                steps_left = (int32_t)steps * SPR
       *                / LA_needle.pitch */
      /*                   - LA_needle.motor.speed_ramp.step_count; */
      /*             } */
      /*             else { */
      /*                steps_left = -1 * (int32_t)steps * SPR
       *                / LA_needle.pitch
       *                */
      /*                   - LA_needle.motor.speed_ramp.step_count; */
      /*             } */
      /*             printf("    Running... Steps Left: %d\n",
       *             steps_left); */
      /*             delay_ms(250); */
      /*          } */
      /*  */
      /*          printf("    Done with command\n"); */
      /*  */
      /*          show_motor_data(position, accel, decel, speed, steps);
       *          */
      /*       } */
   }//end while (1)
}

/** \brief Sets the new state of the machine.
 *
 * Convenience method for setting the state of the machine to a new state. Not
 * implemented as a a macro so we ca force the compiler to type check the
 * parameters.
 *
 * \param[in] machine The machine to set the state
 * \param[in] state The new state to set */
INLINE void set_state(PC_t *machine, PC_STATE_t state) {
   machine->state = state;
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
 *  and the current position of the stepper LA_needle.
 *
 *  \param acceleration Accelration setting.
 *  \param deceleration Deceleration setting.
 *  \param speed Speed setting.
 *  \param steps Position of the stepper LA_needle.
 */
static void show_motor_data(int16_t position, uint16_t acceleration,
                            uint16_t deceleration, uint16_t speed,
                            int16_t steps) {
   printf("\n\r LA_needle pos: %d    a: %u    d: %u    s: %u    m: %d\n\r>",
          position, acceleration, deceleration, speed, steps);
}


static command_t parse_command(int16_t *steps1, uint16_t *accel1,
                               uint16_t *decel1, uint16_t *speed1,
                               int16_t *steps2, uint16_t *accel2,
                               uint16_t *decel2, uint16_t *speed2) {
   char command_buffer[USART_RX_BUFFER_SIZE];
   command_t command = NONE;
   uint8_t command_ndx = 0;

   // If a command is received, check the command and act on it.
   if (USART_BC_get_string(command_buffer))
   {
      if (command_buffer[command_ndx] == 'm') {
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
      else if (command_buffer[command_ndx] == 'c'){// hyper terminal sends \r\n
         command = CAL;
      }
      else if (command_buffer[command_ndx] == 'p'){// hyper terminal sends \r\n
         command = SCAN;
      }
      else if (command_buffer[command_ndx] == '?'){
         command = HELP;
      }
      else {
         command = HELP; // invalid command
      }

      // Clear RXbuffer.
      USART_BC_flush_RX_buffer();
   }//end if (cmd)

   return command;
}

static uint8_t find_next_param(char *command_buffer, uint8_t start_position) {
   uint8_t ndx = start_position;

   while ((command_buffer[ndx]!=' ') &&
          (command_buffer[ndx]!='\n')) {
      ++ndx;
   }
   return ++ndx;
}
