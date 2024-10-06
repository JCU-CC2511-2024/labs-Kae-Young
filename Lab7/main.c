/**************************************************************
 * main.c
 * rev 1.0 18-Sep-2024 Kae Young
 * Lab7
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "terminal.h"

// Define GPIO pins
#define RED_LED                 11
#define BLUE_LED                12
#define GREEN_LED               13
#define TX_PIN                   0
#define RX_PIN                   1

// Define data types
static int chars_rxed = 0;
volatile char buffer [100];
volatile unsigned int buffer_index = 0;
volatile bool has_command = false;
int red_pwm, green_pwm, blue_pwm = 0;
unsigned short background;
unsigned short foreground;

/*
########################################################

This function is an RX interupt handler and detects when 
an event has come through. Had various case statment that 
analyse if the enter or backspace button was pressed and 
adjusts buffer and buffer index accordinly. It thens 
calls sench_ch function to send specific character at 
each instance. 

########################################################
*/

void on_uart_rx() {
    while (uart_is_readable(uart0)) {
      uint8_t ch = uart_getc(uart0);
        switch (ch) {

          // Detect eneter button pressed
          case '\r':
          case '\n':
            buffer[buffer_index] = 0; // Add trailing null
            has_command = true; // Enter key has been pressed
          break;

          // detect backspace button pressed
          case 0x7f:
            if (buffer_index > 0) { // check if buffer is empty
              buffer[buffer_index] = '\0';
              buffer_index--;
            }
            send_ch(0x7f);
          break;

          // default send character
          default:
            if (buffer_index < 99) { // check if buffer is full
              buffer[buffer_index] = ch;
              buffer_index++;
            }
            send_ch(ch);
          break;
        }
        chars_rxed++;
    }
}

// Function to write character
void send_ch(char ch){
   if (uart_is_writable(uart0)) {
      uart_putc(uart0, ch);
    }
}

/*
########################################################

The following various functions creat the terminal user 
interface that is displayed on putty.

########################################################
*/

void draw_heading() {
  term_move_to(15,1);
  term_set_color(clrGreen, clrBlack);
  printf("+--------------CC2511 Lab7--------------+\r\n");
}

void draw_box() {
  term_move_to(1,13);
  term_set_color(clrBlack, clrGreen);
  printf("+-------------------------------------------------------------------+");
}

void draw_pwm_heading() {
  term_move_to(1,3);
  term_set_color(clrBlack, clrGreen);
  printf("+---[PWM STATUS]---+\r\n");
}

void draw_menu_heading() {
  term_move_to(22,3);
  term_set_color(clrBlack, clrGreen);
  printf("+--------------------[MENU]--------------------+\r\n");
}

void draw_menu() {
  term_move_to(25,5);
  term_set_color(clrGreen, clrBlack);
  printf("Type the following commands:");
  term_move_to(25,6);
  printf("> red n       Set the red PWM ration to n");
  term_move_to(25,7);
  printf("> green n     Set the green PWM ration to n");
  term_move_to(25,8);
  printf("> blue n      Set the blue PWM ration to n");
  term_move_to(25,9);
  printf("> stop         Turn all LEDs off");
  term_move_to(25, 11);
  printf("Note: max PWM ratio is 255");
}

void draw_pwm_status(int red_pwm, int green_pwm, int blue_pwm) {
  term_set_color(clrGreen, clrBlack);
  term_move_to(5,6);
  printf("Red:   %i", red_pwm);
  term_move_to(5,7);
  printf("Green: %i", green_pwm);
  term_move_to(5,8);
  printf("Blue:  %i", blue_pwm);
}

void print_command() {
  term_move_to(1,15);
  term_set_color(clrWhite, clrBlack);
  printf("Command prompt: \r\n");
  printf("> ");
}


int main(void) {

  // Initialise library
  stdio_init_all();

  // Initialise UART 0
  uart_init(uart0, 115200);

  // Set to GPIO pin mux to the UART - 0 is TX, 1 is RX
  gpio_set_function(TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(RX_PIN, GPIO_FUNC_UART);

  // Select correct interrupt for the UART 
  int UART_IRQ = uart0 == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(uart0, true, false);

  // Turn off FIFO's
  uart_set_fifo_enabled(uart0, false);

  // Declare new integer values
  uint red_value, green_value, blue_value = 0;

  // Initialise Pins
  gpio_init(RED_LED);
  gpio_init(GREEN_LED);
  gpio_init(BLUE_LED);

  // Set pin directions
  gpio_set_dir(RED_LED, true);
  gpio_set_dir(GREEN_LED, true);
  gpio_set_dir(BLUE_LED, true);

  // Set gpio function 
  gpio_set_function(RED_LED, GPIO_FUNC_PWM);
  gpio_set_function(GREEN_LED, GPIO_FUNC_PWM);
  gpio_set_function(BLUE_LED, GPIO_FUNC_PWM);

  // Slice number
  uint redslice_num = pwm_gpio_to_slice_num(RED_LED);
  uint greenslice_num = pwm_gpio_to_slice_num(GREEN_LED);
  uint blueslice_num = pwm_gpio_to_slice_num(BLUE_LED);

  // Enable
  pwm_set_enabled(redslice_num, true);
  pwm_set_enabled(greenslice_num, true);
  pwm_set_enabled(blueslice_num, true);

  
  while (true) {

    // Call writing functions
    term_cls();
    draw_heading();
    draw_box();
    draw_pwm_heading();
    draw_menu_heading();
    draw_menu();
    draw_pwm_status(red_pwm, green_pwm, blue_pwm);
    print_command();

    while (!has_command) { // does nothing while no command
      __asm ("wfi");
    }
      
      /*
        ########################################################
        
        This function detects when a command has come through.
        It scans what is in the buffer and compares it to the 
        specific commands availiable and sets integer to specific
        PWM interger defined. 
        
        ########################################################
      */

      if (has_command == true) {

          if (sscanf(buffer, "red %i", &red_pwm) == 1){
              printf("Red PWM set to %i\r\n", red_pwm);
              pwm_set_gpio_level(RED_LED, red_pwm*red_pwm);
          }

          if (sscanf(buffer, "green %i", &green_pwm) == 1){
              printf("Green PWM set to %i\r\n", green_pwm);
              pwm_set_gpio_level(GREEN_LED, green_pwm*green_pwm);
          }

          if (sscanf(buffer, "blue %i", &blue_pwm) == 1){
              printf("Blue PWM set to %i\r\n", blue_pwm);
              pwm_set_gpio_level(BLUE_LED, blue_pwm*blue_pwm);
          }

          if (0 == strcmp(buffer, "stop")) {
              red_pwm = 0;
              green_pwm = 0;
              blue_pwm = 0;
              pwm_set_gpio_level(RED_LED, red_pwm);
              pwm_set_gpio_level(GREEN_LED, green_pwm);
              pwm_set_gpio_level(BLUE_LED, blue_pwm);
          }
      }
    has_command = false; // Set command back to false
    buffer_index = 0; // Set buffer index back to 0
  }
}