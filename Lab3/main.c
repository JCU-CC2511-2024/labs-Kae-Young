/**************************************************************
 * main.c
 * rev 1.0 08-Aug-2024 Kae Young
 * Lab3
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>


int main(void) {
  stdio_usb_init();

  #define RED_LED 11
  #define GREEN_LED 12
  #define BLUE_LED 13
  
  gpio_init(RED_LED);
  gpio_init(GREEN_LED);
  gpio_init(BLUE_LED);

  gpio_set_dir(RED_LED, GPIO_OUT);
  gpio_set_dir(GREEN_LED, GPIO_OUT);
  gpio_set_dir(BLUE_LED, GPIO_OUT);

  bool IS_RED_ON = false;
  bool IS_GREEN_ON = false;
  bool IS_BLUE_ON = false;

  while (true) {
    int ch = getchar_timeout_us(0);
    if(ch != PICO_ERROR_TIMEOUT)  {
      switch (ch) {
      case 'r':
        gpio_put(RED_LED, !IS_RED_ON);
        IS_RED_ON = !IS_RED_ON;
        printf("TOGGLE RED\r\n");
        break;
      case 'g':
        gpio_put(GREEN_LED, !IS_GREEN_ON);
        IS_GREEN_ON = !IS_GREEN_ON;
        printf("TOGGLE GREEN\r\n");
        break;
      case 'b':
        gpio_put(BLUE_LED, !IS_BLUE_ON);
        IS_BLUE_ON = !IS_BLUE_ON;
        printf("TOGGLE BLUE\r\n");
        break;
      default:
        printf("UNRECOGNIZED INPUT\r\n");
        break;
      }
    }
    sleep_ms(20);
  }
}
