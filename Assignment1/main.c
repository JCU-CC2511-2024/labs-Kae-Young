/**************************************************************
 * main.c
 * rev 1.0 22-Aug-2024 Kae Young
 * Assignment1
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>

// Declare button pins
#define PUSH1_BUT 2
#define PUSH2_BUT 3
#define PUSH3_BUT 4

// Declare LED pins
#define RED_LED   11
#define GREEN_LED 12
#define BLUE_LED  13

int main(void) {
  // Initialize all pins
  gpio_init(PUSH1_BUT);
  gpio_init(PUSH2_BUT);
  gpio_init(PUSH3_BUT);
  gpio_init(RED_LED);
  gpio_init(GREEN_LED);
  gpio_init(BLUE_LED);

  // Set direction of all pins
  gpio_set_dir(RED_LED, GPIO_OUT);
  gpio_set_dir(GREEN_LED, GPIO_OUT);
  gpio_set_dir(BLUE_LED, GPIO_OUT);
  gpio_set_dir(PUSH1_BUT, GPIO_IN);
  gpio_set_dir(PUSH2_BUT, GPIO_IN);
  gpio_set_dir(PUSH3_BUT, GPIO_IN);

  // Define buttons as pull ups
  gpio_pull_up(PUSH1_BUT);
  gpio_pull_up(PUSH2_BUT);
  gpio_pull_up(PUSH3_BUT);

  // Declare LED state
  bool is_red_on = false;
  bool is_green_on = false;
  bool is_blue_on = false;

  while (true) {
    // Red LED logic
    if (gpio_get(PUSH1_BUT) == 0 & is_red_on == false)  {
      gpio_put(RED_LED, 1); // Red on
      is_red_on = true; // Save state
    }
    else if (gpio_get(PUSH1_BUT) == 1 & is_red_on == true)  {
      gpio_put(RED_LED, 0); // Red off
      is_red_on = false;  // Save state
    }

    // Green LED logic
    if (gpio_get(PUSH2_BUT) == 0 & is_green_on == false)  {
      gpio_put(GREEN_LED, 1); // Green on
      is_green_on = true; // Save state
    }
    else if (gpio_get(PUSH2_BUT) == 1 & is_green_on == true)  {
      gpio_put(GREEN_LED, 0); // Green off
      is_green_on = false;  // Save state
    }

    // Blue LED logic
    if (gpio_get(PUSH3_BUT) == 0 & is_blue_on == false)  {
      gpio_put(BLUE_LED, 1);  // Blue on
      is_blue_on = true;  // Save state
    }
    else if (gpio_get(PUSH3_BUT) == 1 & is_blue_on == true)  {
      gpio_put(BLUE_LED, 0);  // Blue off
      is_blue_on = false; // Save state
    }



    // A simpler method but has the downside of writing to the gpio pins every cycle
    /*
    gpio_put(RED_LED, !gpio_get(PUSH1_BUT))
    gpio_put(GREEN_LED, !gpio_get(PUSH2_BUT))
    gpio_put(BLUE_LED, !gpio_get(PUSH3_BUT))
    */


    // Wait
    sleep_us(250);
  }
}
