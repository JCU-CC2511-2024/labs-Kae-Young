/**************************************************************
 * main.c
 * rev 1.0 19-Sep-2024 kaeyo
 * Lab9
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>
#include "hardware/pwm.h"

#define BLUE_LED    13
#define RED_LED     12
#define GREEN_LED   11
#define SEND_BTN    2
#define MODE_BTN    3

#define t_interval_idle 50
#define t_interval_1    60
#define t_interval_0    40

//Operation Modes
#define TRANSMITTER_MODE  true
#define RECEIVER_MODE     false

//send a 1
void signal_1(void)  {
  gpio_put(BLUE_LED, true);
  gpio_put(GREEN_LED, true);
  gpio_put(RED_LED, true);

  sleep_ms(t_interval_1);

  gpio_put(BLUE_LED, false);
  gpio_put(GREEN_LED, false);
  gpio_put(RED_LED, false);

  sleep_ms(t_interval_1);
}

//send a 0
void signal_0(void) {
  gpio_put(BLUE_LED, true);
  gpio_put(GREEN_LED, true);
  gpio_put(RED_LED, true);

  sleep_ms(t_interval_0);

  gpio_put(BLUE_LED, false);
  gpio_put(GREEN_LED, false);
  gpio_put(RED_LED, false);

  sleep_ms(t_interval_0);
}

//send an idle
void signal_idle(void) {
  gpio_put(BLUE_LED, true);
  gpio_put(GREEN_LED, true);
  gpio_put(RED_LED, true);

  sleep_ms(t_interval_idle);

  gpio_put(BLUE_LED, false);
  gpio_put(GREEN_LED, false);
  gpio_put(RED_LED, false);

  sleep_ms(t_interval_idle);
}

void send_signal(bool *signal, int n)  {
  for (int i = 0; i < n; i++)
  {
    if (signal[i])
    {
      signal_1();
    }
    else
    {
      signal_0();
    }
  }

}

int main(void) {
  //operation mode
  bool operation_mode = RECEIVER_MODE; //true for transmitter //false for receiver

  //init RGB LED's
  gpio_init(BLUE_LED);
  gpio_set_dir(BLUE_LED, GPIO_OUT);
  gpio_init(RED_LED);
  gpio_set_dir(RED_LED, GPIO_OUT);
  gpio_init(GREEN_LED);
  gpio_set_dir(GREEN_LED, GPIO_OUT);

  //init buttons
  gpio_init(SEND_BTN);
  gpio_set_dir(SEND_BTN, GPIO_IN);
  gpio_init(MODE_BTN);
  gpio_set_dir(MODE_BTN, GPIO_IN);

  int n = 10;
  bool signal[] = {1,0,1,0,1,1,0,0,1,0};
  bool send_btn_pressed = false;

  while (true) {
    if(!gpio_get(MODE_BTN))
    {
      operation_mode = !operation_mode;
      if(operation_mode == RECEIVER_MODE)
      {
        gpio_put(RED_LED, true);
        sleep_ms(1000);
        gpio_put(RED_LED, false);
      }
      if(operation_mode == TRANSMITTER_MODE)
      {
        gpio_put(BLUE_LED, true);
        sleep_ms(1000);
        gpio_put(BLUE_LED, false);
      }
      sleep_ms(500);
    }
    if (operation_mode == TRANSMITTER_MODE)
    {
      signal_idle();
      if (!gpio_get(SEND_BTN) && !send_btn_pressed) 
      {
        send_btn_pressed = true;
        send_signal(signal, n);
        send_btn_pressed = false;
      }
    }
    else if (operation_mode == RECEIVER_MODE)
    {

    }
  }
}