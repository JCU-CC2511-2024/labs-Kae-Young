/**************************************************************
 * main.c
 * rev 1.0 03-Nov-2024 kaeyo
 * Young_Kae_Exam
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include <string.h>
#include "terminal.h"
#include <math.h>

//  define pins
#define LDR_PIN   26
#define RED_LED   11
#define GREEN_LED 12
#define BLUE_LED  13
#define PUSH_1    2
#define PUSH_2    3
#define PUSH_3    4


int main(void) {
  // TODO - Initialise components and variables
  stdio_init_all();
  while (true) {
    printf("Hello");
    // TODO - Repeated code here
  }
}
