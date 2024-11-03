/**************************************************************
 * main.c
 * rev 1.0 03-Nov-2024 kaeyo
 * practice_exam
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

//  Q2  - Function to read ldr as voltage
float ldr_read_voltage()  {
  const float conversion_factor = 3.3f / (1 << 12); //calculate conversion factor
  uint16_t result = adc_read(); //get reading
  float voltage = result * conversion_factor; //convert to voltage
  return voltage;
}



int main(void) {
  //  Q1a - initialize io
  stdio_init_all();

  //  Q1b - print title and name
  printf("CC2511 Exam 2022\r\n");
  printf("Kae Young\r\n");

  //  Q2a - configure adc pin
  adc_init();
  adc_gpio_init(LDR_PIN);
  adc_select_input(0);

  //  Q2b
  int ldr_readings_count = 0; //initialize ldr count
  float past_ldr_readings[5]; //initialize ldr readings array
  gpio_init(RED_LED); //initialize red led
  gpio_set_dir(RED_LED, GPIO_OUT);  //set red led gpio to out
  float average_ldr = 0;
  
  while (true) {
    //  Q2b
    float voltage = ldr_read_voltage(); //get reading
    past_ldr_readings[ldr_readings_count % 5] = voltage;  //store voltage
    int N = 5;  //define number of elements
    if (ldr_readings_count < 5)
    {
      N = ldr_readings_count; //if less than 5 readings have been made set N to number of readings that have been made
    }
    float past_ldr_readings_sum = 0;  //initialize ldr sum variable
    for (int i = 0; i < N; i++)
    {
      past_ldr_readings_sum = past_ldr_readings_sum + past_ldr_readings[i]; //loop through each element of past ldr readings, adding each to the sum
    }
    float new_average_ldr = past_ldr_readings_sum/(float)N; //calculate average ldr reading
    if (new_average_ldr < average_ldr)
    {
      gpio_put(RED_LED, true);  //if new average is less than old average turn red light on
    }
    else
    {
      gpio_put(RED_LED, false); //else turn red light off
    }
    average_ldr = new_average_ldr;  //set average to new average
    printf("Voltage = %f, Average = %f, Index = %i\r\n", voltage, average_ldr, ldr_readings_count%5); //print output
    term_move_to(1,3);  //move cursor back to start
    ldr_readings_count++; //increment ldr count
    sleep_ms(500);  //wait 500ms
  }
}
