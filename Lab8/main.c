/**************************************************************
 * main.c
 * rev 1.0 18-Sep-2024 Kae Young
 * Lab8
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "terminal.h"

#define LDR_PIN                 26
#define GREEN_LED               13

float voltage = 0;
int pwm_max = 255;
int pwm = 0;

int main(void) {
  // TODO - Initialise components and variables

  stdio_init_all();
  adc_init();

   // Initialise Pins
  gpio_init(GREEN_LED);
  adc_gpio_init(LDR_PIN);

  // Set pin directions
  gpio_set_dir(GREEN_LED, true);

  // Set gpio function 
  gpio_set_function(GREEN_LED, GPIO_FUNC_PWM);

  // Slice number
  uint greenslice_num = pwm_gpio_to_slice_num(GREEN_LED);

  // Enable
  pwm_set_enabled(greenslice_num, true);

  // 
  adc_select_input(0);


  while (true) {
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    voltage = result * conversion_factor;
    pwm = (result * conversion_factor * pwm_max) / 3.3f;
    pwm_set_gpio_level(GREEN_LED, pwm*pwm);
    printf("Raw value: 0x%03x Voltage: %f PWM: %i\r\n", result, voltage, pwm);
    sleep_ms(200);
  }
}