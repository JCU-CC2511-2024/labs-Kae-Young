/**************************************************************
 * main.c
 * rev 1.0 22-Aug-2024 kaeyo
 * Lab5
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include "hardware/pwm.h"
#include <stdio.h>

#define CONTENT_OF(addr)        (*(volatile uint32_t*)addr)
#define PAD_CONTROL_BASE        0x4001c000
#define REG_PAD_CONTROL_GPIO15  (PAD_CONTROL_BASE+0x40)
#define CTRL_PDE                0x4
#define CTRL_PUE                0x8

#define BLUE_LED                13
#define GREEN_LED               12
#define RED_LED                 11

#define PWR_INCREMENT           10
#define PWR_LIMIT               255

int main(void) {
  // Initialize serial usb io
  stdio_usb_init();

  // Clear pull up and pull down registers for GPIO15
  CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= ~CTRL_PUE;
  CONTENT_OF(REG_PAD_CONTROL_GPIO15) |= ~CTRL_PDE;
  // CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= ~0xc

  // Configure blue LED
  gpio_init(BLUE_LED);                                    //init LED GPIO
  gpio_set_dir(BLUE_LED, GPIO_OUT);                       //set LED GPIO to output
  gpio_set_function(BLUE_LED, GPIO_FUNC_PWM);             //set LED GPIO to PWM function
  uint blue_slice_num = pwm_gpio_to_slice_num(BLUE_LED);  //get slice number
  pwm_set_enabled(blue_slice_num, true);                  //start generating PWM signal
  uint16_t blue_pwr = 0;                                       //init LED power value

  // Configure red LED
  gpio_init(RED_LED);                                     //refer to blue ^^^^^^
  gpio_set_dir(RED_LED, GPIO_OUT);
  gpio_set_function(RED_LED, GPIO_FUNC_PWM);
  uint red_slice_num = pwm_gpio_to_slice_num(RED_LED);
  pwm_set_enabled(red_slice_num, true);
  uint16_t red_pwr = 0;

  // Configure green LED
  gpio_init(GREEN_LED);                                   //refer to blue ^^^^^^
  gpio_set_dir(GREEN_LED, GPIO_OUT);
  gpio_set_function(GREEN_LED, GPIO_FUNC_PWM);
  uint green_slice_num = pwm_gpio_to_slice_num(GREEN_LED);
  pwm_set_enabled(green_slice_num, true);
  uint16_t green_pwr = 0;

  // Configure floating pin 15
  gpio_init(15);
  gpio_set_dir(15, GPIO_IN);
  

  while (true) {
    gpio_put(BLUE_LED, gpio_get(15)); // Set blue LED state to GPIO15 state

    int ch = getchar_timeout_us(0);
    char str[3];
    if(ch != PICO_ERROR_TIMEOUT)  {
      switch(ch)  {
        case 'u':
          printf("Pin 15 to pull up\r\n");
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= ~CTRL_PUE;
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) |= ~CTRL_PDE;
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= CTRL_PUE;
          break;
        
        case 'd':
          printf("Pin 15 to pull down\r\n");
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= ~CTRL_PUE;
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) |= ~CTRL_PDE;
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= CTRL_PDE;
          break;
        
        case 'o':
          printf("Pin 15 to pull none\r\n");
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) &= ~CTRL_PUE;
          CONTENT_OF(REG_PAD_CONTROL_GPIO15) |= ~CTRL_PDE;
          break;
        
        case 'r':
          if((red_pwr-PWR_INCREMENT)>=0)  {
            red_pwr = red_pwr - PWR_INCREMENT;
            pwm_set_gpio_level(RED_LED, red_pwr*red_pwr);
          }
          else  {
            red_pwr = 0;
          }
          intToStr(red_pwr,str);
          printf("Red power: ");
          printf(str);
          printf("\r\n");
          break;

        case 'R':
          if((red_pwr+PWR_INCREMENT)<=PWR_LIMIT)  {
            red_pwr = red_pwr + PWR_INCREMENT;
            pwm_set_gpio_level(RED_LED, red_pwr*red_pwr);
          }
          else  {
            red_pwr = PWR_LIMIT;
          }
          intToStr(red_pwr,str);
          printf("Red power: ");
          printf(str);
          printf("\r\n");
          break;

        case 'b':
          if((blue_pwr-PWR_INCREMENT)>=0)  {
            blue_pwr = blue_pwr - PWR_INCREMENT;
            pwm_set_gpio_level(BLUE_LED, blue_pwr*blue_pwr);
          }
          else  {
            blue_pwr = 0;
          }
          intToStr(blue_pwr,str);
          printf("Blue power: ");
          printf(str);
          printf("\r\n");
          break;
        
        case 'B':
          if((blue_pwr+PWR_INCREMENT)<=PWR_LIMIT)  {
            blue_pwr = blue_pwr + PWR_INCREMENT;
            pwm_set_gpio_level(BLUE_LED, blue_pwr*blue_pwr);
          }
          else  {
            blue_pwr = PWR_LIMIT;
          }
          intToStr(blue_pwr,str);
          printf("Blue power: ");
          printf(str);
          printf("\r\n");
          break;

        case 'g':
          if((green_pwr-PWR_INCREMENT)>=0)  {
            green_pwr = green_pwr - PWR_INCREMENT;
            pwm_set_gpio_level(RED_LED, green_pwr*green_pwr);
          }
          else  {
            green_pwr = 0;
          }
          intToStr(green_pwr,str);
          printf("Green power: ");
          printf(str);
          printf("\r\n");
          break;

        case 'G':
          if((green_pwr+PWR_INCREMENT)<=PWR_LIMIT)  {
            green_pwr = green_pwr + PWR_INCREMENT;
            pwm_set_gpio_level(GREEN_LED, green_pwr*green_pwr);
          }
          else  {
            green_pwr = PWR_LIMIT;
          }
          intToStr(green_pwr,str);
          printf("Green power: ");
          printf(str);
          printf("\r\n");
          break;

        default:
          printf("Valid options are:\r\n u - Pin 15 to pull up\r\n d - Pin 15 to pull down\r\n o - Pin 15 to pull none\r\n R/r - Red LED inc/dcr\r\n B/b - Blue LED inc/dcr\r\n G/g - Green LED inc/dcr\r\n");
          break;
      }
    }
  }
}

void intToStr(int N, char *str) {
    int i = 0;
  
    // Save the copy of the number for sign
    int sign = N;

    // If the number is negative, make it positive
    if (N < 0)
        N = -N;

    // Extract digits from the number and add them to the
    // string
    while (N > 0) {
      
        // Convert integer digit to character and store
          // it in the str
        str[i++] = N % 10 + '0';
          N /= 10;
    } 

    // If the number was negative, add a minus sign to the
    // string
    if (sign < 0) {
        str[i++] = '-';
    }

    // Null-terminate the string
    str[i] = '\0';

    // Reverse the string to get the correct order
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = str[j];
        str[j] = str[k];
        str[k] = temp;
    }
}
