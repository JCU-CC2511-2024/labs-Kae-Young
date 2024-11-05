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

//  define writeable line
#define START_LINE  3

//  define PWM settings
#define MAX_PWR     255

//  define random limits
#define A_UPPER_LIMIT   20

int main(void) {
  //  Q1a - initialize IO
  stdio_init_all();
  //  END Q1a

  //  Q1b - print title and name
  printf("CC2511 Exam 2022\r\n");
  printf("Kae Young\r\n");
  //  END Q1b

  //  Q2a - configure adc pin
  adc_init();
  adc_gpio_init(LDR_PIN);
  adc_select_input(0);
  //  END Q2a

  //  Q2b - init conversion factor
  const float conversion_factor = 3.3f / (1 << 12);
  //  END Q2b

  //  Q2c
  int ldr_readings_count = 0; //initialize ldr count
  float past_ldr_readings[5]; //initialize ldr readings array
  //  END Q2c

  //  Q2d - Initialize LEDs
  // Configure blue LED
  gpio_init(BLUE_LED);                                    //init LED GPIO
  gpio_set_dir(BLUE_LED, GPIO_OUT);                       //set LED GPIO to output
  gpio_set_function(BLUE_LED, GPIO_FUNC_PWM);             //set LED GPIO to PWM function
  uint blue_slice_num = pwm_gpio_to_slice_num(BLUE_LED);  //get slice number
  pwm_set_enabled(blue_slice_num, true);                  //start generating PWM signal
  uint16_t blue_pwr = 0;                                  //init LED power value

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
  //  END Q2d

  //  Q3a - game active state
  bool is_game_active = false;
  //  END Q3a

  //  Q3b - configure random number generation
  int rand(void);
  absolute_time_t t0 = get_absolute_time();
  //  END Q3b

  //  Q3d - init score count
  int score = 0;

  while (true) {
    //  Q3a - Enter game on g
    int ch = getchar_timeout_us(0);
    if(ch == 'g')  
    {
      //clear LEDs
      pwm_set_gpio_level(RED_LED, 0);
      pwm_set_gpio_level(GREEN_LED, 0);
      pwm_set_gpio_level(BLUE_LED, 0);
      //clear text
      term_move_to(1, START_LINE);
      printf("                                            ");
      //3 sec countdown to game start
      term_move_to(1, START_LINE);
      printf("Game starts in 3...");
      sleep_ms(1000);
      term_move_to(1, START_LINE);
      printf("Game starts in 2...");
      sleep_ms(1000);
      term_move_to(1, START_LINE);
      printf("Game starts in 1...");
      sleep_ms(1000);
      //clear text
      term_move_to(1, START_LINE);
      printf("                                            ");
      //set game to active
      is_game_active = true;
      //  END Q3a

      //  Q3b - Seed random numbers
      int rseed = adc_read() * absolute_time_diff_us(get_absolute_time(), t0);
      void srand(rseed);  //seed from LDR value * on board time
      //  END Q3b

      //  Q3d - Reset score counter
      score = 0;
      //  END Q3d
    }

    // Q2 - LDR
    if (!is_game_active)
    {
      //  Q2b - sample light sensor and print results
      sleep_ms(1000);
      term_move_to(1, START_LINE);
      uint16_t ldr_raw = adc_read();
      float ldr_voltage = (float)ldr_raw * conversion_factor;
      printf("Raw value: %i, Voltage: %f\r\n", ldr_raw, ldr_voltage);
      //  END Q2b

      //  Q2c - 5 recent readings comparison
      int N = 5;  //define number of elements
      if (ldr_readings_count < 5)
      {
        N = ldr_readings_count; //if less than 5 readings have been made set N to number of readings that have been made
      }
      ldr_readings_count++; //increment ldr count
      bool is_ldr_count_greater_than_past_5 = true;
      bool is_ldr_count_less_than_past_5 = true; 
      for (int i = 0; i < N; i++)
      {
        if (ldr_voltage >= past_ldr_readings[i])
        {
          is_ldr_count_less_than_past_5 = false;
        }
        if (ldr_voltage <= past_ldr_readings[i])
        {
          is_ldr_count_greater_than_past_5 = false;
        }
      }
      past_ldr_readings[ldr_readings_count % 5] = ldr_voltage;  //store newest voltage
      /*
      (i) voltage is greater than last 5 if is_ldr_count_greater_than_past_5 = true
      (ii) voltage is less than last 5 if is_ldr_count_less_than_past_5 = true
      (iii) voltage is neither if both above = false
      */
      //  END Q2c

      //  Q2d - LED feedback from Q2c
      if (is_ldr_count_greater_than_past_5) //(i) green on
      {
        //green on
        pwm_set_gpio_level(GREEN_LED, MAX_PWR*MAX_PWR);
        //others off
        pwm_set_gpio_level(RED_LED, 0);
        pwm_set_gpio_level(BLUE_LED, 0);
      }
      else if(is_ldr_count_less_than_past_5)  //(ii) red on
      {
        //red on
        pwm_set_gpio_level(RED_LED, MAX_PWR*MAX_PWR);
        //others off
        pwm_set_gpio_level(GREEN_LED, 0);
        pwm_set_gpio_level(BLUE_LED, 0);
      }
      else  //(iii) blue on
      {
        //blue on
        pwm_set_gpio_level(BLUE_LED, MAX_PWR*MAX_PWR);
        //others off
        pwm_set_gpio_level(RED_LED, 0);
        pwm_set_gpio_level(GREEN_LED, 0);
      }
      //  END Q2d
    }
    // Q3 - Game
    else
    {
      //  Q3b - Generate random integers
      int a = rand() % A_UPPER_LIMIT; //generate random a between 0 -> 20
      int b_lower_boundary = (a <= 9) ? 0 : a - 9;  
      int b = b_lower_boundary + (rand() % (a - b_lower_boundary)); //generate rand b between a-9 -> a if a > 9 else between 0 -> a
      //  END Q3b

      // Q3c,d - Prompt user and give feedback, increment/decrement score and display it
      int answer = a - b; //calculate correct answer
      //ask user
      term_move_to(1, START_LINE);
      printf("What is %i - %i?                     \r\n", a, b);
      //get response from user
      ch = getchar_timeout_us(0);
      // Wait for input
      absolute_time_t start_time = get_absolute_time();
      while (ch == PICO_ERROR_TIMEOUT)
      {
        ch = getchar_timeout_us(0);
        if (absolute_time_diff_us(start_time, get_absolute_time()) >= 5000000)
        {
          ch = 9999;
          break;
        }
        
        sleep_ms(50);
      }
      if (ch == 9999)
      {
        printf("Out of time!                   \r\n");
        score--;
        printf("Score: %i", score);
      }
      else
      {
        int input = ch - '0';
        if (answer == input)
        {
          score++;
          term_move_to(1, START_LINE);
          printf("Correct!                       \r\n");
          printf("Score: %i", score);
        }
        else
        {
          score--;
          term_move_to(1, START_LINE);
          printf("Incorrect. The answer was %i   \r\n", answer);
          printf("Score: %i", score);
        }
        //  END Q3c,d
      }

      // Q3d,e - End game if score = -5 | 5
      if (score <= -5)
      {
        //lose message
        term_move_to(1, START_LINE);
        printf("You lose :(                    \r\n");
        printf("              ");
        is_game_active = false;  
      }
      if (score >= 5)
      {
        //win message
        term_move_to(1, START_LINE);
        printf("You win! :)                    \r\n");
        printf("              ");
        is_game_active = false;
      }
      // END Q3d,e
      
      // TIMEOUT FEATURE ACHEIVED

      

      sleep_ms(5000); //wait 5sec
      //clear space
      term_move_to(1, START_LINE);
      printf("                                \r\n");
      printf("                                \r\n");
    }
  }
}
