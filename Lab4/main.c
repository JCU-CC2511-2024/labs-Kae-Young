/**************************************************************
 * main.c
 * rev 1.0 15-Aug-2024 kaeyo
 * Lab4
 * ***********************************************************/

#include <stdbool.h>

//DEFINE BASE REGISTERS
#define SIO_BASE          0xd0000000
#define PAD_BASE          0x4001c000
#define CTRL_BASE         0x40014000

//DEFINE OFFSET REGISTERS
#define GPIO_OE_SET       (SIO_BASE + 0x024)
#define GPIO_OUT_SET      (SIO_BASE + 0x014)
#define GPIO_OUT_CLEAR    (SIO_BASE + 0x018)
#define GPIO25_PAD        (PAD_BASE + 0x68)
#define GPIO25_CTRL       (CTRL_BASE + 0x0cc)

//DEFINE MISC
#define LED_PIN 25
#define CONTENT_OF(addr) (*(volatile uint32_t*)addr)

//DEFINE TYPES
typedef unsigned long uint32_t;


int main(void) {
  bool state = false;

  //Set output disable to off and input enable to on
  CONTENT_OF(GPIO25_PAD) = CONTENT_OF(GPIO25_PAD) & ~(1 << 7) | (1 << 6);

  //Set the GPIO function to SIO
  CONTENT_OF(GPIO25_CTRL) = 5;

  //Set the data direction
  CONTENT_OF(GPIO_OE_SET) = CONTENT_OF(GPIO_OE_SET) | (1 << LED_PIN);

  while (true) {
    //Toggle the state of the pin
    if (state)  {
      CONTENT_OF(GPIO_OUT_CLEAR) = (1 << 25); 
      state = !state;
    }
    else{
      CONTENT_OF(GPIO_OUT_SET) = (1 << 25); 
      state = !state;
    }

    //Wait
    for(uint32_t i = 0; i < 8000000; i++) {
      __asm volatile ("nop");
    }
  }
}
