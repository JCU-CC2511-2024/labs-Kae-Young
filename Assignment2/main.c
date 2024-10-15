/**************************************************************
 * main.c
 * rev 1.0 11-Oct-2024 Clarisse Acero | Glen White | Kae Young
 * Assignment2
 * ***********************************************************/

/* 
  **** READ ME ****

  ASSIGNMENT OBJECTIVES:
    • Control of the three stepper motors to enable arbitrary positioning of the cutting tool.
    • Control over the spindle motor including varying the rotational speed of the cutting tool.
    • Manual control using your choice of input methods to enable the user to directly command each of the motors.
    • OPTIONAL: operation as an engraving machine. In this mode, the user will supply an image or a sequence of 
      coordinates to be cut, and your program will autonomously control the mill in order to engrave the desired image.

  TODO LIST:
    • Create UI:
      - Operation selection
      - Manual controls
    • Spindle configuration
    • Stepper motor configuration
    • Mapping functionality:
      - Zeroing function (user sets coordinates [stepper_x, stepper_y, stepper_z] to [0, 0, 0])
      - Map physical motor increments to virtual coordinate increments
        - Have stepper motor steps
        - convert steps to co-ordinates 
        - Define coordinate boundaries that the motors should not surpass
    • Design multiple coordinate system for drilling predefined pictures

    EDIT ABOVE AS NEEDED AND TRY TO KEEP UP WITH COMMENTING AND COMMIT DESCRIPTIONS. COMMIT FREQUENTLY IDEALY FOR EACH CHANGE YOU MAKE. 
*/

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <string.h>
#include "terminal.h"


// uart stuff
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// UART using pins 0 and 1
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Stepper motor pins
#define STEP_PIN_X  11
#define DIR_PIN_X   12
#define SLEEP_PIN   17
#define RESET_PIN   18

// uart stuff
static int chars_rxed = 0;
volatile char buffer [100];
volatile unsigned int myIndex = 0;
volatile bool input_ready = false;

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        // Echo back the character received
        send_ch(ch);
        // Detect character
        switch (ch) {
            // Check if 'Enter' is pressed
            case '\n':                          
                buffer[myIndex] = 0;
                uart_puts(UART_ID, "\n");
                myIndex = 0;
                input_ready = true;
                break;
            case '\r':
                break;
            // Backspace handling
            case '\177':                        
                if (myIndex > 0) {
                    myIndex--;
                    buffer[myIndex] = '\000';
                }
                break;
            // Save the character to buffer
            default:
                if (myIndex <= 98)  {           
                    buffer[myIndex] = ch;      
                    myIndex++; 
                }
        }
        chars_rxed++;
    }
}

// Write character
void send_ch(char ch)   {
    if(uart_is_writable(UART_ID))   {
        uart_putc(UART_ID, ch);
    }
}

// Initialise pin
void init_pin(uint pin, bool direction) {
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}

// Function to step the motor 'steps' times
void step_motor(int steps, int delay_us) {
    for (int i = 0; i < steps; i++) {
        // Pulse the STEP pin
        gpio_put(STEP_PIN_X, true);
        sleep_us(delay_us);
        gpio_put(STEP_PIN_X, false);
        sleep_us(delay_us);
    }
}

/* 
################################################################
                    FUNCTIONS TO DRAW UI
################################################################                                             
*/
// UI window struct contains height and width information
// Get window width from w1.width
// Get window height from w1.height
typedef struct ui_window {
    int width;
    int height;
}   ui_window_T;

// Draw heading
void draw_heading(ui_window_T w1) {
    char text[] = "CC2511 Assignment 2";
    int cursor_location = round((w1.width-sizeof(text))/2); //set cursor to middle of window line
    
    term_move_to(cursor_location,1);
    term_set_color(clrGreen, clrBlack);
    printf(text);
}

void draw_box() {

}

/*
###############################################################
                    END OF UI FUNCTIONS
###############################################################
*/

int main(void) {
    // Initialise components
    stdio_init_all();
  
    // Initialize GPIO pins
    init_pin(STEP_PIN_X, GPIO_OUT);
    init_pin(DIR_PIN_X, GPIO_OUT);
    init_pin(RESET_PIN, GPIO_OUT);
    init_pin(SLEEP_PIN, GPIO_OUT);

    // Set direction forward and wake up the driver
    gpio_put(DIR_PIN_X, true);  // Forward direction
    gpio_put(SLEEP_PIN, true);  // Enable driver
    gpio_put(RESET_PIN, true);  // Reset any faults
  
    // Set up UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    uart_puts(UART_ID, "Ready for commands...\n");

    int x_steps = 0;

    // Set window size
    ui_window_T w1;    //declare window struct
    w1.width = 80;          //set window width
    w1.height = 20;         //set window height


    //Draw UI
    draw_heading(w1);

    while (true) {
        // Wait for input
        while (!input_ready) {
            __asm("wfi");  // Wait for interrupt
        }

          if (sscanf(buffer, "x %d", &x_steps) == 1) {
            if (x_steps < 0) {
                // If steps are negative, set direction to reverse
                gpio_put(DIR_PIN_X, false);
                x_steps = abs(x_steps);  // Use the absolute value of steps
            } else {
                // If steps are positive, set direction to forward
                gpio_put(DIR_PIN_X, true);
            }
            
            // Move motor with a delay of 1000 us per step
            step_motor(x_steps, 500);  
        } 
        else {
            uart_puts(UART_ID, "Invalid command. Use 'x <steps>'\n");
        }

        input_ready = false;  // Reset input flag for the next command
    }
}
