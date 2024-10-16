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
#include <math.h>


// uart stuff
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// UART using pins 0 and 1
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define UART_TX_PIN 0
#define UART_RX_PIN 1

//Stepper motors
#define HOME_PIN_Z  2
#define STEP_PIN_Z  3 // 3 Z forwards is forwards
#define DIR_PIN_Z   6 // 6

#define HOME_PIN_Y  7
#define STEP_PIN_Y  8 // Y forwards is down
#define DIR_PIN_Y   9 //

#define HOME_PIN_X  10
#define STEP_PIN_X  11  // 11 X forwards is right
#define DIR_PIN_X   12  // 12

#define ENABLE_PIN  14
#define FAULT_PIN   15
#define DECAY_PIN   16
#define SLEEP_PIN   17
#define RESET_PIN   18

#define MIN_POSITION 0
#define X_MAX 5000
#define Y_MAX 5000 //2200
#define Z_MAX 2000 

// Current position of the motor (in steps)
int x_current_position = 0;
int y_current_position = 0;
int z_current_position = 0;

// uart stuff
static int chars_rxed = 0;
volatile char buffer [100];
volatile unsigned int myIndex = 0;
volatile bool input_ready = false;

// Write character
void send_ch(char ch)   {
    if(uart_is_writable(UART_ID))   {
        uart_putc(UART_ID, ch);
    }
}

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

void init_pin(uint pin, bool direction) {
    gpio_init(pin);
    gpio_set_dir(pin, direction);
}


// Generalized motor stepping function
void step_motor(uint step_pin, int steps, int delay_us) {
    for (int i = 0; i < steps; i++) {
        gpio_put(step_pin, true);
        sleep_us(delay_us);
        gpio_put(step_pin, false);
        sleep_us(delay_us);
    }
}

// Generalized function to move motor to a target position
void move_to_position(int *current_position, int target_position, uint step_pin, uint dir_pin, int max_position) {
    if (target_position < MIN_POSITION || target_position > max_position) {
        uart_puts(UART_ID, "Error: Position out of bounds.\n");
        return;
    }

    int steps_to_move = target_position - *current_position;

    if (steps_to_move != 0) {
        gpio_put(dir_pin, steps_to_move > 0);  // Set direction
        step_motor(step_pin, abs(steps_to_move), 800);  // Move motor
        *current_position = target_position;

        char message[50];
        snprintf(message, sizeof(message), "Moved to position: %d\n", *current_position);
        uart_puts(UART_ID, message);
    } else {
        uart_puts(UART_ID, "Already at the target position.\n");
    }
}
/* 
################################################################
                        UI FRAMEWORK
################################################################                                             
*/
// UI box struct contains height and width information
// Get box width from box.width
// Get box height from box.height
typedef struct box {
    int width;
    int height;
    int x_origin;
    int y_origin;
    char *header;
    bool is_heading_centered;
}   box_T;

void clear_ui(box_T b) {
    term_move_to(1,1);
    term_set_color(clrBlack, clrBlack);
    for (int i = 0; i < b.height + 1; i++)
    {
        for (int j = 0; j < b.width + 1; j++)
        {
            printf(" ");
            if(j == b.width)   {
                printf(" \r\n");
            }
        } 
    }  
}

// Draw heading
void draw_heading(box_T b) {
    //set colour
    term_set_color(clrBlack, clrGreen);

    int x_cursor_location; //set horizontal cursor
    if(b.is_heading_centered)
    {
        x_cursor_location = round(((b.width+2)-strlen(b.header))/2) + b.x_origin; //set to center of window
    }
    else
    {
        x_cursor_location = b.x_origin + 2; //set left aligned
    }
    int y_cursor_location = b.y_origin + 1; //set cursor to 1 line below top of box
    
    term_move_to(x_cursor_location, y_cursor_location);
    printf(b.header);
}

void draw_box(box_T b)    {
    //set colour
    term_set_color(clrGreen, clrBlack);

    //draw top border
    term_move_to(b.x_origin, b.y_origin);
    printf("+");
    for (int i = 0; i < b.width - 2; i++)
    {
        printf("-");
    }
    printf("+");

    //draw bottom border
    term_move_to(b.x_origin, b.y_origin + b.height);
    printf("+");
    for (int i = 0; i < b.width - 2; i++)
    {
        printf("-");
    }
    printf("+");

    //draw left border
    for (int i = 1; i < b.height; i++)
    {
        term_move_to(b.x_origin, b.y_origin + i);
        printf("|");
    }

    //draw right border
    for (int i = 1; i < b.height; i++)
    {
        term_move_to(b.x_origin + b.width-1, b.y_origin + i);
        printf("|");
    }

    if (b.header != "\0")
    {
        draw_heading(b);
    }
}

// Print coordinates
void print_coords(box_T xyz_box, int coords[], int coord_text_width, int coord_text_height) {
    term_set_color(clrGreen, clrBlack);
    //set cursor position
    int x_cursor = round(((xyz_box.width+2) - coord_text_width)/2 + xyz_box.x_origin) + 4;
    int y_cursor = round(((xyz_box.height+2) - coord_text_height)/2 + xyz_box.y_origin);
    for (int i = 0; i < coord_text_height; i++)
    {
        term_move_to(x_cursor, i + y_cursor);
        if (coords[i] <= 9)
        {
            printf("00%i", coords[i]);
        }
        else if (coords[i] <= 99)
        {
            printf("0%i", coords[i]);
        }
        else
        {
            printf("%i", coords[i]);
        }
    }
}
/*
###############################################################
                    END OF UI FRAMEWORK
###############################################################
*/

int main(void) {
    // Initialise components
    stdio_init_all();
  
    // Initialize GPIO pins
    // Initialize GPIO pins
    init_pin(STEP_PIN_X, GPIO_OUT);
    init_pin(DIR_PIN_X, GPIO_OUT);
    init_pin(STEP_PIN_Y, GPIO_OUT);
    init_pin(DIR_PIN_Y, GPIO_OUT);
    init_pin(STEP_PIN_Z, GPIO_OUT);
    init_pin(DIR_PIN_Z, GPIO_OUT);
    init_pin(RESET_PIN, GPIO_OUT);
    init_pin(SLEEP_PIN, GPIO_OUT);

    // Set direction forward and wake up the driver
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

    //uart_puts(UART_ID, "Ready for commands...\n");

    int x_steps = 0;

    // Initialize coordinate array
    int coords[] = {0, 0, 0};

    /*
    ###############################################################
                              DRAW UI
    ###############################################################
    */
    // Configure window box
    // TIP: UI works better when width and height are multiples of 9
    box_T win_box;                              //declare box struct
    win_box.width = 150;                        //set box width
    win_box.height = 25;                        //set box height
    win_box.x_origin = 1;                       //set box x origin
    win_box.y_origin = 1;                       //set box y origin
    win_box.header = "CC2511 Assignment 2";     //set box header
    win_box.is_heading_centered = true;         //set heading alignment

    //the window is made up of a 9x9 grid
    double width = win_box.width;
    double height = win_box.height;
    double x_grid_step = width/9;
    double y_grid_step = height/9;

    // Configure xyz box
    box_T xyz_box;
    xyz_box.width = round(2*x_grid_step);
    xyz_box.height = round(3*y_grid_step);
    xyz_box.x_origin = round(1*x_grid_step);
    xyz_box.y_origin = round(1*y_grid_step);
    xyz_box.header = "Coordinates";
    xyz_box.is_heading_centered = true;

    // Configure options box
    box_T opt_box;
    opt_box.width = round(4*x_grid_step);
    opt_box.height = xyz_box.height;                                  //equal height as xyz_box
    opt_box.x_origin = round(win_box.width - x_grid_step - opt_box.width);   //equal width as xyz_box
    opt_box.y_origin = xyz_box.y_origin;                              //vertically aligned with xyz_box
    opt_box.header = "Options";
    opt_box.is_heading_centered = true;

    // Configure input box
    box_T in_box;
    in_box.width = round(xyz_box.width + opt_box.width + x_grid_step);     //same width as left of xyz_box to right of opt_box
    in_box.height = round(1.5*y_grid_step);                                  
    in_box.x_origin = round(xyz_box.x_origin);                             //aligned horizontally with xyz_box
    in_box.y_origin = round(xyz_box.height + 2*y_grid_step);                    
    in_box.header = "Input";
    in_box.is_heading_centered = false;

    //Configure output box
    box_T out_box;
    out_box.width = in_box.width;                       //equal width as input box
    out_box.height = in_box.height;                     //equal height as output box
    out_box.x_origin = in_box.x_origin;                 //horizontally aligned with input box
    out_box.y_origin = in_box.y_origin + in_box.height; //just below input box
    out_box.header = "Output";
    out_box.is_heading_centered = false;

    // Draw UI frame
    clear_ui(win_box);
    draw_box(win_box);
    draw_box(xyz_box);
    draw_box(opt_box);
    draw_box(in_box);
    draw_box(out_box);

    // Draw coord box contents
    term_set_color(clrGreen, clrBlack);
    int coord_text_width = 7;
    int coord_text_height = 3;
    int x_cursor = round(((xyz_box.width+2) - coord_text_width)/2 + xyz_box.x_origin);
    int y_cursor = round(((xyz_box.height+2) - coord_text_height)/2 + xyz_box.y_origin);
    char xyz[] = {'x', 'y', 'z'};
    for (int i = 0; i < coord_text_height; i++)
    {
        term_move_to(x_cursor, i + y_cursor);
        printf("%c : ", xyz[i]);
    }

    // Draw options box contents
    
    static int num_of_options = 5;
    static int max_length = 22;
    char options[4][23] = {"MAN - Manual Control", "LOAD - Load File", "ZERO - Zero System", "RESIZE - Resize Window"};
    x_cursor = round(((opt_box.width+2) - max_length)/2 + opt_box.x_origin);
    y_cursor = round(((opt_box.height+2) - num_of_options)/2 + opt_box.y_origin);
    for (int i = 0; i < 4; i++)
    {
      term_move_to(x_cursor, y_cursor + i);
      printf(options[i]);
    }
    
    // Print coordinates
    print_coords(xyz_box, coords, coord_text_width, coord_text_height);

    // Draw input ready
    x_cursor = in_box.x_origin + 3;
    y_cursor = in_box.y_origin + 2;
    term_move_to(x_cursor, y_cursor);
    printf("> ");
    /*
    ###############################################################
                            DRAW UI END
    ###############################################################
    */

    int x_target_position = 0;
    int y_target_position = 0;
    int z_target_position = 0;

     while (true) {
        // Wait for input
        while (!input_ready) {
            __asm("wfi");  // Wait for interrupt
        }

        // Process input for x and y commands
        if (sscanf(buffer, "x %d", &x_target_position) == 1) {
            move_to_position(&x_current_position, x_target_position, STEP_PIN_X, DIR_PIN_X, X_MAX);
        } else if (sscanf(buffer, "y %d", &y_target_position) == 1) {
            move_to_position(&y_current_position, y_target_position, STEP_PIN_Y, DIR_PIN_Y, Y_MAX);
        } else if (sscanf(buffer, "z %d", &z_target_position) == 1) {
            move_to_position(&z_current_position, z_target_position, STEP_PIN_Z, DIR_PIN_Z, Z_MAX);
        } else {
            uart_puts(UART_ID, "Invalid command. Enter a valid position (0-5000).\n");
        }

        input_ready = false;  // Reset input flag
    }
}
