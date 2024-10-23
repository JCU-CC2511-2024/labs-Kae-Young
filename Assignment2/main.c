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
#define X_MAX 8000
#define Y_MAX 5450
#define Z_MAX 1800
#define STEP_SLEEP 800

#define SPINDLE 22
#define SPIN_MAX 255


#define LEN(arr) ((int) (sizeof (arr) / sizeof (arr)[0]))   //LEN(arr) for number of rows //LEN(arr[0]) for number of columns

/* 
################################################################
                        UI FRAMEWORK
################################################################                                             
*/
// UI box struct contains height and width information
typedef struct box {
    int width;
    int height;
    int x_origin;
    int y_origin;
    char *header;
    bool is_heading_centered;
}   box_T;

// declare boxes
box_T win_box;
box_T xyz_box;
box_T opt_box;
box_T in_box;
box_T out_box;

void clear_ui() {
    term_move_to(win_box.x_origin,win_box.y_origin);
    term_set_color(clrBlack, clrBlack);
    for (int i = 0; i < win_box.height + 1; i++)
    {
        for (int j = 0; j < win_box.width + 1; j++)
        {
            printf(" ");
            if(j == win_box.width)   {
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

// Clear input box
void clr_input()     {
    term_set_color(clrGreen, clrBlack);
    int x_cursor = in_box.x_origin + 5;
    int y_cursor = in_box.y_origin + 2;
    term_move_to(x_cursor, y_cursor);
    printf("                                                                    ");
    term_move_to(x_cursor, y_cursor);
}
// Clear output box
void clr_output()    {
    term_set_color(clrGreen, clrBlack);
    int x_cursor = out_box.x_origin + 2;
    int y_cursor = out_box.y_origin + 2;
    term_move_to(x_cursor, y_cursor);
    printf("                                                                    ");
}

// Print to output box
void print_output(char output[])  {
    clr_output();
    term_set_color(clrGreen, clrBlack);
    int x_cursor = out_box.x_origin + 2;
    int y_cursor = out_box.y_origin + 2;
    term_move_to(x_cursor, y_cursor);
    uart_puts(UART_ID, output);
    clr_input();
}

const int coord_text_width = 9;
const int coord_text_height = 3;
// Print coordinates
void print_coords(int coords[]) {
    int normalised_coords[3];
    int limits[3] = {X_MAX, Y_MAX, Z_MAX};
     for (int i = 0; i < 3; i++)
    {
        normalised_coords[i] = round(((double)coords[i]/(double)limits[i])*100);    //wrong max value
    }
    
    term_set_color(clrGreen, clrBlack);
    //set cursor position
    int x_cursor = round(((xyz_box.width+2) - coord_text_width)/2 + xyz_box.x_origin) + 4;
    int y_cursor = round(((xyz_box.height+2) - coord_text_height)/2 + xyz_box.y_origin);
    for (int i = 0; i < coord_text_height; i++)
    {
        term_move_to(x_cursor, i + y_cursor);
        if (normalised_coords[i] <= 9)
        {
            printf("00%i %%", normalised_coords[i]);
        }
        else if (normalised_coords[i] <= 99)
        {
            printf("0%i %%", normalised_coords[i]);
        }
        else
        {
            printf("%i %%", normalised_coords[i]);
        }
    }
    clr_input();
}

// Draw UI
void draw_ui()  {
    //the window is made up of a 9x9 grid
    double width = win_box.width;
    double height = win_box.height;
    double x_grid_step = width/9;
    double y_grid_step = height/9;

    // Configure xyz box
    xyz_box.width = round(2*x_grid_step);
    xyz_box.height = round(3*y_grid_step);
    xyz_box.x_origin = win_box.x_origin + round(1*x_grid_step);
    xyz_box.y_origin = win_box.y_origin + round(1*y_grid_step);
    xyz_box.header = "Coordinates";
    xyz_box.is_heading_centered = true;

    // Configure options box
    opt_box.width = round(4*x_grid_step);
    opt_box.height = xyz_box.height;                                  //equal height as xyz_box
    opt_box.x_origin = win_box.x_origin + round(win_box.width - x_grid_step - opt_box.width);   //equal width as xyz_box
    opt_box.y_origin = xyz_box.y_origin;                              //vertically aligned with xyz_box
    opt_box.header = "Options";
    opt_box.is_heading_centered = true;

    // Configure input box
    in_box.width = round(xyz_box.width + opt_box.width + x_grid_step);     //same width as left of xyz_box to right of opt_box
    in_box.height = round(1.5*y_grid_step);                                  
    in_box.x_origin = xyz_box.x_origin;                                     //aligned horizontally with xyz_box
    in_box.y_origin = win_box.y_origin + round(xyz_box.height + 2*y_grid_step);                    
    in_box.header = "Input";
    in_box.is_heading_centered = false;

    //Configure output box
    out_box.width = in_box.width;                       //equal width as input box
    out_box.height = in_box.height;                     //equal height as output box
    out_box.x_origin = in_box.x_origin;                 //horizontally aligned with input box
    out_box.y_origin = in_box.y_origin + in_box.height; //just below input box
    out_box.header = "Output";
    out_box.is_heading_centered = false;

    // Draw UI frame
    clear_ui();
    draw_box(win_box);
    draw_box(xyz_box);
    draw_box(opt_box);
    draw_box(in_box);
    draw_box(out_box);

    // Draw coord box contents
    term_set_color(clrGreen, clrBlack);
    int x_cursor = round(((xyz_box.width+2) - coord_text_width)/2 + xyz_box.x_origin);
    int y_cursor = round(((xyz_box.height+2) - coord_text_height)/2 + xyz_box.y_origin);
    char xyz[] = {'x', 'y', 'z'};
    for (int i = 0; i < coord_text_height; i++)
    {
        term_move_to(x_cursor, i + y_cursor);
        printf("%c : ", xyz[i]);
    }

    // Draw options box contents
    
    char options[7][26] = {"move - manual control", "home - move to [0 0 0]", "load - load prefab", "zero - set to [0 0 0]", "setz - set spindle height", "resize - resize Window", "spin - set spindle on/off"};
    int num_of_options = LEN(options);
    int max_length = LEN(options[0]);
    x_cursor = round(((opt_box.width+2) - max_length)/2 + opt_box.x_origin);
    y_cursor = round(((opt_box.height+2) - num_of_options)/2 + opt_box.y_origin);
    for (int i = 0; i < num_of_options; i++)
    {
      term_move_to(x_cursor, y_cursor + i);
      printf(options[i]);
    }
    
    // Print coordinates
    int zero[3] = {0, 0, 0};
    print_coords(zero);

    // Draw input ready
    x_cursor = in_box.x_origin + 3;
    y_cursor = in_box.y_origin + 2;
    term_move_to(x_cursor, y_cursor);
    printf("> ");

    print_output("Ready for commands...\n");
    clr_input();
}

/*
###############################################################
                    END OF UI FRAMEWORK
###############################################################
*/

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
                myIndex = 0;
                input_ready = true;
                break;
            case '\r':
                buffer[myIndex] = 0;
                clr_input();
                myIndex = 0;
                input_ready = true;
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

// Define axis struct
typedef struct axis {
    int current_position;
    int target_position;
    int min_position;
    int max_position;
    int steps_to_move; 
    uint step_pin;
    uint dir_pin;
} axis_T;

// declare axes
axis_T x;
axis_T y;
axis_T z;

void move_z_motor(axis_T* z)    {
    // Set direction
    gpio_put(z->dir_pin, z->steps_to_move > 0);  
        
    // Disregard sign
    z->steps_to_move = abs(z->steps_to_move);

    // Move motor
    for (int i = 0; i < z->steps_to_move; i++) {
        gpio_put(z->step_pin, true);
        sleep_us(STEP_SLEEP);
        gpio_put(z->step_pin, false);
        sleep_us(STEP_SLEEP);
    }

    // Set current position to original target position
    z->current_position = z->target_position;
    z->steps_to_move = 0;

    // Output message
    char message[50];
    snprintf(message, sizeof(message), "Moved to position: %d\n", z->current_position);
    print_output(message);
}

// Generalized function to move motor to a target position
void move_to_position(axis_T* x, axis_T* y, axis_T* z) {
    if (x->target_position < x->min_position || x->target_position > x->max_position) {
        char message[50];
        sprintf(message, "Error: x position out of bounds (0-%d).", X_MAX);
        print_output(message);
        return;
    }
    if (y->target_position < y->min_position || y->target_position > y->max_position) {
        char message[50];
        sprintf(message, "Error: y position out of bounds (0-%d).", Y_MAX);
        print_output(message);
        return;
    }
    if (z->target_position < z->min_position || z->target_position > z->max_position) {
        char message[50];
        sprintf(message, "Error: z position out of bounds (0-%d).", Z_MAX);
        print_output(message);
        return;
    }

    // Calculate steps to move
    x->steps_to_move = x->target_position - x->current_position;
    y->steps_to_move = y->target_position - y->current_position;
    z->steps_to_move = z->target_position - z->current_position;

    // Do nothing if already at position
    if (x->steps_to_move == 0 && y->steps_to_move == 0 && z->steps_to_move == 0)
    {
        print_output("Already at the target position.\n");
        return;
    }
    
    // Do z motor first if raising spindle
    if (z->steps_to_move != 0 & z->current_position != 0)
    {
        move_z_motor(z);
        // Set current position to original target position
        z->current_position = z->target_position;
    }
    
    // Set direction
    gpio_put(x->dir_pin, x->steps_to_move > 0);  
    gpio_put(y->dir_pin, y->steps_to_move > 0);

    // Disregard sign
    x->steps_to_move = abs(x->steps_to_move);
    y->steps_to_move = abs(y->steps_to_move);

    // N = MIN(delta_x, delta_y)/MIN_STEP
    double num_of_increments;
    if (x->steps_to_move == 0)
    {
        num_of_increments = y->steps_to_move;
    }
    else if (y->steps_to_move == 0)
    {
        num_of_increments = x->steps_to_move;
    }
    else    {
        num_of_increments = MIN(x->steps_to_move, y->steps_to_move);
    }

    double dx = (double)x->steps_to_move/num_of_increments;
    double dy = (double)y->steps_to_move/num_of_increments;

    

    double last_x_position = 0;
    double last_y_position = 0;

    for (int N = 0; N < (int)num_of_increments; N++)
    {
        double target_x_position = N*dx;
        double target_y_position = N*dy;

        int actual_dx = round(target_x_position - last_x_position);
        int actual_dy = round(target_y_position - last_y_position);

        last_x_position = last_x_position + actual_dx;
        last_y_position = last_y_position + actual_dy;

        // Find maximum steps out of x and y axis
        int max_steps = MAX(actual_dx, actual_dy);

        // Move x and y motors simultaneously
        for (int i = 0; i < max_steps; i++) {
            // Turn x stepper motor on
            if (i < actual_dx)
            {
                gpio_put(x->step_pin, true);
            }
            // Turn y stepper motor on
            if (i < actual_dy)
            {
                gpio_put(y->step_pin, true);
            }
            // Turn motor off after a duration
            sleep_us(STEP_SLEEP);
            gpio_put(x->step_pin, false);
            gpio_put(y->step_pin, false);
            sleep_us(STEP_SLEEP);
        } 
    }
    
    // Set current position to original target position
    x->current_position = x->target_position;
    y->current_position = y->target_position;

    // Set steps to move to 0
    x->steps_to_move = 0;
    y->steps_to_move = 0;
    
    // Do z motor last if lowering spindle
    if (z->steps_to_move != 0 & z->current_position == 0)
    {
        move_z_motor(z);
        // Set current position to original target position
        z->current_position = z->target_position;
    }

    char message[50];
    snprintf(message, sizeof(message), "Moved to position: x %d, y %d, z %d\n", x->current_position, y->current_position, z->current_position);
    print_output(message);
}

void print_sequence(int sequence[][3], int sequence_length, axis_T* x, axis_T* y, axis_T* z)    {
    for (int i = 0; i < sequence_length; i++)
    {
        x->target_position = sequence[i][0];
        y->target_position = sequence[i][1];
        z->target_position = sequence[i][2];

        move_to_position(x, y, z);
        int coords[3];
        coords[0] = x->current_position;
        coords[1] = y->current_position;
        coords[2] = z->current_position;
        print_coords(coords);
    }
}

void setup_pwm() {
    gpio_set_function(SPINDLE, GPIO_FUNC_PWM);
    uint spindle_slice_num = pwm_gpio_to_slice_num(SPINDLE);
    int spindle_speed = 0;
    pwm_set_wrap(spindle_slice_num, 255);
    pwm_set_gpio_level(SPINDLE, spindle_speed);
    pwm_set_enabled(spindle_slice_num, spindle_speed);
}

void spindle_on(int spindle_speed) {
    pwm_set_gpio_level(SPINDLE, spindle_speed);
}

int len_of_text(char arr[]) {
    int i = 0;
    while (arr[i] != '\000')
    {
        i++;
    }
    return i;
}
/*
#################################################################
                            Main
#################################################################
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

    int x_steps = 0;

    // Configure window box
    // TIP: UI works better when width and height are multiples of 9
    win_box.width = 150;                        //set box width
    win_box.height = 33;                        //set box height
    win_box.x_origin = 5;                       //set box x origin
    win_box.y_origin = 5;                       //set box y origin
    win_box.header = "CC2511 Assignment 2";     //set box header
    win_box.is_heading_centered = true;         //set heading alignment

    draw_ui();

    setup_pwm();
    spindle_on(0);

    // define axes attributes
    x.max_position = X_MAX;
    x.min_position = MIN_POSITION;
    x.current_position = 0;
    x.target_position = 0;
    x.step_pin = STEP_PIN_X;
    x.dir_pin = DIR_PIN_X;

    y.max_position = Y_MAX;
    y.min_position = MIN_POSITION;
    y.current_position = 0;
    y.target_position = 0;
    y.step_pin = STEP_PIN_Y;
    y.dir_pin = DIR_PIN_Y;

    z.max_position = Z_MAX;
    z.min_position = MIN_POSITION;
    z.current_position = 0;
    z.target_position = 0;
    z.step_pin = STEP_PIN_Z;
    z.dir_pin = DIR_PIN_Z;

    int z_down = 1000;
    int z_up = z_down - 200;

    /*
    ############################################################
                        Define Sequences
    ############################################################
    */
    // define house
    const int house[][3] = {
        // Outline
        {1000, 0, z_down},
        {5000, 0, z_down},
        {5000, 2000, z_down},
        {6000, 2000, z_down},
        {3000, 4000, z_down},
        {0, 2000, z_down},
        {1000, 2000, z_down},
        {1000, 0, z_down},
        //Chimney
        {1500, 3000, z_up},
        {1500, 3000, z_down},
        {1500, 3500, z_down},
        {1000, 3500, z_down},
        {1000, 2667, z_down},
        //Door
        {3200, 0, z_up},
        {3200, 0, z_down},
        {3200, 1600, z_down},
        {4200, 1600, z_down},
        {4200, 0, z_down},
        //Window
        {1600, 800, z_up},
        {1600, 800, z_down},
        {1600, 1600, z_down},
        {2400, 1600, z_down},
        {2400, 800, z_down},
        {1600, 800, z_down},
        {2000, 800, z_up},
        {2000, 800, z_down},
        {2000, 1600, z_down},
        {1600, 1200, z_up},
        {1600, 1200, z_down},
        {2400, 1200, z_down},
        // Home
        {0, 0, z_up}
    };

    /*
    ############################################################
                      End Sequence Definition
    ############################################################
    */

    // Initialize coordinate array
    volatile int coords[] = {x.current_position, y.current_position, z.current_position};

    // Initialize spindle speed
    int spindle_speed = 0;
    

    while (true) {
        spindle_on(spindle_speed);
        // Wait for input
        while (!input_ready) {
            __asm("wfi");  // Wait for interrupt
        }
        // Store commands
        char option_move[20] = "move";
        char option_load[20] = "load";
        char option_zero[20] = "zero";
        char option_resize[20] = "resize";
        char option_home[20] = "home";
        char option_setz[20] = "setz";
        char option_spin[20] = "spin";
        // Process input
        char command[20] = "\000";
        char argument[20] = "\000";
        
        int input = sscanf(buffer, "%s %[^\t\n]", &command, &argument);
        
        // MANUAL CONTROL
        if (strcmp(command, option_move) == 0)
        {
            // Set temp coord values
            coords[0] = 9999;
            coords[1] = 9999;
            coords[2] = 9999;

            // Scan input for target coords
            input = sscanf(argument, "%d %d %d", &coords[0], &coords[1], &coords[2]);
            
            // Check if all coords have been set by user
            if (coords[0] == 9999 || coords[1] == 9999 || coords[2] == 9999)
            {
                // If not then show user the correct syntax
                print_output("Syntax: \"move [x] [y] [z]\"");
            }
            else
            {
                // If yes then move to target coords
                x.target_position = coords[0];
                y.target_position = coords[1];
                z.target_position = coords[2];
                move_to_position(&x, &y, &z);
                print_coords(coords);
            }
        }
        // LOAD
        else if (strcmp(command, option_load) == 0)
        {
            // Store prefab options
            char option_house[20] = "house";

            // Preallocate prefab option input
            char sequence[20];
            input = sscanf(argument, "%s", &sequence);

            // Check prefabs
            // house
            if (strcmp(sequence, option_house) == 0)
            {
                print_sequence(house, LEN(house), &x, &y, &z);
                print_output("Sequence: house, completed");
            }
            else
            {
                print_output("Syntax: \"load [prefab]\". Available prefabs are: house");
            }
        }
        // ZERO
        else if (strcmp(command, option_zero) == 0)
        {
            x.current_position = 0;
            y.current_position = 0;
            z.current_position = 0;
            print_output("Coordinates zeroed");
            coords[0] = x.current_position;
            coords[1] = y.current_position;
            coords[2] = z.current_position;
            print_coords(coords);
        }
        // RESIZE
        else if (strcmp(command, option_resize) == 0)    
        {
            // Set temp window parameters
            uint width = 9999;
            uint height = 9999;
            uint x_origin = 9999;
            uint y_origin = 9999;

            input = sscanf(argument, "%d %d %d %d", &width, &height, &x_origin, &y_origin);

            // Check if crucial parameters have changed
            if (width == 9999 || height == 9999)
            {
                // If unchanged show user correct syntax
                print_output("Syntax: \"resize [width] [height] [*x origin] [*y origin]\" *optional");
            }
            else
            {
                // If changed draw new window
                clear_ui();
                win_box.height = height;
                win_box.width = width;

                // Check if origin points are changed
                if (x_origin == 9999 || y_origin == 9999)
                {
                    // If unchanged default origin to [1, 1]
                    win_box.x_origin = 1;
                    win_box.y_origin = 1;
                }
                else
                {
                    win_box.x_origin = x_origin;
                    win_box.y_origin = y_origin;
                }
                
                draw_ui();
            }
        }
        // HOME
        else if (strcmp(command, option_home) == 0)
        {
            x.target_position = 0;
            y.target_position = 0;
            z.target_position = 0;
            move_to_position(&x, &y, &z);
            x.current_position = x.target_position;
            y.current_position = y.target_position;
            z.current_position = z.target_position;
            coords[0] = x.current_position;
            coords[1] = y.current_position;
            coords[2] = z.current_position;
            print_coords(coords);
        }
        // SETZ
        else if (strcmp(command, option_setz) == 0)
        {
            // Set temp depth value
            int depth = 9999;

            input = sscanf(argument, "%d", &depth);

            // Check if depth is unchanged
            if (depth == 9999)
            {
                // If unchanged show user correct syntax
                print_output("Syntax: \"setz [depth]\"");
            }
            else
            {
                // If changed check if valid height
                if ((z.current_position + depth) > Z_MAX)
                {
                    char message[50];
                    sprintf(message, "Error: maximum allowed depth at this height is %d", Z_MAX - z.current_position);
                    print_output(message);
                }
                else
                {
                    z_up = z.current_position;
                    z_down = z_up + depth;
                    char message[50];
                    sprintf(message, "Z up set to %d, Z down set to %d", z_up, z_down);
                    print_output(message);
                }
            }
        }
        else if (strcmp(command, option_spin) == 0)
        {
            // Set temp speed
            int speed = 9999;

            // Process input
            input = sscanf(argument, "%d", &speed);

            // Check if speed has been changed
            if (speed == 9999)
            {
                // If unchanged show user the correct syntax
                char message[50];
                sprintf(message, "Syntax: \"spin [speed]\"", SPIN_MAX);
                print_output(message);
            }
            else if (speed < 0 || speed > SPIN_MAX)
            {
                // If speed is out of bounds show error message
                char message[50];
                sprintf(message, "Error: spindle speed out of bounds (0-%d)", SPIN_MAX);
                print_output(message);
            }
            
            else
            {
                // If changed set spindle speed
                spindle_speed = speed;
                char message[50];
                sprintf(message, "Spindle speed set to %d", spindle_speed);
                print_output(message);
            } 
        }
        // INVALID COMMAND
        else
        {
            print_output("Invalid command.");
            clr_input(in_box);
        }
        
        for (int i = 0; i < 20; i++)
        {
            command[i] = "\000";
            argument[i] = "\000";
        }

        input_ready = false;  // Reset input flag
        spindle_on(0);
    }
}
