/**************************************************************
 * main.c
 * rev 1.0 11-Oct-2024 kaeyo
 * Assignment2
 * ***********************************************************/

#include "pico/stdlib.h"
#include <stdbool.h>
#include "hardware/pwm.h"

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
      - Define coordinate boundaries that the motors should not surpass
    • Design multiple coordinate system for drilling predefined pictures

    EDIT ABOVE AS NEEDED AND TRY TO KEEP UP WITH COMMENTING AND COMMIT DESCRIPTIONS. COMMIT FREQUENTLY IDEALY FOR EACH CHANGE YOU MAKE. 
*/

int main(void) {
  // TODO - Initialise components and variables
  while (true) {
    // TODO - Repeated code here
  }
}
