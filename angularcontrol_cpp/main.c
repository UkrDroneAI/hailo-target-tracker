/**
 * @file : main.c
 * @brief :
 * Main program demonstrating the angular motion control system.
 *
 * This program demonstrates the use of the angular motion control system using PID control.
 * It includes system initialization, calculation of angular velocities and accelerations, 
 * and application of PID control to manage angular motion.
 *
 * @details :
 * Main functions include:
 * - Initialization of the PID controller with specified gain values.
 * - Calculation of angular velocity between two angles over a given time interval.
 * - Calculation of angular acceleration between two angular velocities over a given time interval.
 * - Application of PID control to manage angular motion according to the desired setpoint.
 * 
 * @authored by Dmytro Humennyi
 * @date Jul 15 2024
 * @version V0.04
 */
#include "angular_control.h"
#include "angular_datatypes.h"
#include <stdlib.h>
#include <stdio.h>

int main() {
    fdata offset_x, offset_y, offset_z;
    fdata timesample = 0.1; // Example time sample value
    control_output control_outpul_val;
    
    
      
    while (1) {
        printf("\nPlease give me offsets for x, y and z:\t");
        scanf("%f %f %f", &offset_x, &offset_y, &offset_z);
        
        control_outpul_val = control_angular_motion_ex(offset_x, offset_y, offset_z, timesample,
                                                        TEST_CAMERA_WIDTH_RESOLUTION, TEST_CAMERA_HEIGHT_RESOLUTION,
                                                        TEST_CAMERA_HOR_FOV, TEST_CAMERA_VER_FOV);
        printf("Results: oX= %f ; oY= %f ; oZ= %f .\n", control_outpul_val.control_output_x, control_outpul_val.control_output_y,
                                                        control_outpul_val.control_output_z);

        char exitkey;
        printf("Next round? (n,y)?\n");
        scanf(" %c", &exitkey);
        if (exitkey == 'n') break;
    }

    return 0;
}
