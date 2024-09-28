/*
 * @file : transfer_function.c
 * @details:
 * Transfer Function Module
 *
 * This module provides functions to compute the control surface positions for an airplane 
 * based on the angular positions obtained from PID controllers. The module includes 
 * transfer functions for three primary control angles: roll, pitch, and yaw.
 *
 * The transfer functions are designed as simple first-order systems characterized by a 
 * gain factor and a time constant. These functions translate the angular inputs into 
 * appropriate control surface positions to achieve the desired aircraft orientation.
 *
 * Definitions:
 * - GAIN_FACTOR: The coefficient that amplifies the input signal. It represents the 
 *   effectiveness of the control surface.
 * - TIME_CONSTANT: A parameter representing the system's responsiveness. Higher values 
 *   indicate slower system response.
 * - timesample: The time step between successive calculations. It is used for 
 *   discretizing the continuous control input.
 * - target_angular_velocity: The angular position input to the transfer function, derived from the 
 *   PID controller output.
 * 
 * @author : Dmytro Humennyi
 * @date : Jul 15 2024
 * @version : V0.01
 */

#include <stdlib.h>
#include "transfer_function.h"
#include "angular_datatypes.h"


fdata transfer_function_x(fdata target_angular_velocity_oX, fdata timesample)
{
    fdata transfer_function_oX = (GAIN_FACTOR_X * target_angular_velocity_oX) / (1 + TIME_CONSTANT_X * timesample);
    return transfer_function_oX;
}

fdata transfer_function_y(fdata target_angular_velocity_oY, fdata timesample)
{
    fdata transfer_function_oX = (GAIN_FACTOR_Y * target_angular_velocity_oY) / (1 + TIME_CONSTANT_Y * timesample);
    return transfer_function_oX;
}

fdata transfer_function_z(fdata target_angular_velocity_oZ, fdata timesample)
{
    fdata transfer_function_oX = (GAIN_FACTOR_Z * target_angular_velocity_oZ) / (1 + TIME_CONSTANT_Z * timesample);
    return transfer_function_oX;
}