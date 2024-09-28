/*
 * @file : angular_control.c
 *
 * @details:
 * Angular Control Module
 *
 * This module implements functions for controlling angular motion using PID control. 
 * It adjusts the control outputs to minimize the error between the desired setpoint and 
 * the measured value, ensuring precise and stable control.
 *
 * Functions:
 * - init_pid_controller: Initializes the PID controller with specified proportional, 
 *   integral, and derivative gains.
 * - compute_pid: Computes the control output based on the setpoint, measured value, 
 *   and elapsed time interval.
 *
 * Definitions:
 * - kp: Proportional gain.
 * - ki: Integral gain.
 * - kd: Derivative gain.
 * - setpoint: Desired target value.
 * - measured_value: Current measured value.
 * - time_interval: Time interval over which the control is applied.
 * - exponent_val: The exponential decay factor for weighting previous errors.
 * 
 * @author : Dmytro Humennyi
 * @date : Jul 15 2024
 * @version :s V0.03
 */

#include <stdlib.h>
#include "angular_control.h"
#include "transfer_function.h"
#include "angular_datatypes.h"


// Initialize global variables
static fdata integral = 0.0;
static fdata previous_error = 0.0;
fdata exponent_val = 0.5;

void create_pid(pid_conf **pid)
{
    if (*pid == NULL)
    {
        *pid = malloc(sizeof(pid_conf));
        if (*pid != NULL)
        {
            (*pid)->kP = 1.00;
            (*pid)->kI = 0.10;
            (*pid)->kD = 0.05;
            (*pid)->integral_min = -3.1415926;
            (*pid)->integral_max = 3.1415926;
            (*pid)->weight1 = 1.0;
            (*pid)->weight2 = 0.5;
            (*pid)->weight3 = 0.25;
        }
    }
}

void delete_pid(pid_conf *pid)
{
    free(pid);
}

// PID control function with explicit dt
fdata pid_control(pid_conf *pid, fdata setpoint, fdata measured_value, fdata timesample, Node *errors, fdata exponential_decay_factor)
{
    fdata error = setpoint - measured_value;
    // Proportional term
    pid->proportional = pid->kP * error;

    // Integral term
    pid->integral += pid->kI * error * timesample;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    // Derivative term
    pid->derivative = pid->kD * (error - errors->error) / timesample;

    // Calculate weighted error with exponential decay
    fdata weighted_error = 0.0;
    fdata weight = 1.0;  // Initial weight for the current error
    Node *current = errors;
    while (current != NULL) {
        weighted_error += weight * current->error;
        weight *= exponent_val;  // Apply exponential decay
        current = current->next;
    }
    fdata combined_error = error + weighted_error;

    // Proportional term considering all previous errors
    pid->proportional = pid->kP * combined_error;

    // Update previous errors
    Node *prev_error = NULL;
    current = errors;
    while (current != NULL) {
        if (prev_error == NULL) {
            prev_error = current;
            current->error = error;
        } else {
            prev_error->error = current->error;
            prev_error = current;
        }
        current = current->next;
    }

    // PID output
    return pid->proportional + pid->integral + pid->derivative;
}

control_output control_angular_motion_ex(fdata offset_x, fdata offset_y, fdata offset_z, fdata timesample, unsigned int resolution_width, unsigned int resolution_height, unsigned int horizontal_fov, unsigned int vertical_fov)
{  
    videocam_params vc_params = {resolution_width, resolution_height, horizontal_fov, vertical_fov};
    target_offset target_offset = {offset_x, offset_y};

    target_polar_position setpoint = angular_offsets(vc_params, target_offset);

    // generates the PID controllers for oX, oY, oZ separately.
    // calculates the plane angular velocities
    // and angles on the planes control plates
    static pid_conf *pid_x = NULL;
    static pid_conf *pid_y = NULL;
    static pid_conf *pid_z = NULL;
    create_pid(&pid_x);
    create_pid(&pid_y);
    create_pid(&pid_z);

    // the PID error Node init
    Node node_error_x = {0.0, NULL};
    Node node_error_y = {0.0, NULL};
    Node node_error_z = {0.0, NULL};

    control_output control_angular_motion_out;
    control_angular_motion_out.control_output_x = pid_control(pid_x, setpoint.center_to_target_oX, offset_x, timesample, &node_error_x, exponent_val);
    control_angular_motion_out.control_output_y = pid_control(pid_y, setpoint.center_to_target_oY, offset_y, timesample, &node_error_y, exponent_val);
    control_angular_motion_out.control_output_z = pid_control(pid_z, 0, offset_z, timesample, &node_error_z, exponent_val); // Assuming setpoint for z-axis (roll) is 0

    return control_angular_motion_out;
}
