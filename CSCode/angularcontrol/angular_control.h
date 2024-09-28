#ifndef ANGULAR_CONTROL_H
#define ANGULAR_CONTROL_H

#include "angular_calculator.h"

// Node - The PID error container
typedef struct Node {
    fdata error;
    struct Node *next;
} Node;

typedef struct angular_control
{
    fdata control_output_x;
    fdata control_output_y;
    fdata control_output_z;
} control_output;

typedef struct pid_config
{
    fdata kP;
    fdata kI;
    fdata kD;
    fdata proportional;
    fdata integral;
    fdata derivative;
    fdata integral_min;
    fdata integral_max;
    fdata err;
    fdata err1;
    fdata err2;
    fdata err3;
    fdata weight1;
    fdata weight2;
    fdata weight3;
} pid_conf;

void create_pid (pid_conf **pid);
void delete_pid (pid_conf *pid);
fdata pid_control(pid_conf *pid, fdata setpoint, fdata measured_value, fdata timesample, Node *errors, fdata exponential_decay_factor);
control_output control_angular_motion_ex(fdata offset_x, fdata offset_y, fdata offset_z, fdata timesample, unsigned int resolution_width, unsigned int resolution_height, unsigned int horizontal_fov, unsigned int vertical_fov);

#endif // ANGULAR_CONTROL_H
