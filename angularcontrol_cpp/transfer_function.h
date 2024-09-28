#ifndef TRANSFER_FUNCTION_H
#define TRANSFER_FUNCTION_H

// The coefficient that amplifies the input signal. It represents the effectiveness of the control surface.
// It determines how strongly the input signal (angular input) affects the output signal (control surface positions).
#define GAIN_FACTOR_X 1
#define GAIN_FACTOR_Y 1
#define GAIN_FACTOR_Z 0.2

// A parameter representing the system's responsiveness. Higher values indicate slower system response.
// It determines how quickly the system responds to changes in the input signal. A higher time constant indicates a slower response.
#define TIME_CONSTANT_X 0.3
#define TIME_CONSTANT_Y 0.8
#define TIME_CONSTANT_Z 0.2

// functions to compute the control surface positions for an airplane based on the angular positions obtained from PID controllers
#include "angular_datatypes.h"
fdata transfer_function_x (fdata target_angular_velocity_oX, fdata timesample);
fdata transfer_function_y (fdata target_angular_velocity_oY, fdata timesample);
fdata transfer_function_z (fdata target_angular_velocity_oZ, fdata timesample);

#endif // TRANSFER_FUNCTION_H