#ifndef ANGULAR_CALCULATOR_H
#define ANGULAR_CALCULATOR_H

#define TEST_CAMERA_WIDTH_RESOLUTION 640
#define TEST_CAMERA_HEIGHT_RESOLUTION 480
#define TEST_CAMERA_HOR_FOV 90
#define TEST_CAMERA_VER_FOV 90

// Structure to store the angular positions of the target
typedef struct {
    float center_to_target_oX; // (pitch)
    float center_to_target_oY; // (yaw)
} target_polar_position;

// Structure for camera parameters
typedef struct {
    unsigned int resolution_width;
    unsigned int resolution_height;
    unsigned int horizontal_fov;
    unsigned int vertical_fov;
} videocam_params;

// Structure to store the target's offset from the center of the frame
#include "angular_datatypes.h"
typedef struct {
    fdata offset_x; // (pitch)
    fdata offset_y; // (yaw)
} target_offset;

// Function to calculate angular offsets
target_polar_position angular_offsets(videocam_params vc_params, target_offset target_offset);

#endif // ANGULAR_CALCULATOR_H
