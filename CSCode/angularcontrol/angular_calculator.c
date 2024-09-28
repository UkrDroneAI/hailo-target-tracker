/*
 * @file : angular_calculator.c
 *
 * @details:
 * Angular Calculator Module
 *
 * This module provides functions for calculating angular offsets.
 *
 * Functions:
 * - angular_offsets: Calculates the angular offsets of the target from the center of the frame.
 *
 * Definitions:
 * - vc_params: Video camera parameters including field of view and resolution.
 * - target_offset: The target's offset in pixels from the center of the frame.
 * - target_polar_position: Structure to hold the calculated angular offsets.
 * 
 * @author : Dmytro Humennyi
 * @date : Jul 15 2024
 * @version : V0.02
 */
#include "angular_calculator.h"

/* Function to calculate angular offsets of the target from the center of the frame */
target_polar_position angular_offsets(videocam_params vc_params, target_offset target_offset) {
    target_polar_position target_position;

    // Calculate horizontal angular offset
    target_position.center_to_target_oX = target_offset.offset_x * ((float)vc_params.horizontal_fov / vc_params.resolution_width);

    // Calculate vertical angular offset
    target_position.center_to_target_oY = target_offset.offset_y * ((float)vc_params.vertical_fov / vc_params.resolution_height);

    return target_position; 
}
