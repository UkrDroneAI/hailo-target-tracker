#!/bin/bash

# Create executable file
gcc angular_calculator.c angular_control.c transfer_function.c main.c -o angularcs

# Run the executable
./angularcs