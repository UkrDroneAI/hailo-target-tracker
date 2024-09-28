# Angular Motion Control System

This repository contains a comprehensive implementation of an angular motion control system using PID control. The system is designed to calculate and control angular offsets for a Control Object (target), ensuring stable and precise movement.

## Overview

The system consists of three main components:
- Angular Calculator: Functions to calculate angular offsets.
- Angular Control: Functions to control the angular motion using PID control.
- Transfer fucnction: Acts as a transfer function between inputs and outputs of the regulators.
- Main Program: A demonstration of the control system in action.

## Components

1. **Angular Calculator**
   - Calculates angular offsets.
   - Files: `angular_calculator.h`, `angular_calculator.c`

2. **Angular Control**
   - Controls angular motion using PID control.
   - Files: `angular_control.h`, `angular_control.c`

3. **Transfer Function**
   - Acts as a transfer function between inputs and outputs of the regulators.
   - Files: `transfer_function.h`, `transfer_function.c`

4. **Main Program**
   - Demonstrates the control system.
   - File: `main.c`

## Components

1. **Angular Calculator**
   - Calculates angular offsets.
   - Files: `angular_calculator.h`, `angular_calculator.c`

2. **Angular Control**
   - Controls angular motion using PID control.
   - Files: `angular_control.h`, `angular_control.c`

3. **Transfer Function**
   - Acts as a transfer function between inputs and outputs of the regulators.
   - Files: `transfer_function.h`, `transfer_function.c`

4. **Main Program**
   - Demonstrates the control system.
   - File: `main.c`

## How to Compile and Generate the Library

## Scripts

1. `compyle_and_run.sh` - Compile and run the executable.
2. `gen_dyn_lib.sh` - Generate the dynamic library.


## OR mannual


### Step 1: Remove existing object files
```sh
rm -f *.o
```

### Step 2: Compile the source files with -fPIC
```sh
gcc -c -fPIC main.c
gcc -c -fPIC angular_control.c
gcc -c -fPIC angular_calculator.c
gcc -c -fPIC transfer_function.c
```

### Step 3: Create the shared library
```sh
gcc -shared -o angularcs.so *.o
```

### How to Make Executable Program
```sh
gcc main.c angular_calculator.c angular_control.c transfer_funtion.c -o angularcs; ./angularcs
```
