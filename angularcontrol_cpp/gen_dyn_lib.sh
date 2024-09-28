#!/bin/bash

#clear all Obj.
rm -f *.o

#precompile
gcc -c -fPIC main.c
gcc -c -fPIC angular_control.c
gcc -c -fPIC angular_calculator.c
gcc -c -fPIC transfer_function.c

#so lib gen.
gcc -shared -o ../resources/angularcs.so *.o