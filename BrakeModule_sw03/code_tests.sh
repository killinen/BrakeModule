#!/bin/bash

# cppcheck --addon=misra.json --suppressions-list=suppressions.txt BrakeModule_STM32F1_sw03_misra.ino
# cppcheck --addon=misra.json --suppressions-list=suppressions.txt $1
# cppcheck --addon=./misra/misra.json --suppressions-list=./misra/suppressions.txt $1
FILE="$(find *.ino)"

printf "First make CPPCHECK for $FILE file in current folder\n"
sleep 5

# cppcheck echo find *.ino
cppcheck $FILE


printf "\nThen make MISRA-c2012 check for $FILE file in current folder\n"
sleep 5

cppcheck --addon=../Safety/misra/misra.json --suppressions-list=../Safety/misra/suppressions.txt $FILE
