#!/bin/bash

# cppcheck --addon=misra.json --suppressions-list=suppressions.txt BrakeModule_STM32F1_sw03_misra.ino
# cppcheck --addon=misra.json --suppressions-list=suppressions.txt $1
# cppcheck --addon=./misra/misra.json --suppressions-list=./misra/suppressions.txt $1
FILE="$(find *.ino)"

# This is not used, maybe in future
cppcheck_parameters=( --inline-suppr
                      --language=c++
                      --addon="$script_folder/misra.json"
                      --suppressions-list="$script_folder/suppressions.txt"
                      --platform=avr8
                      --cppcheck-build-dir="$out_folder"
                      -j "$num_cores"
                      -DCORE_AVR=1
                      -D__AVR_ATmega2560__
                      # This is defined in the AVR headers, which aren't included.
                      # cppcheck will not do type checking on unknown types.
                      # It's used a lot and it's unsigned, which can trigger a lot
                      # of type mismatch violations.
                      -Dbyte=uint8_t
                      # All violations from included libraries (*src* folders) are ignored
                      --suppress="*:*src*"
                      # No libdivide - analysis takes too long
                      -UUSE_LIBDIVIDE
                      # Don't parse the /src folder
                      -i "$source_folder/src"
                      "$source_folder/**.ino"
                      "$source_folder/**.cpp")


#printf "First make CPPCHECK for $FILE file in current folder\n"
#sleep 5

# cppcheck echo find *.ino
#cppcheck $FILE 2> misra_violations.txt

#printf "If above was performed without warnings, CPPCHECK was passed\n"

#sleep 5

#printf "\nThen make MISRA-c2012 check for $FILE file in current folder\n"
printf "Make CPPCHECK and MISRA-c2012 check for $FILE file in current folder, and save the output into misra_violations.txt\n\n"
sleep 5

cppcheck --addon=../Safety/misra/misra.json --suppressions-list=../Safety/misra/suppressions.txt $FILE |& tee misra_violations.txt

#cat misra_violations.txt
