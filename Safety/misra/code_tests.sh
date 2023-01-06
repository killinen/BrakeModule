#!/bin/bash

# cppcheck --addon=misra.json --suppressions-list=suppressions.txt BrakeModule_STM32F1_sw03_misra.ino
# cppcheck --addon=misra.json --suppressions-list=suppressions.txt $1
# cppcheck --addon=./misra/misra.json --suppressions-list=./misra/suppressions.txt $1

source_folder="$(find ../.. -maxdepth 1 -name 'BrakeModule*')"
cppcheck_out_file="misra_violations_output.txt"
FILE="$(find ${source_folder}/*.ino)"

# This is not used, maybe in future, this is here to give inspiration :)
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



#printf "\nThen make MISRA-c2012 check for $FILE file in current folder\n"
printf "Make CPPCHECK and MISRA-c2012 check for $FILE and save the output into misra_violations_output.txt\n\n"
sleep 5

cppcheck --addon=misra.json --suppressions-list=suppressions.txt $FILE |& tee $cppcheck_out_file

# Count lines for Mandatory or Required rules
error_count=`grep -i "Mandatory - \|Required - " < "$cppcheck_out_file" | wc -l`
printf "*** MISRA violations count was $error_count ***\n" |& tee -a $cppcheck_out_file
