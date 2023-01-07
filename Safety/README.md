## Safety aspect

In critical systems, it is important to adhere to the principle of rigorous code testing. The BrakeModule code has undergone this process through the use of static analysis tools such as Cppcheck and the Misra addon.

Cppcheck is a static analysis tool for C/C++ code that is designed to detect bugs and undefined behavior. It has been used to check the BrakeModule code for any issues.

The Misra addon is a layer on top of Cppcheck that performs rule checking according to the MISRA-C:2012 standard. The BrakeModule code has passed the Cppcheck test and is currently being checked for MISRA compliance. MISRA violations can be seen on misra/misra_violations_output.txt. For making those tests there is shell script code_tests.sh in misra folder.

During compilation, the ```-warnings=all``` flag is also set to highlight any potential problems.

### General safety filosophy
The BrakeModule aims to minimize the impact on the original ABS/DSC system's functionality and safety features.

TODO: Conduct a systematic analysis of logic or component failures and their effect on the BrakeModule and the rest of the system.

### Software design
Some of the funtional safety stuff that has been implemented in software:
- STM32F1 internal temperature monitoring and and OP disengaged if temperature above certain threshold.
- POWER MOSFET temperature monitoring and and OP disengaged if temperature above certain threshold.
- Check for freezed ADC values.
- IDWG for restarting the program if chip freezes.
- CAN bus failure detection.
- Check that OP related CAN BUS msg is present in the BUS.
- Lock pin configuration after initialization.

### Hardware design
Here text what safety has tried to be implemented in hardware

It is important to note that these safety measures do not guarantee the safety of the BrakeModule. Further analysis may be necessary to fully assess the system's safety.
