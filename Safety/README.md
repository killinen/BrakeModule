In critical systems there should be rigorous code testing and this attempt to obey to this principle.
BrakeModule code has been checked with Cppcheck and misra addon. Cppcheck is a static analysis tool for C/C++ code. It provides unique code analysis to detect bugs and focuses on detecting undefined behaviour and dangerous coding constructs.
Misra addon is layer on top of Cppchck where MISRA-C2012 standard rule checking is done to the code. There atleast should not be any "required" rule errors.
Brakemodule code has passed the Cppcheck and MISRA complience is WIP. Inside Brakemodule software folder there is shell script that will conduct both tests.
