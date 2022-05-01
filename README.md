# BrakeModule

This project has been created to solve the need of make an MY99 BMW 540i (E39) to brake with use of semiautonoumous software OPENPILOT.

---

## BACKROUND

This module has been developed with use of car which uses BOSCHs ABS 5.7 system. This system is used on several different cars so it most likely could be used with these car only software modification. The description of the system is what I have found on my system, yours may vary. I also make a lot of assumptions of how the system works on based on what I have seen but I don't have near close to 100% certainty of it.

List of used cars.

The SOFTWARE has been designed to read braking demand values from CAN bus that are sent from OPENPILOT. OPENPILOT is opensource semiautonomous system (software) that can possibly be retrofitted to your car to have LEVEL 2 autonomous system. The OPENPILOT needs to be able to control longnitudinal (acceleration and deceleratio) and lateral (steering) via CAN bus. 

Video of openpilot

This module could be used with other adaptive cruise control systems (ACC) with modifying the software.

---

## Working principle / HARDWARE

Working principle of the module is 1. to intercept the ABS unit from the pump and connect the ABS module pump line with resistor so that the ABS "thinks" that the pump is online (won't through an precharge pump error if lines are disconnected). 2. Connect 12V to the pump and use N-chan power mosfet to adjust to precharge pump yield (brake pressure). 3. One has to manipulate also the cars brake light (pedal) switch because if car detects increased brake pressure in the system with out info of brake pedal beeing pressed, it throws an brake pressure sensor defekt. Brake light switch has 4 wires: 12V, ground, signal LOW, signal 2 HIGH. Signal LOW is grounded and signal HIGH is floating when pedal not pressed. When pedal is pressed, signal LOW is floating and signal HIGH is connected to 12V.

Picture of BOSCH system.

Brakemodule main functionalities are. 3 relays for switching precharge pump inline with ABS system or BrakeModule. 1 power MOSFET to control the precharge pump. Temperature measurent to read the power MOSFET temperature. FAN + mosfet to cool down the power MOSFET. Voltage divider is used to measure ABS line voltage.
In this development stage the main components of the hardware is LGT8F328P LQFP32 MiniEVB (can work w Nano also), MCP2515 CAN module, 4 relay module, Infineon FR407 power MOSFET, MCP1412 gate driver IC, 2 N7000 N-chan MOSFET, NPN3904 transistor, BS250 P-chan MOSFET, DS18B20 temperature sensor.

Picture of pump control.

Picture of PCB

The PCB layout is much improved on the next generation and should not give too much thought. Example there are not use of 2 power MOSFETs (TO220 and TO252) these slots are for testing purposes.

---

## SOFTWARE

The software has been developed in arduino IDE to LGT8F328P board with Arduino Nano compability in mind. The next generation of the module most likely uses Blue Pill development board (STM32F103).

## Next generation

The next generation of the board is planned to have an Blue Pill development board accompanied with MCP2551 CAN module. 
- Integrate the relays (and better use better ones) into the PCB. 
- Integrate the brakeswitch lines into the PCB, get the low current demand of the board from brake light switch lines.
- "Read" the brake pedal switch true state. 
- Change the temperature sensor from DS18B20 to LM36. 
- Integrate wire connections into PCB.

Picture of PCB

---
