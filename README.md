# BrakeModule
---

This project has been created to solve the need of make an MY99 BMW 540i (E39) to brake with use of semiautonoumous driving software OPENPILOT.

---

### Disclaimer

The software and hardware of this project has been done by fellow that have no knowledge of anykind on automotive, software or hardware developing stuff nor electrical engineering. So if some ever implements this, keep in mind that there is some change that the BrakeModule will behave unwantedly or will damage your cars electrical system! If you have better knowledge than I, please contribute on developing this.

---

## BACKROUND

This module has been developed with use of car which uses BOSCH ABS 5.7 Dynamic Stability Control system 3 (DSCIII). This system is used on several different cars so it could possibly be used with these car only software modification. The description of the system is what I have found on my system, yours may vary. I also make a lot of assumptions of how the system works on based on what I have seen but I don't have near close to 100% certainty of it. There might be a change this could be used on BOSCH 5.3 system if DSCIII implemented.

Some list of of cars that use BOSCH 5.7 system found on the internet.

<p align="left">
  <img src="Pics/Bosch57cars.PNG?raw=true">
</p>

I'm not sure if these are compatible cars, eg. is BOSCH 5.7 system == charge pump system eg. DSCIII (or similar), that needs to be in place to brake the car?

The SOFTWARE has been designed to read braking demand values from CAN bus that are sent from OPENPILOT. OPENPILOT (OP) is opensource semiautonomous system (software) that can possibly be retrofitted to your car to have LEVEL 2 autonomous system. The OPENPILOT needs to be able to control longnitudinal (acceleration and deceleratio) and lateral (steering) via CAN bus. https://github.com/commaai/openpilot

Video introducing OPENPILOT.

[![](https://img.youtube.com/vi/NmBfgOanCyk/0.jpg)](https://youtu.be/NmBfgOanCyk)

My implementation (fork) of OPENPILOT now uses only vision based ACC with stop and go capability. Lateral control is WIP. Top left of the screen shows the set speed 87 (km/h) and top center the current speed reduced by the vision system that detects the lead cars distance, speed and accel/decelration.


https://user-images.githubusercontent.com/37126045/204830637-9adcd51b-5c41-4f0d-8b8d-8c93d2226a8f.mp4



This module could be used with other adaptive cruise control systems (ACC) with modifying the software.

---

## Working principle
### Abstract of BOSCH 5.7/DSCIII system
The system can be roughly devided into few subsystems: 
- control module (ECU) that is responsible all of the logic and electrical stuff
- valves that control the brakefluid flow
- ABS relief pump
- charge pump (also known as pre-charge pump)
- sensors that monitor the system and the trajectory of the car. 

If ABS or DSC events are detected by the control module by reading the sensor values determinating with that the car state, the module can will (depending of situation):
- reduce motor torque requesting it via sending CAN messages to the DME (motor ECU)
- relief pressure of individual brakes by controlling the valves and running the ABS relief pump (ABS event).
- apply pressure to individual brakes by controlling the valves and running the charge pump (DSC event).

Picture of BOSCH system with DSCIII.
<p align="left">
  <img src="Pics/DSCIII.PNG?raw=true">
</p>

As the charge pump is responsible of the increase of the pressure in the brake circuit (10-15 bar), if one can control it, can also control cars deceleration (braking). The charge pump is controlled by 2 N-channel MOSFETs inside the control module in halfbrige configuration.

Pic of 2 FETs inside the module, right one is damaged. These are not the regular type black thingies (TO220) that you normally see as MOSFETs.
<p align="left">
  <img src="Pics/PumpFETs.PNG?raw=true">
</p>

Handwritten schematic of the driving circuit of the charge pump.
<p align="left">
  <img src="Pics/PumpSch.PNG?raw=true">
</p>

If someone want's to deep dive closer on Bosch DSC 5.7 ABS Module Diagnosis and Repair read this great post: https://www.bimmerfest.com/threads/bosch-dsc-5-7-abs-module-diagnosis-and-repair.822139/#post-8854110 (the pics are stolen from it).

Further knowledge on how BOSCH 5.7/DSCIII works, look at https://github.com/killinen/BrakeModule/blob/main/dsc_system.pdf in the repo.

Control module/DSC hydraulic unit shown in M54 engine bay.
<p align="left">
  <img src="Pics/ABSunit.PNG?raw=true">
</p>

Charge pump shown in M62TU engine bay.
<p align="left">
  <img src="Pics/Pump.PNG?raw=true">
</p>

## The name of the Man-in-the-middle is BrakeModule
The basic idea to take control of the ABS/DSC unit ability to brake is to use man-in-the-middle tactic (https://en.wikipedia.org/wiki/Man-in-the-middle_attack). This is implemented by taking the charge pump control from ABS module so that it wont confuse any diagnostic systems by: 
1. Disconnecting the BOSCH control module from the charge pump 
2. Reconnecting the charge pump wires from module with resistor so that the control module "thinks" that the pump is connected. If wires are disconnected, module throws an charge pump error, because it will detect open circuit via the feedback lines (f/b) shown in handwritten schematic. 
3. Connecting 12V to the pump and use N-channel power MOSFET to adjust to charge pump load (brake pressure). 
4. Manipulating the cars brake light (pedal) switch detection wires, because if car detects increased brake pressure in the system without detection of brake pedal beeing pressed, it throws an brake pressure sensor defekt error. 

Main principle of driving charge pump and brakelight switch with BrakeModule.
<p align="center">
  <img src="Pics/BMMain.png?raw=true">
</p>

Brake light switch has 4 wires: 12V, ground, signal LOW, signal 2 HIGH. Signal LOW is grounded and signal HIGH is floating when pedal not pressed. When pedal is pressed, signal LOW is floating and signal HIGH is connected to 12V. Those lines are in BMW lingo S_BLS (brake-light switch) S_BLTS (brake-light test switch). 
<p align="center">
  <img src="Pics/Brake light switch.JPG?raw=true">
</p>

In E39 The DME control unit evaluates the signals for the purpose of registering brake operation. The brake-light switch connects to earth (B-), the brake-light test switch connects to B+. And uses this truth table for operation:
<p align="center">
  <img src="Pics/BLS_logik.PNG?raw=true">
</p>

---

## BrakeModule HARDWARE

Brakemodule main functionalities are 3 relays for switching charge pump inline with DSC system or BrakeModule. 1 power MOSFET to control the charge pump. Temperature measurement to read the power MOSFET temperature. FAN + mosfet to cool down the power MOSFET. CAN module is used for communication. Voltage divider is used to measure DSC+ line voltage and cars brake light switch.

The board size is around 110x82mm and it contains.
- Blue Pill development board containing STM32F103 MCU.
- MCP2551 high-speed CAN protocol controller bus interface module for commucation.
- 3 SMIH-12VDC-SL-C relays for switching the charge pump in/offline. 
- TC1413N DIP8 mosfet driver.
- POWERPAK SO-8L MOSFET.
- 12V to 5V buck converter module.
- BS250P for driving the brakelight switch high side. 
- TMP36 temperature sensor for MOSFET. 
- SOT23-NPN transistors for common switching purposes, eg. possible fan.
- TO92-NPN transistors for driving the SMIH-12VDC-SL-C relays
- Have USB to UART (FTDI) connector for updating the software and debugging.
- Voltage dividers for sensing brake pedal high side (S_BLTS) and DSC charge pump driver state (is voltage fed to CP via DSC).


Picture of PCB of BrakeModule v0.3.2.
<p align="center">
  <img src="Pics/PCB_032.PNG?raw=true">
</p>


Picture of BrakeModule inside 3D-printed case.
<p align="center">
  <img src="Pics/20220817_162326.jpg?raw=true">
</p>

---

## Next iteration

If I have the insipiration to make of BM v0.3.3, there could be: 
- Fixes for HW bugs (funny I found few). 
- Reduce part count (simplify).
- Maybe add (working) brake light switch low side sensing (redundancy).
- Maybe 4 layer PCB desing (could lower the power tracing resistance?)
- Maybe use of different kind of power MOSFET (is the footprint right).


## Possible future development

Possibly ditch the pump side relays and make same kinda MOSFET configuration that in the DSC module. Then sense the DSC wires and drive the pump accordingly. Below is picture that shows voltage in millivolts (DSC_VOLTAGE) measured from DSC+ wire. There seems to be somekinda bootstrapping cycle and somehow voltage is increased when the pump is running driven by the BrakeModule. The pump running can be observed from the PWM value (2047 = 100% dutycycle). Brakepressure (bar) is shown for reference. The pressure reading wo pump been driven is me pressing the brake pedal :)

<p align="center">
  <img src="Pics/DSC_VOLT.png?raw=true">
</p>

The unfortunate thing in these voltage spikes are IMO that if you would like to eliminate the change of charge pump not running when DSC module is demanding it but BrakeModule controlling the pump it will result of somewhat of an lag if you are waiting to see that is this voltage spike bootstrapping cycle or an real pump control demand. Will this result an real world meningful lag, I can't really say.

In older HW v0.2 I have used (tested) relay or N-channel MOSFET for controlling the LOW side BLS signal line (S_BLS). I somehow prefer the use of NC-relay, but maybe tranfer to some other type of solution in the future. Dual channel MOSFET IC maybe or something.

---

## BrakeModule SOFTWARE

The software has been developed in arduino IDE to LGT8F328P board with Arduino Nano compability in mind. The next generation of the module most likely uses Blue Pill development board (STM32F103). Normally charge pump is controlled via BOSCH control module (relays are on NC mode) but when decelaration demand from OP is detected in CAN msg 0x343 (BRK_CMD), it'll disconnect the module from the pump and start controlling the pump with 15 kHz PWM signal of the power MOSFET (relays state are switched OFF from NC mode). Also brake light swithes HIGH (S_BLTS) and LOW (S_BLS) signal lines are driven so that the car detects brake pedal pressed event. When BRK_CMD demand is no longer detected, first 12V line and ground (PWR MOSFET) will be disconnected from the pump and after 600 ms DSC control module is switch back inline with the pump. Also brakelight switch is turned OFF. This delay is because if the transition from pump activated with BrakeModule back to DSC module is too fast, DSC modue will give error code. I think this is caused of pump still rotating (generating voltage to pump wires) and you will connect the pump to DSC module, modules feedback lines detects voltage at the pump when it shouldn't and gives an error.

See below if it true ATM (not been implemented in this HW version).
TMP36 measures power MOSFET temperature every 10 seconds and small fan will turn on if over 45 degrees is detected. Also if temperature exceeds 80 degrees, brake module will disable OPENPILOT and wont engage until temperature is below that (this might is not nessaccary at least haven't been for me).

Speed value (car_speed) is read on message 0x153 and send to OPENPILOT as ACC set speed when OP is engaged (set_speed). Brake pedal state is read on message 0x329 (BRK_ST) and sent to OP when there is no braking demand and pedal press is detected to disengage OP (BRK_ST_OP). BMW cruise control steering wheel button presses (BTN_CMD) are detected on same message. If BTN_CMD contains RESUME button press, it will engage or disengage the OP ACC depenfing on state. BTN_CMD includes steering wheel + and - button presses and adjusts the set speed of the OP ACC accordingly when those buttons are pressed. Cruise control state (OCC) is detected on 0x545 and if it is on OP wont engage to prevent original CC and OP to control longnitudinal simultatoneusly.

Filtering of CAN messages are used to give better change to not to lose wanted messages. After filtering measured capture rate of CAN messages was 99.97 % in 1 minute test with Arduino Nano, with LGT8F328P little bit better. Average loop time per 1 million loopcycles was ~31 uS w Nano (measured with millis() function).

If DEBUG is #defined you can control the board via serial (look at the readSerial() function) + some debugging messages are shown.

BrakeModule is used to emulate TOYOTA corollas cruise controller because this is the car which is used on my OPENPILOT fork. This implementation is shown as sent data sent in 0x1D2 and 0x1D3 CAN messages which are originally used by TOYOTA cruise controller. The use of TOYOTA in OP is from legacy reasons because the first guy that used OP on older cars implement it on TOYOTA Celica and my code is just revision of that.

For discussion of "old" cars impelementation of OPENPILOT join discord: discord server link here.

---

## OT of CAN bus messages
if someone want to see some CAN messages in E39 CAN bus https://github.com/killinen/opendbc/blob/master/BMW_E39.dbc (this is not perfect, would be cool if someone else would like to look into it). This is in OPENDBC format. If don't know what DBC is here's couple good reads https://github.com/stefanhoelzl/CANpy/blob/master/docs/DBC_Specification.md http://socialledge.com/sjsu/index.php/DBC_Format. Great insipiration for reverse engineering the CAN messages was this thread relating to E46 CAN bus messages: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project. Also for some other chassis CAN msgs see dzid's issue https://github.com/killinen/BrakeModule/issues/1.

---

### Problems that I know of
What I have understand there are standards for automotive hardware and software design and this does project does not follow any of those.

The main worry point that I have is that if an stability control event should happen on the same time that BrakeModule is controlling the charge pump there is quite high change that the stability control system wont funtion as planned. This is an issue that I want to address after getting the next generation hardware done.

If you damage the ABS control unit it is quite hard to repair and if bought new also quite pricey. I give no promises that using the BrakeModule won't brake anything. I think even that it is somewhat likely that it could happen at least with this HARDWARE/SOFTWARE. Knock on wood, I haven't broke my unit even though it has had quite a bit of rough love. Thanks to German engineer.

I don't know what is the max capability of the charge pump eg. if you run it too long can it overheat or something. The software does not restrict this at all.

If brake pedal is pressed hard and BrakeModule is controlling the pump and switched back to "normal mode", DSC will give an error.

Good design would prolly be to install the module on a professional case so components wouldn't be exposed with integrated heat dispersion.

---

### Someone might think

That wouldn't it be best if you could control the DSCIII control module via CAN to use the charge pump. Yes it prolly would, but I don't have knowledge how to do it. I think this could be feaseble because this same unit is used with ACC systems and the only way I can think of is to apply the brakes is to use the charge pump.

The benefits in my mind of the Brake Module to latter is to have full control of braking (my undestanding is that OEM system won't brake below certain speed in ACC mode). Scalability is also benefit because you probably wont need to reverse engineer all the possible messages/programs that are implemented on different car brands and models. And lastly this is more fun :)

---
