# BrakeModule

This project has been created to solve the need of make an MY99 BMW 540i (E39) to brake with use of semiautonoumous driving software OPENPILOT.

---

### Disclaimer

The software and hardware of this project has been done by fellow that have no knowledge of anykind on automotive, software or hardware developing stuff nor electrical engineering. So if you use it as is keep in mind that there is some change that the BrakeModule will behave unwantetly or will damage your cars electrical system!
If you have better knowledge than I, please contribute on developing this.

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

This module could be used with other adaptive cruise control systems (ACC) with modifying the software.

---

## Working principle
Superabstaract of BOSCH 5.7/DSCIII system. In my mind the system can be roughly derived into few subsystems: control module (ECU) that is responsible all of the electrical stuff, valves that control the brakefluid flow, ABS relief pump, charge pump (also known as pre-charge pump) and sensors that monitor the system and the trajectory of the car. In ABS/DSC situation control module can reduce motors torque via CAN messages, relief brake pressure on individual brakes with valves and ABS relief pump or apply pressure to certain brakes by controlling the valves and running the charge pump.

Picture of BOSCH system with DSCIII.
<p align="left">
  <img src="Pics/DSCIII.PNG?raw=true">
</p>

As the charge pump can increase the pressure in the brake circuit (10-15 bar), if one can control it, can also control cars deceleration (braking). The charge pump is controlled normally by 2 N-channel MOSFETs inside the control module in halfbrige configuration.

Pic of 2 FETs inside the module, right one is damaged. These are not the regular type black thingies that you normally see as MOSFETs.
<p align="left">
  <img src="Pics/PumpFETs.PNG?raw=true">
</p>

Handwritten schematic of the driving circuit of the charge pump.
<p align="left">
  <img src="Pics/PumpSch.PNG?raw=true">
</p>

If someone want's to deep dive closer on Bosch DSC 5.7 ABS Module Diagnosis and Repair read this great post: https://www.bimmerfest.com/threads/bosch-dsc-5-7-abs-module-diagnosis-and-repair.822139/#post-8854110 (the pics are stolen from it).

Further knowledge on how BOSCH 5.7/DSCIII works, look at dsc_system.pdf on the repo.

Main thesis of how the BrakeModule works is 1. to disconnect the BOSCH control module from the charge pump and connect the charge pump wires from module with resistor so that the control module "thinks" that the pump is connected. If wires are disconnected, module throws an charge pump error, because it will detect open circuit with the feedback lines (f/b) shown in handwritten schematic. 2. Connect 12V to the pump and use N-channel power MOSFET to adjust to charge pump yield (brake pressure). 3. One has to manipulate also the cars brake light (pedal) switch because if car detects increased brake pressure in the system without detection of brake pedal beeing pressed, it throws an brake pressure sensor defekt error. 

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

Control module/DSC hydraulic unit shown in M54 engine bay.
<p align="left">
  <img src="Pics/ABSunit.PNG?raw=true">
</p>

Charge pump shown in M62TU engine bay.
<p align="left">
  <img src="Pics/Pump.PNG?raw=true">
</p>

---

## BrakeModule HARDWARE

Brakemodule main functionalities are 3 relays for switching charge pump inline with DSC system or BrakeModule. 1 power MOSFET to control the charge pump. Temperature measurement to read the power MOSFET temperature. FAN + mosfet to cool down the power MOSFET. CAN module is used for communication. Voltage divider is used to measure DSC+ line voltage.

In this development stage the main components of the hardware is LGT8F328P LQFP32 MiniEVB (can work w Nano also), MCP2515 CAN module, 4 relay module, Infineon IRF40R207 N-chan power MOSFET, TC1413N gate driver IC, 2 2N7000 N-chan MOSFET, 2N3904 NPN transistor, BS250 P-chan MOSFET, DS18B20 temperature sensor.

Picture of PCB of BrakeModule v0.2.
<p align="center">
  <img src="Pics/BM0.2.PNG?raw=true">
</p>

BrakeModule PCB is mounted on 3D-printed case with 4 relay module, and attachments for the fan.
Picture of BrakeModule.
<p align="center">
  <img src="Pics/BM.jpg?raw=true">
</p>

The PCB layout is much improved on the next generation and should not give too much thought. Example there are not use of 2 power MOSFETs (TO220 and TO252) these slots are for testing purposes.

---

## Next generation (in development)

The next generation of the board is planned to have an Blue Pill development board accompanied with MCP2551 CAN module. 
- Integrate the relays (and use better ones) into the PCB. 
- Integrate the brakeswitch lines into the PCB, get the low current demand of the board from brake light switch lines.
- Add 12V to 5V buck converter module.
- "Read" the brake pedal switch true state. 
- Change the temperature sensor from DS18B20 to TMP36. 
- Integrate pump wire connections into PCB.
- Have USB to UART (FTDI) connector for updating the software and debugging.
- Maybe use of different kind of power MOSFET.

<p align="center">
  <img src="Pics/BM03.PNG?raw=true">
</p>

## Possible future development

Possibly ditch the pump side relays and make same kinda MOSFET configuration that in the DSC module. Then sense the DSC wires and drive the pump accordingly. Below is picture that shows voltage in millivolts (DSC_VOLTAGE) measured from DSC+ wire. There seems to be somekinda bootstrapping cycle and somehow voltage is increased when the pump is running driven by the BrakeModule. The pump running can be observed from the PWM value (2047 = 100% dutycycle). Brakepressure (bar) is shown only reference. The pressure reading wo pump been driven is me pressing the brake pedal :)

<p align="center">
  <img src="Pics/DSC_VOLT.png?raw=true">
</p>

The unfortunate thing in these voltage spikes are IMO that if you would like to eliminate the change of charge pump not running when DSC module is demanding it but BrakeModule controlling the pump it will result of somewhat of an lag if you are waiting to see that is this voltage spike bootstrapping cycle or an real pump control demand. Will this result an real world meningful lag, I can't really say.

In HW v0.2 I have used (tested) relay or N-channel MOSFET for controlling the LOW side BLS signal line (S_BLS). I somehow prefer the use of NC-relay, but maybe tranfer to some other type of solution in the future. Dual channel MOSFET IC maybe or something. The impelementation of both solution can be seen in SW as #define values B_MOSFET or B_RELAY.

---

## SOFTWARE

The software has been developed in arduino IDE to LGT8F328P board with Arduino Nano compability in mind. The next generation of the module most likely uses Blue Pill development board (STM32F103). Normally charge pump is controlled via BOSCH control module (relays are on NC mode) but when decelaration demand from OP is detected in CAN msg 0x343 (BRK_CMD), it'll disconnects the module from the pump and start controlling the pump with ~2 kHz PWM signal of the power MOSFET (relays state are switched OFF from NC mode). Also brake light swithes HIGH (S_BLTS) and LOW (S_BLS) signal lines are driven so that the car detects brake pedal pressed event. When BRK_CMD demand is no longer detected, first 12V line and ground (PWR MOSFET) will be disconnected from the pump and after 600 ms DSC control module is switch back inline with the pump. Also brakelight switch is turned OFF. This delay is because if the transition from pump activated with BrakeModule back to DSC module is too fast, DSC modue will give error code. I think this is caused of pump still rotating (generating voltage to pump wires) and you will connect the pump to DSC module, modules feedback lines detects voltage at the pump when it shouldn't and gives an error.

DS18B20 measures power MOSFET temperature every 10 seconds and small fan will turn on if over 45 degrees is detected. Also if temperature exceeds 80 degrees, brake module will disable OPENPILOT and wont engage until temperature is below that.

Speed value (car_speed) is read on message 0x153 and send to OPENPILOT as ACC set speed when OP is engaged (set_speed). Brake pedal state is read on message 0x329 (BRK_ST) and sent to OP when there is no braking demand and pedal press is detected to disengage OP (BRK_ST_OP). BMW cruise control steering wheel button presses (BTN_CMD) are detected on same message. If BTN_CMD contains RESUME button press, it will engage or disengage the OP ACC depenfing on state. BTN_CMD includes steering wheel + and - button presses and adjusts the set speed of the OP ACC accordingly when those buttons are pressed. Cruise control state (OCC) is detected on 0x545 and if it is on OP wont engage to prevent original CC and OP to control longnitudinal simultatoneusly.

Filtering of CAN messages are used to give better change to not to lose wanted messages. After filtering measured capture rate of CAN messages was 99.97 % in 1 minute test with Arduino Nano, with LGT8F328P little bit better. Average loop time per 1 million loopcycles was ~31 uS w Nano (measured with millis() function).

If DEBUG is #defined you can control the board via serial (look at the readSerial() function) + some debugging messages are shown.

BrakeModule is used to emulate TOYOTA corollas cruise controller because this is the car which is used on my OPENPILOT fork. This implementation is shown as sent data sent in 0x1D2 and 0x1D3 CAN messages which are originally used by TOYOTA cruise controller. The use of TOYOTA in OP is from legacy reasons because the first guy that used OP on older cars implement it on TOYOTA Celica and my code is just revision of that.

The software ofcourse needs some rewrite when migrating to STM32 family of microcontrollers in next hardware generation.

For discussion of "old" cars impelementation of OPENPILOT join discord: discord server link here.

Note to self:
- Add DBC file that contains only E39 and TOYOTA corolla msgs.
- Add reference to how to read DBC's.
- Openpilot is disangaded rapidly if 80 degrees is detected, is there a better way?

---

### Problems that I know of
What I have understand there are standards for automotive hardware and software design and this does project does not follow any of those.

The main worry point that I have is that if an stability control event should happen on the same time that BrakeModule is controlling the charge pump there is quite high change that the stability control system wont funtion as planned. This is an issue that I want to address after getting the next generation hardware done.

If you damage the ABS control unit it is quite hard to repair and if bought new also quite pricey. I give no promises that using the BrakeModule won't brake anything. I think even that it is somewhat likely that it could happen at least with this HARDWARE/SOFTWARE. Knock on wood, I haven't broke my unit even though it has had quite a bit of rough love.

I don't know what is the max capability of the charge pump eg. if you run it too long can it overheat or something. The software does not restrict this at all.

If brake pedal is pressed hard and BrakeModule is controlling the pump and switched back to "normal mode", DSC will give an error.

Good design would prolly be to install the module on a professional case so components wouldn't be exposed with integrated heat dispersion.

---

### Someone might think

That wouldn't it be best if you could control the DSCIII control module via CAN to use the charge pump. Yes it prolly would, but I don't have knowledge how to do it. I think this could be feaseble because this same unit is used with ACC systems and the only way I can think of is to apply the brakes is to use the charge pump.

---
