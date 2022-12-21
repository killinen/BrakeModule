
/*
  Software for BrakeModule v0.3.2
  Sofware version 0.3

  The Main purpose of the module is to control charge pump so that the car will brake wo any errors on the car side and be an CC for OP so that it can controlled from the steering wheel buttons.
  Functionalities in software version 0.1:
  - Read CAN data:
    - Car speed
    - Brakelight signal
    - BMW original cruise state and cruise button presses
    - OP brake demand and cancel request
  - Send CAN data:
    - Emulate Toyota cruise messages 0x1D2 and 0x1D3 to control OP
  - Disconnect DSC module from charge pump with relays
  - Control charge pump + side with relay and - side with PWR MOSFET
  - Control cars two brake light signal with relay and P-channel FET
  - Use ADC with DMA to read:
    - DSC line voltage through voltage divider
    - Brakelight BLTS signal (12V) through voltage divider

Remember that in STM32F103xC max currents are (datasheet p. 43):
- Total current into VDD /V DDA power lines (source) 150 mA
- Output current sunk by any I/O and control pin 25 mA
- Output current source by any I/Os and control pin 25 mA
- Injected current on five volt tolerant pins -5/+0 mA
- Injected current on any other pin +/- 5 mA
- Total injected current (sum of all I/O and control pins) +/- 25 mA

  v0.2 will try to:
  - Fix bug where DSC gives DTC when harshly braking and OP disconnect (I believe it is caused by brake light signal going to zero w high brakeline pressure (zeroing problem)
    - Add brakepressure reading from 0x77F to CAN read
    - Add if statement not to disenage brakelight signal if brakepressure over threashold
  - Make change to OP disengage logik so that it will disconnect only when brake pedal is pressed and OP want's to accelerate, not only for brakelight signal
    - Add GAS_COMMAND from 0x200 to CAN read
  - Some code cleanup
    - Delete STM32 uncommented variables
    - Uncomment vout and temp variables
    - Delete uncommented old MyTim->setPWM functions
    
  v0.3 will try to:
  - Make relays state changes to functions -> DONE
    - Use CMSIS calls instead of arduino functions -> DONE
  - Add safety measures
    - Verify GPIO OUTPUT state (IDR is updated every APB2 bus clock cycle (on datasheet APB2 bus speed is 72 MHz so check this) so there might be some delay in pin state change detection)
      -> FUNCTION made
    - Lock pin configuration in GPIOx_LCKR register, so that our pin counfigurations won't change for accidential bit flip (have to check if this is configuration lock or pin state lock) http://slemi.info/2017/04/16/configuring-io-pins/
      -> DONE?
    - Panic at high STM32 chip temperature
    - IWDG -> Done before
    - Detect ADC freeze (see if there is some normal fluctuation on values) -> DONE
    - ADC averaging (suppress one off)
    - ADC min/max values (analogWatchdog?) / max ROC / not possible values
    - CAN msg CRC check (can be done in HW? and can this be implement somehow to other critical data?)  -> DONE?
    - Add bit and operation for CAN msg byte relevant bits (decrease possibility of bit flip messing the wanted value)
    - CAN bus fail detection -> It is DONE in STM32F1_CAN.h and if not 0x343 is correctly received in x amount cycles
    - Brake demand boudaries -> DONE?
    - SetSpeed min/max values -> DONE?
    - Read and compare:
      - Brake pressure from CAN
      - Brake pedal state from CAN
    - Compare pump line voltage to DSC line voltage when -> NOT POSSIBLE AT THIS HW
    - Think how transition from BM ON state to OFF state should be done so that it would be most unlikely that the BM will trigger DTC in DSC
        - Is it possible to DSC to run CP when brake pedal (or brakepressure) press is detected
            - If PMP line voltage is detected wo brake demand don't switch to OFF state
            - How would brake pressure zeroing problem would be tackled when transitiong from ON to OFF state in panic mode
                - Is this problem in DSC event
        - Is best strategy to change BM state or track DSC brake demand and run BM accordingly


  TODO:
  - Make OP disconnect only when brake pedal pressed AND OP wants to accelerate, this gives flexibility to S&G driving and is probably nowadays accepted protocol by comma
  - Look if ADCwatchdog stuff really needed
  - Clean the AVR/STM32 parallel code
  - Test if releasetimer could be replaced with brakepressure
  - Try to find out what is wrong with TMP36 -> Defected sensor
  - Implement internal temperature sensor (for what?)
  - Add 1M looptyme to CAN
  - Don't connect pump back to DSC if brake pressure is above certain threshold (try to prevent bug that triggers DSC DTC if braked hard and pump is running and then connects back to DSC)
  - Clean readSerial() stuff -> Done?
  - Have multiple redings of each ADC channel (really needed for this application?)
  - Implement BLS LOW side detection (is it possible w this HW?)
  
*/

#include "STM32F1_CAN.h"
#include "ADC_DMA.h"
#include <IWatchdog.h>

// #define FAN_CTRL                          // If defined use FAN for PWR MOSFET temperature control
// #define DEBUG                             // If defined use serial for debugging -> This is defined in "STM32F1_CAN.h"

#define RAMP_VALUE  60U                    // Interval time between multiple button presses detections in milliseconds
#define INTERVAL    30U                    // Interval time at which to send 0x1D2 and 0x1D3 messages (milliseconds)
#define UP          32U                    // This is UP button press value for button press byte (BTN_CMD)
#define DWN         64U                    // This is DOWN button press value for button press byte (BTN_CMD)
#define RSM         96U                    // This is RESUME button press value for button press byte (BTN_CMD)

#define FROZEN_THRESHOLD 10               // Freezed ADC detection threshold value
#define DSC_THRESHOLD 3000U                // Value that trigger DSC pump demand
#define DSC_MAX 3500U                      // MAX value that should be detected in DSC_volt


uint32_t dallasTyme = 0;                  // 5 sec interval time value for reading DS18B20 and some other stuff
uint32_t loopTyme = 0;                    // Time value for every 1 million loop cycles
uint32_t currentMillis = 0;               // Looptime
uint32_t previousMillis1 = 0;             // Time value for last sent 0x1D2 msg
uint32_t previousMillis2 = 15;            // Time value for last sent 0x1D3 msg
uint32_t timerRelease = 0;                // Time value for delaying the transition from direct drive of the charge pump back to ABS control
uint32_t rampTyme = 0;                    // Time value for interval value for detecting multiple set speed up or down value
uint32_t loopydoo = 0;                    // Time difference value for every 1 million loop cycles
uint32_t counter = 0;                     // Count value for counting 1 million loops
uint32_t CAN_count = 0;                   // Count value for number of CAN msg reads

int16_t  whole = 0;                       // Whole number of the DS18B20 temperature read
int16_t  BRK_CMD = 0;                     // Deceleration value (wanted braking) from OP read from CAN
uint16_t errcounter = 0;                  // Errorcounter for disabling OP if non 0x343 is received at 65000 cycletimes
uint16_t CNL_REQ_CNT = 0;                 // Counter for detecting BrakeModule cancel request (disable module)
uint16_t BRK_ST_CNT = 0;                  // Counter for delaying BRK_ST_OP happening so that OP wont disengage unnessaccarely
uint16_t DSC_volt = 0;                    // Value of voltage from DSC+ line in milliVolts (this has to be int if used in minus calculation)
uint16_t previousDSC_volt = 0;            // Save previous DSC_volt for freezed ADC detection (this has to be int if used in minus calculation)
uint16_t car_speed = 0;                   // Value of the readed car speed
uint16_t ADC_frozen = 0;                  // Counter for frozed ADC values

uint8_t set_speed = 0;                    // Speed value sent to OP to be the set speed of the cruise control
uint8_t BRK_ST = 0;                       // Brake pedal state from CAN
uint8_t BRK_ST_OP = 0;                    // Brake pedal state send for OP by CAN
uint8_t BTN_CMD = 0;                      // Steering wheel button state from CAN
uint8_t OCC = 0;                          // Car cruise control pre-enable state from CAN
uint8_t CNL_REQ = 0;                      // Cancel request (disable) to BrakeModule sent by OP through CAN
uint8_t BRK_CMD_CRC = 0;                  // 0x343 CRC calculation

bool BRK_FLG = false;                     // Braking logik flags, flags true when OP starts to brake
bool RLS_FLG = false;                     // Braking logik flags, flag for intermediate step for going back from OP control to ABS control
bool ADC_FLG;                             // Flag for ADC conrversion
bool FAN_ON = 0;                          // Flag for FAN control
uint8_t CC_ST_OP = false;                 // State of emulation of TOYOTA cruise control sent to OP
bool BTN_PRS_FLG = false;                 // Flag for Steering wheel button presses
// bool fail_flag = false;                // Flag for any failure that would prevent OPENPILOT to engage and BrakeModule to turn ON

//Interpolation shit, this is done BC relationship of PWM and brake pressure is not linear

// in[] holds the read values from BRK_CMD so are the INPUT values for the interpolation for OUTPUTing the PWM values regaring of the INPUT
// note: the in array should have increasing values
int cmd_in[]  = {-1000, -900, -800, -700, -600, -500, -400, -300, -200, -100, -50};
// out[] holds the values wanted in OUTPUT regarding to INPUT in[], these are PWM duty cycle% values that are interpolated from BRK_CMD values
int pwm_out[] = {100, 97, 93, 88, 81, 73, 67, 59, 54, 39, 20};    // This is STM323F1 values

// ******************************* STM32 Variables *******************************************************

#define pwm_pin PA8
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

uint16_t freq = 15000;

//uint16_t vout = 0;
//int16_t temp = 0;

uint16_t BLTS_volt = 0;                   // Brake-light test switch circuit read
uint16_t BRK_PRS = 0;
uint16_t ACC_CMD = 0;

// uint16_t id;    // This is unnessaccary? Why don't just use CAN_RX_msg.id
// uint8_t dlc;
// uint8_t arry[8];

int16_t PWM = 0;

// Setup Hardware timer
//HardwareTimer *MyTim;
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm_pin), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm_pin), PinMap_PWM));
HardwareTimer *MyTim = new HardwareTimer(Instance);


// *******************************************************************************************************
// *********************************************** SETUP *************************************************
// *******************************************************************************************************
void setup() {

#ifdef DEBUG 
  // initialize serial:
  Serial.begin(230400);                                 // I use this w STM32
#endif
  
  // Setup pins   HAL configuration GPIO stuff https://simonmartin.ch/resources/stm32/dl/STM32%20Tutorial%2001%20-%20GPIO%20Operations%20using%20HAL%20(and%20FreeRTOS).pdf
  pinMode(PA7, OUTPUT);     // 12V relay
  pinMode(PB0, OUTPUT);     // PMP+ relay
  pinMode(PB1, OUTPUT);     // PMP- relay
  pinMode(PB12, OUTPUT);    // BLS_HIGH driver
  pinMode(PB13, OUTPUT);    // BLS_LOW relay driver
  pinMode(PB14, OUTPUT);    // FAN driver
  pinMode(PC13, OUTPUT);    // Blinky LED

  // This is to make sure that brakelight signal transistors are in right state and not floating (really needed?)
  digitalWrite(PB12, HIGH);
  digitalWrite(PB13, HIGH);
  digitalWrite(PB12, LOW);
  digitalWrite(PB13, LOW);

  // Setup the PWM
  MyTim->setPWM(channel, pwm_pin, freq, 0); 
  //MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);

  // Setup ADC
  // Used and modded ADC config for mulitple channels: https://github.com/MahdiKarimian/Multiple-ADC-with-DMA-in-STM32-without-interrupt/blob/main/main.c
  // Great tutorial for CMSIS ADC DMA config https://www.youtube.com/watch?v=Gn1UhtXqiaI&t=61s

  /* Configure the ADC peripheral */
  ADC_Config();
  
  /* Run the ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) {
    /* Calibration Error */
    Error_Handler();
  }
  /* Start ADC conversion on regular group with transfer by DMA */
  if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)ADCValues, ADC_BUFFER_SIZE) != HAL_OK) {
    /* Start Error */
    Error_Handler();
  }

  // Setup CAN
  bool ret = CANInit(CAN_500KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  if (!ret) 
  {
    while(true) {}
  }

  // LOCK pin configurations after intialization
  port_lock_conf();

  // README for IWacthdog library https://github.com/stm32duino/Arduino_Core_STM32/blob/main/libraries/IWatchdog/README.md
  // Initialize the IWDG with 4 seconds timeout.
  // This would cause a CPU reset if the IWDG timer
  // is not reloaded in approximately 400 milliseconds.
  IWatchdog.begin(400000);
  
}



// *******************************************************************************************************
// ********************************************* LOOPYDOO ************************************************
// *******************************************************************************************************

void loop() {

  currentMillis = millis();

  // This is to show how fast is the looptyme, the value given is average loop excecution time per 1 million loopcycles in nanoseconds
  if (counter > 1000000U) {
   loopydoo = currentMillis - loopTyme;
   #ifdef DEBUG
   Serial.print("Loop tyme was in 1M cycles: ");
   Serial.println(loopydoo);
   #endif
   counter = 0;
   loopTyme = millis();
  }


// ******************************* READ SERIAL FOR INPUT ***************************************** 

#ifdef DEBUG
  readSerial();
#endif


// ************************************** ADC STUFF **********************************************
  
  DSC_volt = ADCValues[0];
  BLTS_volt = ADCValues[1];

  // Check if the ADC value is changing by more than the threshold
  // over multiple readings
  if (abs(DSC_volt - previousDSC_volt) < FROZEN_THRESHOLD) {
    // The ADC is frozen, so do something about it (e.g. try restarting it)
    ADC_frozen++;
    if (ADC_frozen > 10000U) {     // TODO: Calculate if this value changes per loop and how long this takes if frozen
      fail_flag = true;
      #ifdef DEBUG
        Serial.println("ADC frozed");
      #endif
    }
  }
  else {
    ADC_frozen = 0;
  }

  // Save the current ADC value for the next iteration
  previousDSC_volt = DSC_volt;

  // Detect not possible value from DSC
  if (DSC_volt > DSC_MAX) {
    fail_flag = true;
  }

// ************************************* READ CAN STUFF ******************************************

  CAN_msg_t CAN_RX_msg;

  // if(CANMsgAvail()) {
  if(CANMsgAvail() != 0U) {
    CANReceive(&CAN_RX_msg);
    //id = CAN_RX_msg.id;

  // Put needed CAN DATA into variables
    //if (id == 0x153) {                         // Read speed from 0x153 and do some conversion
    if (CAN_RX_msg.id == 0x153) {                         // Read speed from 0x153 and do some conversion
      //car_speed = (((CAN_RX_msg.data[2] << 3) + (CAN_RX_msg.data[1] >> 5)) * 0.25) - 2.64;
      car_speed = ((((CAN_RX_msg.data[2] & B00011111) << 3) + ((CAN_RX_msg.data[1] & B11100000) >> 5)) * 0.25) - 2.64;
      //Serial.println("0x153");
      CAN_count++;
    }
    //if (id == 0x200) {                         // Read GAS_COMMAND request value from OP
    if (CAN_RX_msg.id == 0x200) {                         // Read GAS_COMMAND request value from OP
      ACC_CMD = ((CAN_RX_msg.data[0] << 8) | CAN_RX_msg.data[1]);
      //Serial.println("0x200");
      CAN_count++;
    }
    //if (id == 0x329) {
    if (CAN_RX_msg.id == 0x329) {
      BTN_CMD = (CAN_RX_msg.data[3] & B01100000);             // &B01100000 is to make sure that other bits in the byte don't bother
      BRK_ST = (CAN_RX_msg.data[6] & B00000001);              // Read brake pedal switch state
      BRK_ST_CNT++;                                       // Use this for not triggering BRK_ST_OP too early after OP has stopped braking
      //Serial.println("0x329");
      CAN_count++;
    }
    //if (id == 0x343) {
    if (CAN_RX_msg.id == 0x343) {
      uint8_t cksum = can_cksum(CAN_RX_msg.data, 7, 0x343); // Calculate TOYOTA checksum
      if (cksum == CAN_RX_msg.data[7]) {
        BRK_CMD = ((CAN_RX_msg.data[0] << 8) | CAN_RX_msg.data[1]);   // Read brake request value from OP
        BRK_CMD = max(-3500, BRK_CMD);                          // Is these values clipped already in interpolation
        BRK_CMD = min(100, BRK_CMD);                            // Is these values clipped already in interpolation
        CNL_REQ = (CAN_RX_msg.data[3] & B00000001);             // Read cancel request flag from OP
        //Serial.println("0x343");
        CAN_count++;
        errcounter = 0;                                        // If 0x343 read and checksum OK, set error counter to 0        
      }
      else {
        fail_flag = true;                                     // If 0x343 checksum does not match raise fail_flag to disable OP
      }
    }
    //if (id == 0x545) {
    if (CAN_RX_msg.id == 0x545) {
      OCC = (CAN_RX_msg.data[0] & B00001000);                 // If OCC == 0x08 BMW original cruise is pre-enabled (cruise light is ON) and OP should not engage!
      //Serial.println("0x545");
      CAN_count++;
    }
    //if (id == 0x77F) {                        // Read brakepressure from 0x77F in hehtopascals
    if (CAN_RX_msg.id == 0x77F) {                        // Read brakepressure from 0x77F in hehtopascals
      //BRK_PRS = (CAN_RX_msg.data[7] << 6 | CAN_RX_msg.data[6] >> 2);
      BRK_PRS = ((CAN_RX_msg.data[7] & B00000011) << 6) | ((CAN_RX_msg.data[6] & B11111100) >> 2);
      //Serial.println("0x77F");
      CAN_count++;
    }
  }




// ******************************** OP LOGIK STUFF **********************************************

  if ((BTN_CMD == RSM) && (BTN_PRS_FLG == false)) {                // Engage/disengage OP if resume button press is detected, BTN_PRS_FLG prevents multiple button presses
    if (CC_ST_OP == false) {                                   // If OP cruise state is false
      set_speed = car_speed;                                   // When OP CC activation is set, save car_speed to set_speed
      CNL_REQ_CNT = 0;
      #ifdef DEBUG
      Serial.println("Speed setted");
      #endif
    }
    CC_ST_OP = !CC_ST_OP;                                      // If resume button pressed change OP cruise control state flag
  }

  if ((BTN_CMD == UP) && (currentMillis - rampTyme) > RAMP_VALUE) {       // Ramp up set speed in orderly manner, 60 ms delay on incrementing set speed value so that change isn't too fast
    set_speed++;
    set_speed = min(set_speed, 135U);                                    // Restrict max value of set_speed to 135
    rampTyme = currentMillis;
  }
  if ((BTN_CMD == DWN) && (currentMillis - rampTyme) > RAMP_VALUE) {      // Ramp down set speed in orderly manner, 60 ms delay on incrementing set speed value so that change isn't too fast
    set_speed--;
    set_speed = max(set_speed, 0U);                                      // Restrict min value of set_speed to 0
    rampTyme = currentMillis;
  }

  if (BTN_CMD == RSM) {                                         // If resume cruise button press is detected, set RSM_PRS_FLG to avoid multiple button press detections
    BTN_PRS_FLG = true;                                         // This should avoid making multiple button presses at one long button press if (BTN_PRS_FLG == false) is used
  }
  else {
    BTN_PRS_FLG = false;
  }

// //  Option 2: Tested and worked! 
//   if (BRK_ST == 1 && BRK_FLG == false) {                        // If brake pedal pressed and OP is no braking, trigger brake flag for OP to know after consective 5 msgs
//     if (BRK_ST_CNT > 5) {                                       // If 0x153 interval is 10 ms, BRK_ST_OP it will be triggered after 60 ms (or is it 70 ms?)
//       BRK_ST_OP = 1;
//       BRK_ST_CNT = 0;
//       #ifdef DEBUG
//       //Serial.println("BRAKE!");
//       #endif
//     }
//   }
//   else {
//     BRK_ST_OP = 0;
//     //BRK_ST_CNT = 0;                                             // Maybe if this is deleted, BRK_ST_OP is triggered faster BC counter is most probably > 5
//   }
 

  // New OP brake signal logik based on reading OP acceleration demand and brake pedal state. NOTE TO SELF make better logik, this for testing only
  if ((BLTS_volt > 1500U) && (ACC_CMD > 500U)) {   // 
    BRK_ST_OP = 1;
  }
  else {
    BRK_ST_OP = 0;
  }
  
  if (errcounter == 0U) {                                        // If 0x343 msg is received, this is triggered BC errcounter is set to 0
    CNL_REQ_CNT++;
    if (CNL_REQ_CNT > 5U) {                                      // 0x343 interval is 10 ms so this is triggered every 60 ms (or is it 70 ms?)
      if ((CNL_REQ == 1U) || (OCC == 8U)) {                           // If OP has sent cancel request or BMW original CC is pre-enabled, set TOYOTA CC ACTIVE flag to false
        CC_ST_OP = false;
      CNL_REQ_CNT = 0;
      }
    }
  }


// ************************************** BRAKE LOGIK *******************************************

  // After braking event disconnect charge pump from main voltage before connecting it back to DSC-module
  if ((BRK_CMD > -30) && (BRK_FLG == true) && (RLS_FLG == false)) {  // This is when we wanna go back to normal, isolate pump from 12V (relay & MOSFET) and start releasetimer
    // digitalWrite(PA7, LOW);                 // 12V to PUMP relay LOW == disconnect
    //OCR1A = 0;                            // Set D9 PWM duty cycle to 0% (B- LOW)   COMMENT! MAYBE disable this, could to be better for the FLYBACKING voltage to get DOWN, do it @ releasetimer
    pwr_relay_off();
    MyTim->setCaptureCompare(channel, 100, PERCENT_COMPARE_FORMAT);     // Set PWM duty cycle to 0% (B- LOW)   COMMENT! MAYBE disable this, could to be better for the FLYBACKING voltage to get DOWN, do it @ releasetimer
    timerRelease = currentMillis;         // Set timer for releasing the control back to DSC after settling time (if switch is done too fast car gives DSC error)
    RLS_FLG = true;                       // Set brakingReleaseFlag to true
  }

  // If upper function was excecuted and we went back to braking territory, set BRK_FLG and RLS_FLG false so we can start braking before timerRelease (lower func) is excecuted
  if ((BRK_CMD < -50) && (RLS_FLG == true)) {
    MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
    BRK_FLG = false;
    RLS_FLG = false;
  }
 
  // 600 ms after the pump was disconnected from 12V, turn OFF brakelights and turn OFF DSC relays so that the pump is controlled by DSC-module
  //if (((currentMillis - timerRelease) > 600) && RLS_FLG == true) {    // Original value 400
  if (((currentMillis - timerRelease) > 600U) && (RLS_FLG == true) && (BRK_PRS < 6U)) {    // Test high brakepressure vs brakelight signal fix
    MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);     // Set PWM duty cycle to 0%      // Is this needed to do again?
    relays_off();

    // digitalWrite(PB13, LOW);              // Set BLS LOW side OFF
    // digitalWrite(PB12, LOW);              // Set BLS HIGH side OFF
    // digitalWrite(PB0, LOW);               // Set DSC relays LOW
    // digitalWrite(PB1, LOW);               // Set DSC relays LOW

    RLS_FLG = false;
    BRK_FLG = false;
    BRK_ST_CNT = 0;                       // This is needed BC otherwise BRK_ST -> BRK_ST_OP might be triggered too early
    #ifdef DEBUG
    Serial.println("Not braking");
    #endif
  }

  // This is when we want to start braking
  if ((BRK_CMD < -50) && (BRK_FLG == false)) {
    // digitalWrite(PB0, HIGH);
    // digitalWrite(PB1, HIGH);

    // digitalWrite(PB13, HIGH);                     // Set BLS LOW side ON
    // digitalWrite(PB12, HIGH);                    // Set BLS HIGH side ON

    // digitalWrite(PA7, HIGH);                      // 12V to PUMP relay HIGH == connect

    relays_on();
    BRK_FLG = true;
    #ifdef DEBUG
    Serial.println("Braking");
    #endif
  }

  //if (BRK_FLG == true && RLS_FLG == false){   // This is done this way BC then upper func wont be excecuted in every loop, only this
  if ((BRK_FLG == true) && (RLS_FLG == false) && (errcounter == 0U)){   // This is done this way BC so that the PWM wont be done at every cycle loop but every 10 ms (change of 0x343)
    // Check if DSC line has drive voltage and drive the FET accordingly (this is safety feature)
    if (DSC_volt > DSC_THRESHOLD){
      PWM = 100;
    }
    else {
      PWM = multiMap(BRK_CMD, cmd_in, pwm_out, 11);
    }
    MyTim->setCaptureCompare(channel, PWM, PERCENT_COMPARE_FORMAT);
  }


// ************************************* SEND CAN STUFF *****************************************

  // This is emulating TOYOTA cruise controller messages sent to OP, TOYOTA CC is used BC of legacy reasons :)
  CAN_msg_t CAN_TX_msg;
  
  // Send 0x1D2 msg PCM_CRUISE
  if ((currentMillis - previousMillis1) >= INTERVAL) {
    // Save the last time you send message
    previousMillis1 = currentMillis;

    CAN_TX_msg.type = DATA_FRAME;

    CAN_TX_msg.id  = 0x1D2;
    CAN_TX_msg.len = 8;
    CAN_TX_msg.data[0] = (CC_ST_OP << 5) & 0x20U;           // The wanted state of the OP cruise control (CC_ST_OP)
    CAN_TX_msg.data[1] = 0x0;
    CAN_TX_msg.data[2] = 0x0;
    CAN_TX_msg.data[3] = 0x0;
    CAN_TX_msg.data[4] = BRK_ST_OP & 0x01U;                 // Brake pedal state sent to OP (BRK_ST_OP)
    CAN_TX_msg.data[5] = 0x0;
    CAN_TX_msg.data[6] = (CC_ST_OP << 7) & 0x80U;           // The wanted state of the OP cruise control (CC_ST_OP)
    CAN_TX_msg.data[7] = can_cksum(CAN_TX_msg.data, 7, 0x1D2);
    CANSend(&CAN_TX_msg);
  }

  // Send 0x1D3 msg PCM_CRUISE_2
  if ((currentMillis - previousMillis2) >= INTERVAL) {
    // Save the last time you send message
    previousMillis2 = currentMillis;

    CAN_TX_msg.id  = 0x1D3;
    CAN_TX_msg.len = 8;
    CAN_TX_msg.data[0] = PWM;                                // Debugging msg, PWM value
    CAN_TX_msg.data[1] = 0xA0;                               // 0xA0 is for setting the pre-enable state of the TOYOTA cruise control (MAIN ON and LOW_SPEED_LOCKDOWN = 1)
    CAN_TX_msg.data[2] = set_speed;                          // Speed value sent to OP to set the cruise controller set speed
    CAN_TX_msg.data[3] = (BLTS_volt >> 8);                   // Debugging msg, BLTS wire voltage in milliVolts
    CAN_TX_msg.data[4] = BLTS_volt;                          // Debugging msg, BLTS wire voltage in milliVolts
    CAN_TX_msg.data[5] = (DSC_volt >> 8);                    // Debugging msg, DSC+ wire voltage in milliVolts
    CAN_TX_msg.data[6] = DSC_volt;                           // Debugging msg, DSC+ wire voltage in milliVolts
    CAN_TX_msg.data[7] = can_cksum(CAN_TX_msg.data, 7, 0x1D3);
    CANSend(&CAN_TX_msg);
  }


// ********************************** DEBUGGING ***************************************** 

  // Do this every 1 sec so it wont bother other stuff so much
  if ((currentMillis - dallasTyme) > 1000U) {
    #ifdef DEBUG
    digitalWrite(PC13, !digitalRead(PC13));
    Serial.print("ADC[0] (PA0) value is ");
    Serial.println(DSC_volt);
    Serial.print("ADC[1] (PA5) value is ");
    Serial.println(ADCValues[1]);
    Serial.print("ADC[2] value (PA6) is ");
    Serial.println(ADCValues[2]);
    Serial.print("ADC watchdog status is ");
    Serial.println(ubAnalogWatchdogStatus);
    Serial.print("Brake pressure is ");
    Serial.println(BRK_PRS);
    Serial.println(BRK_CMD);
    Serial.println(PWM);
    #endif
    
    dallasTyme = millis();
  }

  
// *********************************** FAN CONTROL **************************************

// NOTE TO SELF this is not implemented on 0.3.2 version bc havent got TMP36 working and cooling unlikely neccassary

  #ifdef FAN_CTRL
  if ((whole > 45) && !FAN_ON) {
    digitalWrite(PB14, HIGH);                 // Turn FAN pin HIGH when FET temp over 45 degrees
    FAN_ON = 1;
  }
  if ((whole < 45) && FAN_ON) {
    digitalWrite(PB14, LOW);;                // Turn FAN pin LOW when FET temp under 45 degrees
    FAN_ON = 0;
  }
  #endif

  if (whole > 80) {               // If FET temp over 80 degrees disable OP
    CC_ST_OP = false;
    CNL_REQ = 1;
  }
  
  if ((errcounter > 65000U) || (fail_flag == true)) {      // errcounter func is triggered every (65000 x ~9us =) 585 ms if no 0x343 msg is detected which should arrive every 10 ms
    CC_ST_OP = false;
    CNL_REQ = 1;
    errcounter = 0;
    MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
    digitalWrite(PA7, LOW);               // 12V to PUMP relay HIGH == connect
    digitalWrite(PB0, LOW);
    digitalWrite(PB1, LOW);
    digitalWrite(PB13, LOW);              // Set BLS LOW side OFF
    digitalWrite(PB12, LOW);              // Set BLS HIGH side OFF
  }
  
  errcounter++;                           // Add errcounter value, this value is set to zero allways when 0x343 is detected
  counter++;                              // Add counter value, this is for counting loops for measuring average looptyme
  ubAnalogWatchdogStatus = RESET;         // This will set WatchdogStatus to zero when analogvalues are inside bounds

  // make sure the code in this loop is executed in
  // less than 4 milliseconds to leave 50% headroom for
  // the timer reload.
  IWatchdog.reload();
}

// *******************************************************************************************************
// ***************************************** END of LOOPYDOO *********************************************
// *******************************************************************************************************

// ******************************************** FUNCTIONS ************************************************

// TOYOTA CAN CHECKSUM
// I think this is written by wocsor, same as the TOYOTA cruise CAN send stuff
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00U) >> 8) + (addr & 0x00FFU) + len + 1U;
  for (uint8_t i = 0; i < len; i++) {
    checksum += (dat[i]);
  }
  return checksum;
}


// Interpolation shit
// This is direct copy from https://github.com/RobTillaart/MultiMap
// note: the _in array should have increasing values
int multiMap(int val, int* in, int* out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= in[0]) 
  {
    return out[0];
  }
  
  if (val >= in[size-1U]) 
  {
    return out[size-1U];
  }

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > in[pos]) 
  {
    pos++;
  }

  // this will handle all exact "points" in the _in array
  if (val == in[pos]) 
  {
    return out[pos];
  }

  // interpolate in the right segment for the rest
  return (val - in[pos-1U]) * (out[pos] - out[pos-1U]) / (in[pos] - in[pos-1U]) + out[pos-1U];
}


// Use faster CMISS defines in pin state changes, using BSRR instead of ODR and BRR u can use atomic instead of or configuration (faster)
// https://www.youtube.com/watch?v=CyfaDirhp9M
// BrakeModule relays state ON
void relays_on()
{
  // PMP- relay to ON (PB0)
  // PMP+ relay to ON (PB1)
  // Set BLS LOW side ON (PB13)
  // Set BLS HIGH side ON (PB12)
  // Set pins 0, 1, 12, 13 HIGH on bit set/reset register on port GPIOB
  GPIOB -> BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1 | GPIO_BSRR_BS12 | GPIO_BSRR_BS13;

  // 12V to PUMP relay (PA7)
  // Set pin 7 HIGH on bit set/reset register on port GPIOA
  GPIOA -> BSRR = GPIO_BSRR_BS7;
}


// BrakeModule 12V relay to charge pump OFF
void pwr_relay_off()
{
  // 12V to PUMP relay (PA7)
  // Set pin 7 LOW on bit set/reset register on port GPIOA
  GPIOA -> BSRR = GPIO_BSRR_BR7;
}

// BrakeModule 12V relay to charge pump OFF
void relays_off()
{
  // PMP- relay (PB0)
  // PMP+ relay to (PB1)
  // Set BLS LOW side (PB13)
  // Set BLS HIGH side (PB12)
  // Set pins 0, 1, 12, 13 LOW on bit set/reset register on port GPIOB
  GPIOB -> BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR12 | GPIO_BSRR_BR13;
}

// Read pin OUTPUT states from input data register IDR (STM32F1 RM0008 p. 163, 172)
int read_output_states()
{
  /* read PC13 */
  //if(GPIOC -> IDR & GPIO_PIN_13)
  uint8_t state;
  if((GPIOB -> IDR & (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13)) && (GPIOA -> IDR & GPIO_PIN_7))
  {
    // Stuff
    state = true;
  }
  else
  {
    state = false;
  }
  return state;
}

// LOCK pin configuration (STM32F1 RM0008 p. 174)
// Once the pin configuration is locked, it cannot be changed until the next reset. 
// This can help prevent accidental or malicious changes to the pin configuration, which could cause your application to malfunction.
void port_lock_conf()
{
  /* LOCK pin configuration of PA7, PB0, PB1, PB12, P13 */
  HAL_GPIO_LockPin(GPIOA, GPIO_PIN_7);
  HAL_GPIO_LockPin(GPIOB, GPIO_PIN_0);
  HAL_GPIO_LockPin(GPIOB, GPIO_PIN_1);
  HAL_GPIO_LockPin(GPIOB, GPIO_PIN_12);
  HAL_GPIO_LockPin(GPIOB, GPIO_PIN_13);

  /* LOCK pin configuration of PA0, PA5, PA6 */
  HAL_GPIO_LockPin(GPIOA, GPIO_PIN_0);
  HAL_GPIO_LockPin(GPIOA, GPIO_PIN_5);
  HAL_GPIO_LockPin(GPIOA, GPIO_PIN_6);
}

// Serial read function for DEBUGGING
void readSerial()
{
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    int16_t input = Serial.parseInt();

    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      Serial.print("Input is: ");
      Serial.println(input);

      if ((input > 1) && (input < 101)) {
        MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
      }
      else if (input == 101) {
        Serial.print("CAN count was ");
        Serial.println(CAN_count);
        CAN_count = 0;
      }
      else if (input == 200) {
        digitalWrite(PA7, LOW);               // 12V to PUMP relay HIGH == connect
        digitalWrite(PB0, LOW);               // PMP+
        digitalWrite(PB1, LOW);               // PMP-
        digitalWrite(PB13, LOW);              // Set BLS LOW side OFF
        digitalWrite(PB12, LOW);              // Set BLS HIGH side OFF
        Serial.println("Braking mode was disabled");
      }
      else if (input == 201) {
        digitalWrite(PB0, HIGH);               // PMP+
        digitalWrite(PB1, HIGH);               // PMP-
        digitalWrite(PB13, HIGH);              // Set BLS LOW side OFF
        digitalWrite(PB12, HIGH);              // Set BLS HIGH side OFF        
        digitalWrite(PA7, HIGH);               // 12V to PUMP relay HIGH == connect
        Serial.println("Braking mode was enabled");
      }
      else if (input == 666) {
        digitalWrite(PB13, !digitalRead(PB13));
        Serial.println("BLS_LOW pin was toggled");
      }
      else if (input == 667) {
        digitalWrite(PB12, !digitalRead(PB12));
        Serial.println("BLS_HIGH was toggled");
      }
      else if (input == 668) {
        digitalWrite(PB1, !digitalRead(PB1));
        Serial.println("PMP- pin was toggled");
      }
      else if (input == 669) {
        digitalWrite(PB14, !digitalRead(PB14));
        Serial.println("FAN pin was toggled");
      }
      else if (input == 700) {
        digitalWrite(PB0, !digitalRead(PB0));
        Serial.println("PMP+ pin was toggled");
      }
      else if (input == 701) {
        digitalWrite(PA7, !digitalRead(PA7));
        Serial.println("12V pin was toggled");
      }
      else if (input == 702) {
        digitalWrite(PA7, HIGH);
        digitalWrite(PB0, HIGH);
        digitalWrite(PB1, HIGH);
        Serial.println("Brake sequence is ON");
      }
      else if (input == 703) {
        digitalWrite(PA7, LOW);
        digitalWrite(PB0, LOW);
        digitalWrite(PB1, LOW); 
        Serial.println("Brake sequence is OFF");
      }
      else if (input < -10) {
        BRK_CMD = input;
        Serial.print("BRK_CMD has been set to value ");
        Serial.println(BRK_CMD);
      }      
      else{
        MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
      }
    }
  }
}
