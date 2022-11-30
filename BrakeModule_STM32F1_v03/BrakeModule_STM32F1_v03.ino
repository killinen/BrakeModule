
/*
  Software for BrakeModule v0.3.2
  Sofware version 0.2

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
    

  TODO:
  - Make OP disconnect only when brake pedal pressed AND OP wants to accelerate, this gives flexibility to S&G driving and is probably nowadays accepted protocol by comma
  - Look if ADCwatchdog stuff really needed
  - Clean the AVR/STM32 parallel code
  - Test if releasetimer could be replaced with brakepressure
  - Try to find out what is wrong with TMP36
  - Implement internal temperature sensor (for what?)
  - Add 1M looptyme to CAN
  - Don't connect pump back to DSC if brake pressure is above certain threshold (try to prevent bug that triggers DSC DTC if braked hard and pump is running and then connects back to DSC)
  - Clean readSerial() stuff -> Done?
  - Have multiple redings of each ADC channel (really needed for this application?)
  - Implement BLS LOW side detection (is it possible w this HW?)
  
*/


//#define FAN_CTRL                          // If defined use FAN for PWR MOSFET temperature control
#define DEBUG                             // If defined use serial for debugging
#define CAN_FILTER                        // If defined use predefined CAN filters for wanted msgs   

#define adcPin 0                          // DSC+ line is readed at A0 pin

#define RAMP_VALUE  60                    // Interval time between multiple button presses detections in milliseconds
#define INTERVAL    30                    // Interval time at which to send 0x1D2 and 0x1D3 messages (milliseconds)
#define UP          32                    // This is UP button press value for button press byte (BTN_CMD)
#define DWN         64                    // This is DOWN button press value for button press byte (BTN_CMD)
#define RSM         96                    // This is RESUME button press value for button press byte (BTN_CMD)


unsigned long dallasTyme = 0;             // 5 sec interval time value for reading DS18B20 and some other stuff
unsigned long loopTyme = 0;               // Time value for every 1 million loop cycles
unsigned long currentMillis = 0;          // Looptime
unsigned long previousMillis1 = 0;        // Time value for last sent 0x1D2 msg
unsigned long previousMillis2 = 15;       // Time value for last sent 0x1D3 msg
unsigned long timerRelease = 0;           // Time value for delaying the transition from direct drive of the charge pump back to ABS control
unsigned long rampTyme = 0;               // Time value for interval value for detecting multiple set speed up or down value
unsigned long loopydoo = 0;               // Time difference value for every 1 million loop cycles

uint32_t counter = 0;                     // Count value for counting 1 million loops
uint32_t CAN_count = 0;                   // Count value for number of CAN msg reads

int16_t  whole = 0;                       // Whole number of the DS18B20 temperature read
int16_t  BRK_CMD = 0;                     // Deceleration value (wanted braking) from OP read from CAN
uint16_t errcounter = 0;                  // Errorcounter for disabling OP if non 0x343 is received at 65000 cycletimes
uint16_t CNL_REQ_CNT = 0;                 // Counter for detecting BrakeModule cancel request (disable module)
uint16_t BRK_ST_CNT = 0;                  // Counter for delaying BRK_ST_OP happening so that OP wont disengage unnessaccarely
uint16_t DSC_volt = 0;                    // Value of voltage from DSC+ line in milliVolts
uint16_t car_speed = 0;                   // Value of the readed car speed

uint8_t set_speed = 0;                    // Speed value sent to OP to be the set speed of the cruise control
uint8_t BRK_ST = 0;                       // Brake pedal state from CAN
uint8_t BRK_ST_OP = 0;                    // Brake pedal state send for OP by CAN
uint8_t BTN_CMD = 0;                      // Steering wheel button state from CAN
uint8_t OCC = 0;                          // Car cruise control pre-enable state from CAN
uint8_t CNL_REQ = 0;                      // Cancel request (disable) to BrakeModule sent by OP through CAN

bool BRK_FLG = false;                     // Braking logik flags, flags true when OP starts to brake
bool RLS_FLG = false;                     // Braking logik flags, flag for intermediate step for going back from OP control to ABS control
bool ADC_FLG;                             // Flag for ADC conrversion
bool FAN_ON = 0;                          // Flag for FAN control
bool CC_ST_OP = false;                    // State of emulation of TOYOTA cruise control sent to OP
bool BTN_PRS_FLG = false;                 // Flag for Steering wheel button presses

//Interpolation shit, this is done BC relationship of PWM and brake pressure is not linear

// in[] holds the read values from BRK_CMD so are the INPUT values for the interpolation for OUTPUTing the PWM values regaring of the INPUT
// note: the in array should have increasing values
int in[]  = {-1000, -900, -800, -700, -600, -500, -400, -300, -200, -100, -50};
// out[] holds the values wanted in OUTPUT regarding to INPUT in[], these are PWM duty cycle% values that are interpolated from BRK_CMD values
int out[] = {100, 97, 93, 88, 81, 73, 67, 59, 54, 39, 20};    // This is STM323F1 values

// ******************************* STM32 Variables *******************************************************

#include "STM32F1_CAN.h"
#include "ADC_DMA.h"
#include <IWatchdog.h>

#define pin PA8

uint32_t chan;
uint16_t freq = 15000;

//uint16_t vout = 0;
//int16_t temp = 0;

int16_t  BLTS_volt = 0;                   // Brake-light test switch circuit read
uint16_t BRK_PRS = 0;
uint16_t ACC_CMD = 0;

uint16_t id;
uint8_t dlc;
uint8_t arry[8];

int16_t PWM = 0;

// Setup Hardware timer
//HardwareTimer *MyTim;
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
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
  //pinMode(PA12, OUTPUT);    // Nothing?
  //pinMode(PA15, OUTPUT);    // BLS_LOW detection
  pinMode(PB0, OUTPUT);     // PMP+ relay
  pinMode(PB1, OUTPUT);     // PMP- relay
  pinMode(PB12, OUTPUT);    // BLS_HIGH driver
  pinMode(PB13, OUTPUT);    // BLS_LOW relay driver
  pinMode(PB14, OUTPUT);    // FAN driver
  pinMode(PC13, OUTPUT);    // Blinky LED
  //pinMode(PA15, INPUT_ANALOG);    // BLS LOW
  //pinMode(PA5, INPUT_ANALOG);    // BLS HIGH

  // This is to make sure that brakelight signal transistors are in right state and not floating (really needed?)
  digitalWrite(PB12, HIGH);
  digitalWrite(PB13, HIGH);
  digitalWrite(PB12, LOW);
  digitalWrite(PB13, LOW);

  // Setup the PWM
  MyTim->setPWM(channel, pin, freq, 0); 
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
  if (!ret) while(true);

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
  if (counter > 1000000) {
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

// ************************************* READ CAN STUFF ******************************************

  CAN_msg_t CAN_RX_msg;

  if(CANMsgAvail()) {
    CANReceive(&CAN_RX_msg);
    id = CAN_RX_msg.id;

  // Put needed CAN DATA into variables
    if (id == 0x153) {                         // Read speed from 0x153 and do some conversion
      car_speed = (((CAN_RX_msg.data[2] << 3) + (CAN_RX_msg.data[1] >> 5)) * 0.25) - 2.64;
      //Serial.println("0x153");
      CAN_count++;
    }
    if (id == 0x200) {                         // Read GAS_COMMAND request value from OP
      ACC_CMD = (CAN_RX_msg.data[0] << 8 | CAN_RX_msg.data[1]);
      //Serial.println("0x200");
      CAN_count++;
    }
    if (id == 0x329) {
      BTN_CMD = (CAN_RX_msg.data[3] & B01100000);             // &B01100000 is to make sure that other bits in the byte don't bother
      BRK_ST = (CAN_RX_msg.data[6] & B00000001);              // Read brake pedal switch state
      BRK_ST_CNT++;                                       // Use this for not triggering BRK_ST_OP too early after OP has stopped braking
      //Serial.println("0x329");
      CAN_count++;
    }
    if (id == 0x343) {
      BRK_CMD = (CAN_RX_msg.data[0] << 8 | CAN_RX_msg.data[1]);   // Read brake request value from OP
      CNL_REQ = (CAN_RX_msg.data[3] & B00000001);             // Read cancel request flag from OP
      errcounter = 0;                                     // If 0x343 read, set error counter to 0
      //Serial.println("0x343");
      CAN_count++;
    }
    if (id == 0x545) {
      OCC = (CAN_RX_msg.data[0] & B00001000);                 // If OCC == 0x08 BMW original cruise is pre-enabled (cruise light is ON) and OP should not engage!
      //Serial.println("0x545");
      CAN_count++;
    }
    if (id == 0x77F) {                        // Read brakepressure from 0x77F in hehtopascals
      BRK_PRS = (CAN_RX_msg.data[7] << 6 | CAN_RX_msg.data[6] >> 2);
      //Serial.println("0x77F");
      CAN_count++;
    }
  }




// ******************************** OP LOGIK STUFF **********************************************

  if (BTN_CMD == RSM && BTN_PRS_FLG == false) {                // Engage/disengage OP if resume button press is detected, BTN_PRS_FLG prevents multiple button presses
    if (CC_ST_OP == false) {                                   // If OP cruise state is false
      set_speed = car_speed;                                   // When OP CC activation is set, save car_speed to set_speed
      CNL_REQ_CNT = 0;
      #ifdef DEBUG
      Serial.println("Speed setted");
      #endif
    }
    CC_ST_OP = !CC_ST_OP;                                      // If resume button pressed change OP cruise control state flag
  }

  if (BTN_CMD == UP && (currentMillis - rampTyme) > RAMP_VALUE) {       // Ramp up set speed in orderly manner, 60 ms delay on incrementing set speed value so that change isn't too fast
    set_speed++;
    rampTyme = currentMillis;
  }
  if (BTN_CMD == DWN && (currentMillis - rampTyme) > RAMP_VALUE) {      // Ramp down set speed in orderly manner, 60 ms delay on incrementing set speed value so that change isn't too fast
    set_speed--;
    rampTyme = currentMillis;
  }

  if (BTN_CMD == RSM) {                                         // If resume cruise button press is detected, set RSM_PRS_FLG to avoid multiple button press detections
    BTN_PRS_FLG = true;                                         // This should avoid making multiple button presses at one long button press if (BTN_PRS_FLG == false) is used
  }
  else {
    BTN_PRS_FLG = false;
  }
/*
//  Option 2: Tested and worked! 
  if (BRK_ST == 1 && BRK_FLG == false) {                        // If brake pedal pressed and OP is no braking, trigger brake flag for OP to know after consective 5 msgs
    if (BRK_ST_CNT > 5) {                                       // If 0x153 interval is 10 ms, BRK_ST_OP it will be triggered after 60 ms (or is it 70 ms?)
      BRK_ST_OP = 1;
      BRK_ST_CNT = 0;
      #ifdef DEBUG
      //Serial.println("BRAKE!");
      #endif
    }
  }
  else {
    BRK_ST_OP = 0;
    //BRK_ST_CNT = 0;                                             // Maybe if this is deleted, BRK_ST_OP is triggered faster BC counter is most probably > 5
  }
 */

  // New OP brake signal logik based on reading OP acceleration demand and brake pedal state. NOTE TO SELF make better logik, this for testing only
  if (BLTS_volt > 1500 && ACC_CMD > 500) {   // 
    BRK_ST_OP = 1;
  }
  else {
    BRK_ST_OP = 0;
  }
  
  if (errcounter == 0) {                                        // If 0x343 msg is received, this is triggered BC errcounter is set to 0
    CNL_REQ_CNT++;
    if (CNL_REQ_CNT > 5) {                                      // 0x343 interval is 10 ms so this is triggered every 60 ms (or is it 70 ms?)
      if (CNL_REQ == 1 || OCC == 8) {                           // If OP has sent cancel request or BMW original CC is pre-enabled, set TOYOTA CC ACTIVE flag to false
        CC_ST_OP = false;
      CNL_REQ_CNT = 0;
      }
    }
  }


// ************************************** BRAKE LOGIK *******************************************

  // After braking event disconnect charge pump from main voltage before connecting it back to DSC-module
  if (BRK_CMD > -30 && BRK_FLG == true && RLS_FLG == false) {  // This is when we wanna go back to normal, isolate pump from 12V (relay & MOSFET) and start releasetimer
    digitalWrite(PA7, LOW);                 // 12V to PUMP relay LOW == disconnect
    //OCR1A = 0;                            // Set D9 PWM duty cycle to 0% (B- LOW)   COMMENT! MAYBE disable this, could to be better for the FLYBACKING voltage to get DOWN, do it @ releasetimer
    MyTim->setCaptureCompare(channel, 100, PERCENT_COMPARE_FORMAT);     // Set PWM duty cycle to 0% (B- LOW)   COMMENT! MAYBE disable this, could to be better for the FLYBACKING voltage to get DOWN, do it @ releasetimer
    timerRelease = currentMillis;         // Set timer for releasing the control back to DSC after settling time (if switch is done too fast car gives DSC error)
    RLS_FLG = true;                       // Set brakingReleaseFlag to true
  }

  // If upper function was excecuted and we went back to braking territory, set BRK_FLG and RLS_FLG false so we can start braking before timerRelease (lower func) is excecuted
  if (BRK_CMD < -50 && RLS_FLG == true) {
    MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
    BRK_FLG = false;
    RLS_FLG = false;
  }
 
  // 600 ms after the pump was disconnected from 12V, turn OFF brakelights and turn OFF DSC relays so that the pump is controlled by DSC-module
  //if (((currentMillis - timerRelease) > 600) && RLS_FLG == true) {    // Original value 400
  if (((currentMillis - timerRelease) > 600) && RLS_FLG == true && BRK_PRS < 6) {    // Test high brakepressure vs brakelight signal fix
    MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);     // Set PWM duty cycle to 0%      // Is this needed to do again?

    digitalWrite(PB13, LOW);              // Set BLS LOW side OFF
    digitalWrite(PB12, LOW);              // Set BLS HIGH side OFF
                      
    //PORTC = DSC_LOW;                      // Set DSC relays LOW
    digitalWrite(PB0, LOW);               // Set DSC relays LOW
    digitalWrite(PB1, LOW);
    RLS_FLG = false;
    BRK_FLG = false;
    BRK_ST_CNT = 0;                       // This is needed BC otherwise BRK_ST -> BRK_ST_OP might be triggered too early
    #ifdef DEBUG
    Serial.println("Not braking");
    #endif
  }

  // This is when we want to start braking
  if (BRK_CMD < -50 && BRK_FLG == false) {
    //PORTC = DSC_HIGH;                     // Set DSC relays HIGH
    digitalWrite(PB0, HIGH);
    digitalWrite(PB1, HIGH);

    digitalWrite(PB13, HIGH);                     // Set BLS LOW side ON
    digitalWrite(PB12, HIGH);                    // Set BLS HIGH side ON

    digitalWrite(PA7, HIGH);                      // 12V to PUMP relay HIGH == connect
    BRK_FLG = true;
    #ifdef DEBUG
    Serial.println("Braking");
    #endif
  }

  //if (BRK_FLG == true && RLS_FLG == false){   // This is done this way BC then upper func wont be excecuted in every loop, only this
  if (BRK_FLG == true && RLS_FLG == false && errcounter == 0){   // This is done this way BC so that the PWM wont be done at every cycle loop but every 10 ms (change of 0x343)
    PWM = multiMap(BRK_CMD, in, out, 11);
    MyTim->setCaptureCompare(channel, PWM, PERCENT_COMPARE_FORMAT);
  }


// ************************************* SEND CAN STUFF *****************************************

  // This is emulating TOYOTA cruise controller messages sent to OP, TOYOTA CC is used BC of legacy reasons :)
  CAN_msg_t CAN_TX_msg;
  
  // Send 0x1D2 msg PCM_CRUISE
  if (currentMillis - previousMillis1 >= INTERVAL) {
    // Save the last time you send message
    previousMillis1 = currentMillis;

    CAN_TX_msg.type = DATA_FRAME;

    CAN_TX_msg.id  = 0x1D2;
    CAN_TX_msg.len = 8;
    CAN_TX_msg.data[0] = (CC_ST_OP << 5) & 0x20;           // The wanted state of the OP cruise control (CC_ST_OP)
    CAN_TX_msg.data[1] = 0x0;
    CAN_TX_msg.data[2] = 0x0;
    CAN_TX_msg.data[3] = 0x0;
    CAN_TX_msg.data[4] = BRK_ST_OP;                        // Brake pedal state sent to OP (BRK_ST_OP)
    CAN_TX_msg.data[5] = 0x0;
    CAN_TX_msg.data[6] = (CC_ST_OP << 7) & 0x80;           // The wanted state of the OP cruise control (CC_ST_OP)
    CAN_TX_msg.data[7] = can_cksum(CAN_TX_msg.data, 7, 0x1D2);
    CANSend(&CAN_TX_msg);
  }

  // Send 0x1D3 msg PCM_CRUISE_2
  if (currentMillis - previousMillis2 >= INTERVAL) {
    // Save the last time you send message
    previousMillis2 = currentMillis;

    CAN_TX_msg.id  = 0x1D3;
    CAN_TX_msg.len = 8;
    CAN_TX_msg.data[0] = PWM;                                // Debugging msg, PWM value
    CAN_TX_msg.data[1] = 0xA0;                               // 0xA0 is for setting the pre-enable state of the TOYOTA cruise control (MAIN ON and LOW_SPEED_LOCKDOWN = 1)
    CAN_TX_msg.data[2] = set_speed;                          // Speed value sent to OP to set the cruise controller set speed
    CAN_TX_msg.data[3] = (BLTS_volt >> 8);                   // Debugging msg, BLTS wire voltage in milliVolts
    CAN_TX_msg.data[4] = BLTS_volt;                          // Debugging msg, BLTS wire voltage in milliVolts
    CAN_TX_msg.data[5] = (DSC_volt >> 8);                     // Debugging msg, DSC+ wire voltage in milliVolts
    CAN_TX_msg.data[6] = DSC_volt;                            // Debugging msg, DSC+ wire voltage in milliVolts
    CAN_TX_msg.data[7] = can_cksum(CAN_TX_msg.data, 7, 0x1D3);
    CANSend(&CAN_TX_msg);
  }


// ********************************** DEBUGGING ***************************************** 

  // Do this every 1 sec so it wont bother other stuff so much
  if ((currentMillis - dallasTyme) > 1000) {
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
  if (whole > 45 && !FAN_ON) {
    digitalWrite(PB14, HIGH);                 // Turn FAN pin HIGH when FET temp over 45 degrees
    FAN_ON = 1;
  }
  if (whole < 45 && FAN_ON) {
    digitalWrite(PB14, LOW);;                // Turn FAN pin LOW when FET temp under 45 degrees
    FAN_ON = 0;
  }
  #endif

  if (whole > 80) {               // If FET temp over 80 degrees disable OP
    CC_ST_OP == false;
    CNL_REQ = 1;
  }
  
  if (errcounter > 65000) {      // errcounter func is triggered every (65000 x ~9us =) 585 ms if no 0x343 msg is detected which should arrive every 10 ms
    CC_ST_OP == false;
    CNL_REQ = 1;
    errcounter = 0;
    MyTim->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
    digitalWrite(PA7, LOW);                      // 12V to PUMP relay HIGH == connect
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


// TOYOTA CAN CHECKSUM
// I think this is written by wocsor, same as the TOYOTA cruise CAN send stuff
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int i = 0; i < len; i++) {
    checksum += (dat[i]);
  }
  return checksum;
}


// Interpolation shit
// This is direct copy from https://github.com/RobTillaart/MultiMap
// note: the _in array should have increasing values
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}


// Serial read function for DEBUGGING
void readSerial()
{
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    uint16_t input = Serial.parseInt();

    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      Serial.print("Input is: ");
      Serial.println(input);

      if (input > 1 && input < 101) {
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
