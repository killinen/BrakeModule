
/*
  Software for BrakeModule v0.2

  The Main purpose of the module is to control charge pump so that the car will brake wo any errors on the car side and be an CC for OP so that it can controlled from the steering wheel buttons.
  The sub functionalities are:
  - Read CAN data
  - Send CAN data
  - Disconnect DSC module from charge pump
  - Control charge pump with relay and PWR MOSFET
  - Measure PWR FETs temp and cool it w FAN
  - Control cars brake light signal with relay (or N-channel FET) and P-channel FET
  - Act as an Toyota cruise controller so OP can be engaged from the steering wheel buttons

  TODO:
  - Clean readSerial() stuff
*/

#include <OneWire.h>
#include <mcp2515.h>

#define FAN_CTRL                          // If defined use FAN for PWR MOSFET temperature control
#define DEBUG                             // If defined use serial for debugging
#define CAN_FILTER                        // If defined use predefined CAN filters for wanted msgs
#define B_MOSFET                          // If defined use N-channel MOSFET @ HW to control brakelight swithes LOW side
//#define B_RELAY                           // If defined use relay @ HW to control brakelight swithes LOW side      

#ifdef DEBUG
  #include <LibPrintf.h>
#endif

#define dallaspin 7                       // DS18B20 connected to pin D7
#define adcPin 0                          // DSC+ line is readed at A0 pin

#define CHG_A2 B00000100;                 // Debug: FLip A2 pin register bit
#define CHG_A3 B00001000;                 // Debug: FLip A3 pin register bit
#define CHG_A4 B00010000;                 // Debug: FLip A4 pin register bit
#define CHG_A5 B00100000;                 // Debug: FLip A5 pin register bit
#define CHG_A6 B01000000;                 // Debug: FLip A6 pin register bit
#define CHG_A7 B10000000;                 // Debug: FLip A7 pin register bit
#define FAN    B01000000;                 // FAN driver pin D6 register bit

#define DSC_LOW   (PORTC | B00110000)     // Set pins A4 and A5 HIGH (DSC relays OFF)
#define DSC_HIGH  (PORTC & ~B00110000)    // Set pins A4 and A5 LOW (DSC relays ON)

#ifdef B_RELAY
#define S_BLS_OFF   PORTC | B00000100     // Set pin A2 HIGH == Brakepedal not pressed (BrakeLightSwith LOW side closed)
#define S_BLS_ON    PORTC & B11111011     // Set pins A2 LOW == Brakepedal pressed (BrakeLightSwith LOW side open)
#define S_BLTS_OFF  PORTD & B11011111     // Set pin D5 LOW == Brakepedal not pressed  (BrakeLightSwith HIGH side not connected to 12V by BM)
#define S_BLTS_ON   PORTD | B00100000     // Set pin D5 HIGH == Brakepedal pressed (BrakeLightSwith HIGH side connected to 12V)
#endif

#ifdef B_MOSFET
#define BLS_LOW   (PORTD | B00010000) & B11011111   // This sets D4 HIGH and D5 LOW, so the BLS LOW side is conductive and HIGH side non-conductive == brakepedal not pressed
#define BLS_HIGH  (PORTD | B00100000) & B11101111   // This sets D4 LOW and D5 HIGH, so the BLS LOW side is non-conductive and HIGH side conductive == brakepedal pressed
#endif

#define BP_LOW    (PORTC | B00001000)     // Set pin A3 HIGH == disconnect chargepump from 12V
#define BP_HIGH   (PORTC & ~B00001000)    // Set pin A3 LOW == connect chargepump to 12V

#define RAMP_VALUE  60                    // Interval time between multiple button presses detections in milliseconds
#define INTERVAL    30                    // Interval time at which to send 0x1D2 and 0x1D3 messages (milliseconds)
#define UP          32                    // This is UP button press value for button press byte (BTN_CMD)
#define DWN         64                    // This is DOWN button press value for button press byte (BTN_CMD)
#define RSM         96                    // This is RESUME button press value for button press byte (BTN_CMD)

struct can_frame canMsg;                  // CAN frame values from CAN read
struct can_frame canMsg1;                 // CAN frame values to sent 0x1D2
struct can_frame canMsg2;                 // CAN frame values to sent 0x1D3

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
bool dallas_flip = 1;                     // Flip value for DS18B20 reading

//Interpolation shit, this is done BC relationship of PWM and brake pressure is not linear

// in[] holds the read values from BRK_CMD so are the INPUT values for the interpolation for OUTPUTing the PWM values regaring of the INPUT
// note: the in array should have increasing values
int in[]  = {-1000, -900, -800, -700, -600, -500, -400, -300, -200, -100, -50};
// out[] holds the values wanted in OUTPUT regarding to INPUT in[], these are OCR1A (PWM) values that are interpolated from BRK_CMD values
int out[] = {2047, 1980, 1900, 1800, 1660, 1500, 1380, 1200, 1000, 600, 200};       

MCP2515 mcp2515(10);                      // CS pin is D10

// *******************************************************************************************************
// *********************************************** SETUP *************************************************
// *******************************************************************************************************
void setup() {

#ifdef DEBUG 
  // initialize serial:
  //Serial.begin(115200);                                 // If Arduino Nano is used
  Serial.begin(230400);                                 // I use this w LGT8F
#endif
  
  // Setup pins
  DDRB |= B00000010;                                    // Set D9 as OUTPUT
  DDRC = B00111100;                                     // Set A0 & A1 INPUT and A2, A3, A4, and A5 OUTPUT in PORTC (Analog pins on Arduino)
  DDRD |= B01110000;                                    // Set D4, D5 and D6 as OUTPUT
  PORTC |= B00111100;                                   // SET relay pins A2-5 HIGH so the relays are low 

  // If LOW side BLS MOSFET is used, this has to be done or the line will stay floating
  PORTD = BLS_HIGH;                                     // Turn ON brakelight switch      
  PORTD = BLS_LOW;                                      // Turn OFF brakelight switch

  // Setup the PWM
  TCCR1A = _BV(COM1A1) | _BV(WGM11);                    // Enable the PWM output OC1A on digital pin 9
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);         // Set fast PWM and prescaler of 1(CS10) or 8(CS11) or 256(CS12) on timer 1
  ICR1 = 2047;                                          // Set the PWM frequency to ~2 kHz: 32MHz/(8 * (2047 + 1)) = 1.953 kHz
  OCR1A = 0;

  // Setup ADC
  ADCSRA =  bit (ADEN);                                 // Turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);   // Prescaler of 128
  ADMUX =   bit (REFS0) | (adcPin & 0x07);              // AVcc(?) and set ADC reading to adcPin

  // Setup CAN
  mcp2515.reset();                                      // Reset CAN module counfiguration, maybe unnessaccary
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);            // Use of CAN bus bitrate and CAN module crystal oscillation frequency
  mcp2515.setNormalMode();                              // This is unnessary when filtering is done

  #ifdef CAN_FILTER                                     // Set the MCP2515 filtering for the used CAN msgs. How masks&filters work: https://forum.arduino.cc/t/filtering-and-masking-in-can-bus/586068/3
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);  // Set all 11 bits masked in MASK0 and MASK0
  mcp2515.setFilter(MCP2515::RXF0, false, 0x153);       // Filter (RXF0-5) these messages so other wont get through BC they are not needed, if one needs other msgs you have to modify or disable this
  mcp2515.setFilter(MCP2515::RXF1, false, 0x329);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);  
  mcp2515.setFilter(MCP2515::RXF2, false, 0x153);       
  mcp2515.setFilter(MCP2515::RXF3, false, 0x329);
  mcp2515.setFilter(MCP2515::RXF4, false, 0x343);
  mcp2515.setFilter(MCP2515::RXF5, false, 0x545);
  mcp2515.setNormalMode();
  #endif

  // Start first conversion of temperature of the DS18B20
  OneWire ds(dallaspin);
  ds.reset();
  ds.write(0xCC);                                       // Skip command
  ds.write(0x44, 1);                                    // Start conversion and leave power applied to the 1 wire bus. 
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
  
  // ADC reading is done this way BC it is quicker that waiting for the conversion to happen
  if (!ADC_FLG) {                                         // If ADC conversion is not happening
    bitSet (ADCSRA, ADSC);                                // Start a conversion
    ADC_FLG = true;                                       // Conversion is started so set working flag to true 
  }

  // the ADC clears the bit when done
  if (bit_is_clear(ADCSRA, ADSC)) {
    DSC_volt = (ADC * 3.6);                               // Read ADC value and multiply by 3.6, this comes from approximation of 12 bit ADC and 10k/4k7 voltage divider
    ADC_FLG = false;                                      // Conversion is done so set ADC flag to false 
  }


// ************************************* READ CAN STUFF ******************************************

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x153) {                         // Read speed from 0x153 and do some conversion
      car_speed = (((canMsg.data[2] << 3) + (canMsg.data[1] >> 5)) * 0.25) - 2.64;
    }
    if (canMsg.can_id == 0x329) {
      BTN_CMD = (canMsg.data[3] & B01100000);             // &B01100000 is to make sure that other bits in the byte don't bother
      BRK_ST = (canMsg.data[6] & B00000001);              // Read brake pedal switch state
      BRK_ST_CNT++;                                       // Use this for not triggering BRK_ST_OP too early after OP has stopped braking
    }
    if (canMsg.can_id == 0x343) {
      BRK_CMD = (canMsg.data[0] << 8 | canMsg.data[1]);   // Read brake request value from OP
      CNL_REQ = (canMsg.data[3] & B00000001);             // Read cancel request flag from OP
      errcounter = 0;                                     // If 0x343 read, set error counter to 0
    }
    if (canMsg.can_id == 0x545) {
      OCC = (canMsg.data[0] & B00001000);                 // If OCC == 0x08 BMW original cruise is pre-enabled (cruise light is ON) and OP should not engage!
    }
    #ifdef DEBUG
    CAN_count++;
    #endif
  }


// ******************************** OP LOGIK STUFF **********************************************

  if (BTN_CMD == RSM && BTN_PRS_FLG == false) {                // Engage/disengage OP if resume button press is detected, BTN_PRS_FLG prevents multiple button presses
    if (CC_ST_OP == false) {
      set_speed = car_speed;                                   // When OP CC activation is set, save car_speed to set_speed
      CNL_REQ_CNT = 0;
      #ifdef DEBUG
      Serial.println("Speed setted");
      #endif
    }
    CC_ST_OP = !CC_ST_OP;
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

//  Option 2: Tested and worked! 
  if (BRK_ST == 1 && BRK_FLG == false) {                        // If brake pedal pressed and OP is no braking, raise trigger brake flag for OP to know after consective 5 msgs
    if (BRK_ST_CNT > 5) {                                       // If 0x153 interval is 10 ms, BRK_ST_OP it will be triggered after 60 ms (or is it 70 ms?)
      BRK_ST_OP = 1;
      BRK_ST_CNT = 0;
      #ifdef DEBUG
      Serial.println("BRAKE!");
      #endif
    }
  }
  else {
    BRK_ST_OP = 0;
    //BRK_ST_CNT = 0;                                             // Maybe if this is deleted, BRK_ST_OP is triggered faster BC counter is most probably > 5
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
    PORTC = BP_LOW;                       // 12V to PUMP relay LOW == disconnect
    //OCR1A = 0;                            // Set D9 PWM duty cycle to 0% (B- LOW)   COMMENT! MAYBE disable this, could to be better for the FLYBACKING voltage to get DOWN, do it @ releasetimer
    OCR1A = 2047;
    timerRelease = currentMillis;         // Set timer for releasing the control back to DSC after settling time (if switch is done too fast car gives DSC error)
    RLS_FLG = true;                       // Set brakingReleaseFlag to true
  }

  // If upper function was excecuted and we went back to braking territory, set BRK_FLG and RLS_FLG false so we can start braking before timerRelease (lower func) is excecuted
  if (BRK_CMD < -50 && RLS_FLG == true) {
    OCR1A = 0;
    BRK_FLG = false;
    RLS_FLG = false;
  }
 
  // 600 ms after the pump was disconnected from 12V, turn OFF brakelights and turn OFF DSC relays so that the pump is controlled by DSC-module
  if (((currentMillis - timerRelease) > 600) && RLS_FLG == true) {    // Original value 400
    OCR1A = 0;                            // Set D9 PWM duty cycle to 0%      // Is this needed to do again?
    #ifdef B_RELAY
    PORTC = S_BLS_OFF;                    // Set BLS LOW side OFF
    PORTD = S_BLTS_OFF;                   // Set BLS HIGH side OFF
    #endif
    #ifdef B_MOSFET
    PORTD = BLS_LOW;                      // Set BLS OFF
    #endif
    PORTC = DSC_LOW;                      // Set DSC relays LOW
    RLS_FLG = false;
    BRK_FLG = false;
    BRK_ST_CNT = 0;                       // This is needed BC otherwise BRK_ST -> BRK_ST_OP might be triggered too early
    #ifdef DEBUG
    Serial.println("Not braking");
    #endif
  }

  // This is when we want to start braking
  if (BRK_CMD < -50 && BRK_FLG == false) {
    PORTC = DSC_HIGH;                     // Set DSC relays HIGH
    #ifdef B_RELAY
    PORTC = S_BLS_ON;                     // Set BLS LOW side ON
    PORTD = S_BLTS_ON;                    // Set BLS HIGH side ON
    #endif
    #ifdef B_MOSFET
    PORTD = BLS_HIGH;                     // Set BLS ON
    #endif
    PORTC = BP_HIGH;                      // 12V to PUMP relay HIGH == connect
    BRK_FLG = true;
    #ifdef DEBUG
    Serial.println("Braking");
    #endif
  }

  //if (BRK_FLG == true && RLS_FLG == false){   // This is done this way BC then upper func wont be excecuted in every loop, only this
  if (BRK_FLG == true && RLS_FLG == false && errcounter == 0){   // This is done this way BC so that the PWM wont be done at every cycle loop but every 10 ms (change of 0x343)
    OCR1A = multiMap(BRK_CMD, in, out, 11);
  }


// ************************************* SEND CAN STUFF *****************************************

  // This is emulating TOYOTA cruise controller messages sent to OP, TOYOTA CC is used BC of legacy reasons :)
  // Send 0x1D2 msg PCM_CRUISE
  if (currentMillis - previousMillis1 >= INTERVAL) {
    // Save the last time you send message
    previousMillis1 = currentMillis;

    canMsg1.can_id  = 0x1D2;
    canMsg1.can_dlc = 8;
    canMsg1.data[0] = (CC_ST_OP << 5) & 0x20;           // The wanted state of the OP cruise control (CC_ST_OP)
    canMsg1.data[1] = 0x0;
    canMsg1.data[2] = 0x0;
    canMsg1.data[3] = 0x0;
    canMsg1.data[4] = BRK_ST_OP;                        // Brake pedal state sent to OP (BRK_ST_OP)
    canMsg1.data[5] = 0x0;
    canMsg1.data[6] = (CC_ST_OP << 7) & 0x80;           // The wanted state of the OP cruise control (CC_ST_OP)
    canMsg1.data[7] = can_cksum(canMsg1.data, 7, 0x1D2);
    mcp2515.sendMessage(&canMsg1);
  }

  // Send 0x1D3 msg PCM_CRUISE_2
  if (currentMillis - previousMillis2 >= INTERVAL) {
    // Save the last time you send message
    previousMillis2 = currentMillis;

    canMsg2.can_id  = 0x1D3;
    canMsg2.can_dlc = 8;
    canMsg2.data[0] = whole;                              // Debugging msg, FET temperature
    canMsg2.data[1] = 0xA0;                               // 0xA0 is for setting the pre-enable state of the TOYOTA cruise control (MAIN ON and LOW_SPEED_LOCKDOWN = 1)
    canMsg2.data[2] = set_speed;                          // Speed value sent to OP to set the cruise controller set speed
    canMsg2.data[3] = (OCR1A >> 8);                       // Debugging msg, PWM duty cycle
    canMsg2.data[4] = OCR1A;                              // Debugging msg, PWM duty cycle
    canMsg2.data[5] = (DSC_volt >> 8);                    // Debugging msg, DSC+ wire voltage in milliVolts
    canMsg2.data[6] = DSC_volt;                           // Debugging msg, DSC+ wire voltage in milliVolts
    canMsg2.data[7] = can_cksum(canMsg2.data, 7, 0x1D3);
    mcp2515.sendMessage(&canMsg2);
  }


// ******************************* DS18B20 READ and some DEBUGGING ***************************************** 

  // Even w optimized code DS18B20 is slow to commmunicate, do this every 5 sec so it wont bother other stuff so much
  if ((currentMillis - dallasTyme) > 5000) {
    //uint16_t chuck = millis();
    int16_t result;

    if (dallas_flip == 1) {
      OneWire ds(dallaspin);
      byte i;
      byte data[2];
      ds.reset();
      ds.write(0xCC);       // Skip command
      ds.write(0xBE);       // Read 1st 2 bytes of Scratchpad
      for ( i = 0; i < 2; i++) data[i] = ds.read();
      result = (data[1]<<8) | data[0];
      if (data[0]&8) ++result;
      dallas_flip = 0;
    }
    else {
      OneWire ds(dallaspin);
      ds.reset();
      ds.write(0xCC);       //Skip command
      ds.write(0x44, 1);    // Start conversion and leave power applied to the 1 wire bus.  
      dallas_flip = 1;  
    }
    uint8_t dec = (result & 0b00001111) * 625;
    whole =  result>>=4;
    //uint8_t diff = millis() - chuck;
    #ifdef DEBUG
    if ( dallas_flip == 0 ) {
      Serial.print("DS18B20 Temperatture is: ");
      Serial.print(whole);
      Serial.print(".");
      Serial.println(dec);
    }
    printf("BRK_FLG was: %u and RLS_FLG was %u and CC_ST_OP %u and CNL_REQ %u AND BRK_ST_OP %u\n", BRK_FLG, RLS_FLG, CC_ST_OP, CNL_REQ, BRK_ST_OP); 
    #endif
    
    dallasTyme = millis();
  }

  
// *********************************** FAN CONTROL **************************************

  #ifdef FAN_CTRL
  if (whole > 45 && !FAN_ON) {
    PORTD |= FAN;                 // Turn FAN pin HIGH when FET temp over 45 degrees
    FAN_ON = 1;
  }
  if (whole < 45 && FAN_ON) {
    PORTD &= ~FAN;                // Turn FAN pin LOW when FET temp under 45 degrees
    FAN_ON = 0;
  }
  #endif

  if (whole > 80) {               // If FET temp over 80 degrees disable OP
    CC_ST_OP == false;
    CNL_REQ = 1;
  }
  
  if (errcounter > 65000) {      // errcounter func is triggered every (65000 x ~15us =) 975 ms if no 0x343 msg is detected which should arrive every 10 ms
    CC_ST_OP == false;
    CNL_REQ = 1;
    errcounter = 0;
  }
  
  errcounter++;                   // Add errcounter value, this value is set to zero allways when 0x343 is detected
  counter++;                      // Add counter value, this is for counting loops for measuring average looptyme
  //PORTC ^= CHG_A5;
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

      if (input == 69) {
        uint16_t chuck = millis();
        
        #ifdef  DALLAS
        //ds.selectNext();
        //int16_t measure = dallas(dallaspin, 0);
        int16_t measure = dallas(dallaspin, 0);
        //uint16_t dec = measure<<12;
        //dec = dec >> 12;
        //dec = dec * 625;
        uint16_t dec = (measure & 0b00001111) * 625;
        int16_t whole =  measure>>=4;
        Serial.print("DS18B20 Temperatture is: ");
        //Serial.println(dallas(dallaspin, 0));
        Serial.print(whole);
        Serial.print(".");
        //Serial.println(dallas(dallaspin, 0));
        Serial.println(dec);
        uint16_t diff = millis() - chuck;
        Serial.print("Execution time for DS18B20 reading was in millis: ");
        Serial.println(diff);
        #endif
        //Serial.print("ADC value is: ");
        //Serial.println(adc1);
        //printf("ADC value is: %i\n", adc1);
        Serial.print("CAN count was: ");
        Serial.println(CAN_count);
        //printf("CAN count was %lu\n", CAN_count);
        CAN_count = 0;
      }
      else if (input == 42) {
      //if (input == 42) {
        PORTC ^= CHG_A2;
        //printf("AnalogPin %i in now %i\n", input, PORTC >> 2 & 1);
        //digitalWrite(A2, !digitalRead(A2));
        Serial.print("AnalogPin ");
        Serial.print(input);
        Serial.print(" is now ");
        Serial.println(digitalRead(A2));
      }
       else if (input == 43) {
        PORTC ^= CHG_A3;
        printf("AnalogPin %i in now %i\n", input, PORTC >> 3 & 1);
      }
       else if (input == 44) {
        PORTC ^= CHG_A4;
        printf("AnalogPin %i in now %i\n", input, PORTC >> 4 & 1);
      }
        else if (input == 45) {
        PORTC ^= CHG_A5;
        printf("AnalogPin %i in now %i\n", input, PORTC >> 5 & 1);
      }
       else if (input == 46) {
        PORTC ^= CHG_A6;
        printf("AnalogPin %i in now %i\n", input, digitalRead(A6));
      }
       else if (input == 47) {
        PORTC ^= CHG_A7;
        printf("AnalogPin %i in now %i\n", input, digitalRead(A7));
      }
      else if (input > 50) {
          OCR1A = input;
          Serial.println("Strest testing BModule..");
      }
      else if (input == 0) {
        OCR1A = input;
        printf("Set PWM zero");
      }
      else if (input == 1) {
        PORTC = DSC_LOW;          // Set DSC relays LOW
      }
      else if (input == 2) {
        PORTC = DSC_HIGH;          // Set DSC relays HIGH
      }
      else if (input == 3) {
        PORTD = BLS_LOW;            // Set brakelight switch OFF
        //PORTC = BLSL_LOW;         // Set BLS relay/FET LOW
        //PORTD = BLSH_LOW;
        //PORTC ^= CHG_A2;
      }
      else if (input == 4) {
        PORTD = BLS_HIGH;             // Brakelight switch ON
        //PORTC = BLSL_HIGH;          // Set BLS relay/FET HIGH
        //PORTD = BLSH_HIGH;
        //PORTC ^= CHG_A2;
      }
      else if (input == 5) {
        PORTC = BP_LOW;          // Set B+ relay LOW
      }
      else if (input == 6) {
        PORTC = BP_HIGH;          // Set B+ relay HIGH
      }
      else if (input == 7) {
        uint8_t r;
        r = mcp2515.getInterrupts();
        Serial.print("getInterrupts() (MCP_CANINTF) = ");
        Serial.println(r, BIN);
      }
      else if (input == 8) {
        uint8_t r;
        mcp2515.clearInterrupts();
        Serial.print("clearInterrupts() = interrupts cleared ");
        Serial.println("(MCP_CANINTF)");
      }
      else if (input == 9) {
        uint8_t r;
        r = mcp2515.getInterruptMask();
        Serial.print("getInterruptMask() (MCP_CANINTE) = ");
        Serial.println(r, BIN);
        SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
        PORTB &= B11111011;
        //SPI.transfer(INSTRUCTION_WRITE);
        SPI.transfer(0x02);
        //SPI.transfer(MCP_CANINTE);
        SPI.transfer(0x2B);
        //SPI.transfer(CANINTF_RX0IF | CANINTF_RX1IF);
        SPI.transfer(B00000011);
        PORTB |= B00000100;
        SPI.endTransaction();
        //uint8_t r;
        r = mcp2515.getInterruptMask();
        Serial.print("getInterruptMask() (MCP_CANINTE) = ");
        Serial.println(r, BIN);
      }
      else {
      // fade the red, green, and blue legs of the LED:
        digitalWrite(input, !digitalRead(input));
        // print the three numbers in one string as hexadecimal:
        printf("Pin %i is now %s\n", input, digitalRead(input) ? "ON" : "OFF");
      }
    }
  }
}
