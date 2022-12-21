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