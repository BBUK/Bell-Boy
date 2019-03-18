#include "LowPower.h"
#include <Wire.h>
#include <EEPROM.h>

#define piPowerEnablePin 2  // output - enables power to the Pi
#define batteryChargeEnablePin 3 // output enables power to the battery charger
#define piRunPin 4 // output - forces pi not to boot when powered
#define piPowerDownPin 5 // input detects whether Pi is shut down (LOW=UP HIGH=POWERED_DOWN
#define vSwitchDetectPin 6 //input detects status of power button
#define chargeStatusPin 7 // input detects status of battery charge from MCP73831 must be pulled up

uint8_t debounce = 0 ; // debounce count for power button
uint8_t buttonStatus = 0;
#define PRESS 1
#define CHANGED 2

uint8_t promData[5];

volatile uint8_t flags = 0;
#define RECIEVEDBOOTUP 1
#define SENDLOWBATT 2
#define WRITETOPROM 4 

uint8_t loopCount;
volatile float volts = 0;
volatile uint8_t i2cRegister = 0;
volatile char i2cbuffer[12];

// STATE 0 = unknown power source, Pi run pin = low
// STATE 1 = external power source, Pi run pin = low, battery charging
// STATE 2 = external power source, Pi run pin = low, battery charge complete
// STATE 3 = on battery power battery not measured
// STATE 4 = on battery power, battery OK, start booting (flashing red)
// STATE 5 = on battery power, battery OK, Pi booting (flashing green)
// STATE 6 = on battery power, Pi booted (solid green)
// STATE 7 = ordered shutdown mode
// STATE 99 = shut down
volatile uint8_t STATE = 0;

void setup() {
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  ledOff();

  pinMode(piPowerEnablePin, OUTPUT);
  digitalWrite(piPowerEnablePin, LOW);

  pinMode(batteryChargeEnablePin, OUTPUT);
  digitalWrite(batteryChargeEnablePin, LOW);

  pinMode(piRunPin, OUTPUT);
  digitalWrite(piRunPin, LOW); // stop pi running
  Wire.begin(16);
  Wire.setClock(100000);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cPushData);
  ADMUX = _BV(REFS0); // reference to VCC (PC0/ADC0 being measured)
  delay(10);
  volts = measureVcc();
  if (digitalRead(vSwitchDetectPin)) { buttonStatus = PRESS; debounce = 4;} // stops a long switch on hold switching the thing off immediately
}

void loop() {
  LowPower.idle(SLEEP_500MS, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_ON);
  static uint8_t count = 0;
  if (!(loopCount % 8)) {
    volts = ((3.0 * volts) + measureVcc())/4.0;
  }
  loopCount += 1;
  
  if (digitalRead(vSwitchDetectPin)) {
    if (debounce != 5) debounce += 1;
  } else {
    if (debounce != 0) debounce -= 1;
  }
  if ((debounce >= 3) && !(buttonStatus & PRESS)) buttonStatus = PRESS | CHANGED; // was off now on, so switch on and flag (bit 2)
  if ((debounce < 2 ) && (buttonStatus & PRESS)) buttonStatus = CHANGED; // was on now off, so switch on and flag (bit 2)

  if (buttonStatus & CHANGED) {
    buttonStatus &= ~CHANGED;  // switch off change flag
    if (buttonStatus & PRESS) {
      if(STATE == 1 || STATE==2 || STATE == 4 || STATE == 5 || STATE == 7) STATE = 99;  // power button forces shutdown
      if(STATE == 6){ // go into order shutdown mode
        flags |= SENDLOWBATT;
        count = 0;
        STATE = 7;
      }
    }
  }
  if(flags & WRITETOPROM) writeFloat();

// (1.65/4.7)*1024 = 359 = connected to external power
// (1.65/3.9)*1024 = 433 = internal power battery full
// (1.65/3.2)*1024 = 528 = internal power battery low
// (1.65/3.1)*1024 = 545 = internal power battery empty
//                    0 = pi not powered

  
  switch (STATE) {
    case 0: // startUp
      ledRedOn();
      if (volts < 380 && volts > 200) { // must be on external power
        digitalWrite(batteryChargeEnablePin, HIGH); // enable charging
        pinMode(piRunPin, OUTPUT);
        digitalWrite(piRunPin, LOW); // stop Pi running
        digitalWrite(piPowerEnablePin, HIGH); // need this to ensure arduino will continue to run
        STATE = 1;
      } else {
        digitalWrite(batteryChargeEnablePin, LOW); // disable charging
        digitalWrite(piPowerEnablePin, HIGH); // on battery power, and switch on, send power to Pi
        pinMode(piRunPin, OUTPUT);
        digitalWrite(piRunPin, LOW); // don't allow Pi to run just yet
        delay(100);
        volts=measureVcc(); // remeasure voltage state now Pi is powered
        STATE = 3;
      }
      break;
    case 1: // external power, battery charging
      if (volts > 380) {
        STATE = 99;  // power removed
        break;
      }
      if (digitalRead(chargeStatusPin) == HIGH) {
        STATE = 2;  // charge complete
        digitalWrite(batteryChargeEnablePin, LOW);
        break;
      }
//      if(flags & RECIEVEDBOOTUP) flags |= SENDLOWBATT; // run header not installed - shut pi down
      ledRedOn();
      break;
    case 2: // external power, charge complete
      if (volts > 380) {
        STATE = 99;  // power removed go into shutdown mode
        break;
      }
//      if(flags & RECIEVEDBOOTUP) flags |= SENDLOWBATT; // run header not installed - shut pi down
      ledGreenOn();
      break;
    case 3: // battery power,   pi powered, run still low
      if (volts < 500) {
        if (!(buttonStatus & PRESS)) {
         STATE = 4;  // battery OK and button not pressed release run pin
         pinMode(piRunPin, INPUT);
        }
        break;
      } else {
        STATE = 99; // battery not OK, go into shutdown
        break;
      }
    case 4: // battery power, pi powered and running wait for Pi to start booting
      if (digitalRead(piPowerDownPin) == LOW) {
        STATE = 5;  // Pi now booting
        break;
      }
      if(!(loopCount % 2)){
        ledOff();
      } else {
        ledRedOn();
      }
      break;
    case 5: // battery power, pi powered and booting, wait for booting to finish
      if (flags & RECIEVEDBOOTUP) {
        STATE = 6;  // Pi now fully up
        break;
      }
      if(!(loopCount % 2)){
        ledOff();
      } else {
        ledGreenOn();
      }
      break;
    case 6: // steady state operation
      if (digitalRead(piPowerDownPin) == HIGH) { // Pi shut itself down
        STATE = 99;
      }
      if (volts < 380) { // power plugged in -> go into order shutdown mode
        flags |= SENDLOWBATT;
        count = 0;
        STATE = 7;
      }
      if (volts >= 540) { // battery low -> go into order shutdown mode
        flags |= SENDLOWBATT;
        count = 0;
        STATE = 7;
      }
      if(!(loopCount % 8)) {
        if (volts < 528.0) {
          ledGreenChirrup();
        } else {
          ledRedChirrup();
        }
      }
      break; 
    case 7: // ordered shutdown
      if (digitalRead(piPowerDownPin) == HIGH) { // Pi shut down
        STATE = 99;
        break;
      }
      count++;
      if (count < 150) { // flash red for 75 seconds then force shutdown
        if(!(loopCount % 2)){
          ledRedOn();
        } else {
          ledOff();
        }
        break;
      } else {
        STATE = 99;
      }
      break;
    case 99: // shutdown
      for(uint8_t j = 0; j<10; j++){
        ledRedOn();
        delay(50);
        ledOff();
        delay(50);
      }
      digitalWrite(batteryChargeEnablePin, LOW);
      pinMode(piRunPin, OUTPUT);
      digitalWrite(piRunPin, LOW);
      digitalWrite(piPowerEnablePin, LOW);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

void writeFloat(void){
	EEPROM.write(promData[0] << 2,promData[1]);
	EEPROM.write((promData[0] << 2) + 1,promData[2]);
	EEPROM.write((promData[0] << 2) + 2,promData[3]);
	EEPROM.write((promData[0] << 2) + 3,promData[4]);
	flags &= ~WRITETOPROM;
}

void i2cReceive(int number) {
  i2cRegister = Wire.read();
  if(i2cRegister == 1) flags |= RECIEVEDBOOTUP;
  if(i2cRegister >= 10 && i2cRegister <= 22 && number == 5 && !(flags & WRITETOPROM)){ // got an PROM write COMMAND
    promData[0] = i2cRegister;
	  promData[1] = Wire.read();
	  promData[2] = Wire.read();
	  promData[3] = Wire.read();
	  promData[4] = Wire.read();
	  flags |= WRITETOPROM;
  }
}

void i2cPushData() {
  uint16_t iVolts;
  if(i2cRegister == 2){ // send voltage
      iVolts = (uint16_t)(volts+0.5);
      if(flags & SENDLOWBATT) iVolts += 0x8000;
      i2cbuffer[0] = iVolts >> 8;
      i2cbuffer[1] = iVolts & 0x00FF;
      Wire.write((char *)i2cbuffer,2);
  } else if(i2cRegister >= 10 && i2cRegister <= 22){ // send EEPROM data
      i2cbuffer[0] = EEPROM.read(i2cRegister << 2);
      i2cbuffer[1] = EEPROM.read((i2cRegister << 2)+1);
      i2cbuffer[2] = EEPROM.read((i2cRegister << 2)+2);
      i2cbuffer[3] = EEPROM.read((i2cRegister << 2)+3);
      Wire.write((char *)i2cbuffer,4);
  } else if(i2cRegister == 5){ // send accBiasXYZ
	  populateBuffer(44);
      Wire.write((char *)i2cbuffer,12);
  } else if(i2cRegister == 6){ // send accScaleXYZ
	  populateBuffer(56);
      Wire.write((char *)i2cbuffer,12);
  } else if(i2cRegister == 7){ // send gyroBiasXYZ
	  populateBuffer(68);
      Wire.write((char *)i2cbuffer,12);
  } else if(i2cRegister == 8){ // send gyroScaleXYZ
	  populateBuffer(80);
      Wire.write((char *)i2cbuffer,12);
  } else {
	  Wire.write(99);
  }
}

void populateBuffer(uint8_t reg){
  i2cbuffer[0] = EEPROM.read(reg);
  i2cbuffer[1] = EEPROM.read(reg+1);
  i2cbuffer[2] = EEPROM.read(reg+2);
  i2cbuffer[3] = EEPROM.read(reg+3);
  i2cbuffer[4] = EEPROM.read(reg+4);
  i2cbuffer[5] = EEPROM.read(reg+5);
  i2cbuffer[6] = EEPROM.read(reg+6);
  i2cbuffer[7] = EEPROM.read(reg+7);
  i2cbuffer[8] = EEPROM.read(reg+8);
  i2cbuffer[9] = EEPROM.read(reg+9);
  i2cbuffer[10] = EEPROM.read(reg+10);
  i2cbuffer[11] = EEPROM.read(reg+11);
}

// (1.65/5.0)*1024 = 338 = connected to external power
// (1.65/4.2)*1024 = 402 = internal power battery full
// (1.65/3.5)*1024 = 482 = internal power battery low
// (1.65/3.4)*1024 = 496 = internal power battery empty
//                    0 = pi not powered

uint16_t measureVcc(void) {
  uint8_t low;
  //    LowPower.adcNoiseReduction(SLEEP_15MS, ADC_ON, TIMER2_ON);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  low  = ADCL;
  return (ADCH << 8) | low;
}

void ledRedOn(void) {
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
}

void ledGreenOn(void) {
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
}

void ledOff(void) {
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
}

void ledYellow(void) {
  for (uint8_t i = 0; i < 25; ++i) {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    delay(10);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    delay(5);
  }
}

void ledRedChirrup(void) {
  ledRedOn();
  LowPower.idle(SLEEP_30MS, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_ON);
  ledOff();
}

void ledGreenChirrup(void) {
  ledGreenOn();
  LowPower.idle(SLEEP_30MS, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_ON);
  ledOff();
}