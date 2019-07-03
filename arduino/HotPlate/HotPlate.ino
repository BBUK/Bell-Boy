#include <SPI.h>
#include <PID_v1.h>

const PROGMEM uint8_t WIDTH[] = {
  0x02, 0x03, 0x04, 0x05,
  0x05, 0x05, 0x05, 0x03,
  0x04, 0x03, 0x05, 0x05,
  0x03, 0x05, 0x03, 0x05,

  0x05, 0x04, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x03, 0x03,
  0x05, 0x05, 0x04, 0x05,

  0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05,
  0x05, 0x03, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05,

  0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x03,
  0x05, 0x03, 0x05, 0x05,

  0x03, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05,
  0x05, 0x03, 0x04, 0x04,
  0x03, 0x05, 0x05, 0x05,

  0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x03,
  0x03, 0x03, 0x05, 0x05
};

const PROGMEM uint8_t FONT[] = {
  0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x5F, 0x00, 0x00, 0x00,  0x00, 0x07, 0x00, 0x07, 0x00,  0x14, 0x7F, 0x14, 0x7F, 0x14,
  0x24, 0x2A, 0x7F, 0x2A, 0x12,  0x23, 0x13, 0x08, 0x64, 0x62,  0x36, 0x49, 0x55, 0x22, 0x50,  0x00, 0x05, 0x03, 0x00, 0x00,
  0x1C, 0x22, 0x41, 0x00, 0x00,  0x41, 0x22, 0x1C, 0x00, 0x00,  0x08, 0x2A, 0x1C, 0x2A, 0x08,  0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x50, 0x30, 0x00, 0x00,  0x08, 0x08, 0x08, 0x08, 0x08,  0x00, 0x60, 0x60, 0x00, 0x00,  0x20, 0x10, 0x08, 0x04, 0x02,

  0x3E, 0x51, 0x49, 0x45, 0x3E,  0x00, 0x42, 0x7F, 0x40, 0x00,  0x42, 0x61, 0x51, 0x49, 0x46,  0x21, 0x41, 0x45, 0x4B, 0x31,
  0x18, 0x14, 0x12, 0x7F, 0x10,  0x27, 0x45, 0x45, 0x45, 0x39,  0x3C, 0x4A, 0x49, 0x49, 0x30,  0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36,  0x06, 0x49, 0x49, 0x29, 0x1E,  0x00, 0x36, 0x36, 0x00, 0x00,  0x00, 0x56, 0x36, 0x00, 0x00,
  0x00, 0x08, 0x14, 0x22, 0x41,  0x14, 0x14, 0x14, 0x14, 0x14,  0x41, 0x22, 0x14, 0x08, 0x00,  0x02, 0x01, 0x51, 0x09, 0x06,

  0x32, 0x49, 0x79, 0x41, 0x3E,  0x7E, 0x11, 0x11, 0x11, 0x7E,  0x7F, 0x49, 0x49, 0x49, 0x36,  0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C,  0x7F, 0x49, 0x49, 0x49, 0x41,  0x7F, 0x09, 0x09, 0x01, 0x01,  0x3E, 0x41, 0x41, 0x51, 0x32,
  0x7F, 0x08, 0x08, 0x08, 0x7F,  0x41, 0x7F, 0x41, 0x00, 0x00,  0x20, 0x40, 0x41, 0x3F, 0x01,  0x7F, 0x08, 0x14, 0x22, 0x41,
  0x7F, 0x40, 0x40, 0x40, 0x40,  0x7F, 0x02, 0x04, 0x02, 0x7F,  0x7F, 0x04, 0x08, 0x10, 0x7F,  0x3E, 0x41, 0x41, 0x41, 0x3E,

  0x7F, 0x09, 0x09, 0x09, 0x06,  0x3E, 0x41, 0x51, 0x21, 0x5E,  0x7F, 0x09, 0x19, 0x29, 0x46,  0x46, 0x49, 0x49, 0x49, 0x31,
  0x01, 0x01, 0x7F, 0x01, 0x01,  0x3F, 0x40, 0x40, 0x40, 0x3F,  0x1F, 0x20, 0x40, 0x20, 0x1F,  0x7F, 0x20, 0x18, 0x20, 0x7F,
  0x63, 0x14, 0x08, 0x14, 0x63,  0x03, 0x04, 0x78, 0x04, 0x03,  0x61, 0x51, 0x49, 0x45, 0x43,  0x7F, 0x41, 0x41, 0x00, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20,  0x41, 0x41, 0x7F, 0x00, 0x00,  0x04, 0x02, 0x01, 0x02, 0x04,  0x40, 0x40, 0x40, 0x40, 0x40,

  0x01, 0x02, 0x04, 0x00, 0x00,  0x20, 0x54, 0x54, 0x54, 0x78,  0x7F, 0x48, 0x44, 0x44, 0x38,  0x38, 0x44, 0x44, 0x44, 0x20,
  0x38, 0x44, 0x44, 0x48, 0x7F,  0x38, 0x54, 0x54, 0x54, 0x18,  0x08, 0x7E, 0x09, 0x01, 0x02,  0x08, 0x14, 0x54, 0x54, 0x3C,
  0x7F, 0x08, 0x04, 0x04, 0x78,  0x44, 0x7D, 0x40, 0x00, 0x00,  0x20, 0x40, 0x44, 0x3D, 0x00,  0x7F, 0x10, 0x28, 0x44, 0x00,
  0x41, 0x7F, 0x40, 0x00, 0x00,  0x7C, 0x04, 0x18, 0x04, 0x78,  0x7C, 0x08, 0x04, 0x04, 0x78,  0x38, 0x44, 0x44, 0x44, 0x38,

  0x7C, 0x14, 0x14, 0x14, 0x08,  0x08, 0x14, 0x14, 0x18, 0x7C,  0x7C, 0x08, 0x04, 0x04, 0x08,  0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20,  0x3C, 0x40, 0x40, 0x20, 0x7C,  0x1C, 0x20, 0x40, 0x20, 0x1C,  0x3C, 0x40, 0x30, 0x40, 0x3C,
  0x44, 0x28, 0x10, 0x28, 0x44,  0x0C, 0x50, 0x50, 0x50, 0x3C,  0x44, 0x64, 0x54, 0x4C, 0x44,  0x08, 0x36, 0x41, 0x00, 0x00,
  0x00, 0x7F, 0x00, 0x00, 0x00,  0x41, 0x36, 0x08, 0x00, 0x00,  0x08, 0x08, 0x2A, 0x1C, 0x08,  0x08, 0x1C, 0x2A, 0x08, 0x08
};

uint8_t t = 0, s = 0, ti, u; // temporary globals
#define DISPLAYWIDTH 102

#define CHIPSELECT 8
#define COMMANDDATA 9
#define PLATE 4
#define COMMAND LOW
#define DATA HIGH

#define LOOPTIME 40

uint8_t buttons[4][3] = {{5, 0, 0}, {7, 0, 0}, {3, 0, 0}, {2, 0, 0} }; // bit 7 = on/off bit 6=changed flag bits 0-2 = debounce on moving average basis
#define PINS 0
#define ONOFF 1
#define DEBOUNCE 2
#define PRESS 1
#define CHANGED 2
#define UP 0
#define DOWN 2
#define LEFT 3
#define RIGHT 1

#define MAXBRIGHT 140
uint8_t backlight = 255;

#define NORMAL 0x00
#define REVERSED 0x01 // 0x01 = REVERSED
#define CENTERED 0x02 // 0x02 = CENTRED

uint8_t loopCount = 0;
double output;
double setPoint = 25;
double currentTemp = 20.0;
float currentReading = 0;
uint16_t timeCounter = 0;

uint8_t state = 0;
#define CURRENTLINE 0x10
#define CURRENTMODE 0x20
#define CURRENTAUTOMODE 0x40
#define CURRENTRUN 0x80
// state, bits 0,1 = profile stage.  bit4=line (0/1) bit5=mode (0=manual, 1 = auto) bit6=(jedec/lowtemp) bit7=auto running

uint16_t jedecProfile[4][2]   = {{0, 120}, {180, 160}, {280, 220}, {390, 25}};
uint16_t lowTempProfile[4][2] = {{0, 120}, {240, 165}, {360, 25} , {500, 25}};

double Kp = 3.0, Ki = 0.01, Kd = 0.80;  // double 3.0, Ki = 0.01, Kd = 0.64;
//double Kp = 2.2, Ki = 0.4, Kd = 3.00;  // double 3.0, Ki = 0.01, Kd = 0.64;

PID myPID(&currentTemp, &output, &setPoint, Kp, Ki, Kd, DIRECT);

void setup() {
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  digitalWrite(CHIPSELECT, HIGH); // ss or strobe
  digitalWrite(COMMANDDATA, DATA); // command/data
  digitalWrite(11, HIGH); // data
  digitalWrite(13, HIGH); // clock

  pinMode(buttons[UP][PINS], INPUT); //yuk use constants!!!
  digitalWrite(buttons[UP][PINS], HIGH);
  pinMode(buttons[DOWN][PINS], INPUT);
  digitalWrite(buttons[DOWN][PINS], HIGH);
  pinMode(buttons[LEFT][PINS], INPUT);
  digitalWrite(buttons[LEFT][PINS], HIGH);
  pinMode(buttons[RIGHT][PINS], INPUT);
  digitalWrite(buttons[RIGHT][PINS], HIGH);

  pinMode(6, OUTPUT);
  analogWrite(6, 128);

  pinMode(PLATE, OUTPUT);

  pinMode(CHIPSELECT, OUTPUT); // SS STROBE
  pinMode(COMMANDDATA, OUTPUT); // command/data
  uint8_t startseq[] = {0x40, 0xA0, 0xC8, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x0C, 0xAC, 0x00, 0xAF};
  digitalWrite(COMMANDDATA, COMMAND); // set to command
  digitalWrite(CHIPSELECT, LOW); // SS
  for (uint8_t i = 0; i < 14; i++) SPI.transfer(startseq[i]);
  digitalWrite(CHIPSELECT, HIGH); // SS
  cls(0x00);
  lcdprint(0, 0, "HOT PLATE", REVERSED | CENTERED);
  printModeLine();
  analogReference(EXTERNAL);
  checkTempKeys();
  myPID.SetOutputLimits(0, 12);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  printTargetStart();
  Serial.begin(115200);
}

void checkTempKeys() {
  for (ti = 0; ti < 4; ti++) {
    if (!digitalRead(buttons[ti][PINS])) {
      if (buttons[ti][DEBOUNCE] != 8) buttons[ti][DEBOUNCE] += 1;
    }
    else {
      if (buttons[ti][DEBOUNCE] != 0) buttons[ti][DEBOUNCE] -= 1;
    }
    if ((buttons[ti][DEBOUNCE] >= 2) && !(buttons[ti][ONOFF] & PRESS)) buttons[ti][ONOFF] = PRESS | CHANGED; // was off now on, so switch on and flag (bit 2)
    if (buttons[ti][DEBOUNCE] == 8) buttons[ti][ONOFF] = PRESS | CHANGED; // key repeat
    if ((buttons[ti][DEBOUNCE] <= 1) && (buttons[ti][ONOFF] & PRESS)) buttons[ti][ONOFF] = CHANGED; // was off now on, so switch on and flag (bit 2)
  }
  currentReading = 0.6 * currentReading + 0.4 * analogRead(A0);
  float currentResistance = 100000 / ((1023.0 / currentReading) - 1.0);
  currentResistance = currentResistance / 100000;
  currentResistance = log(currentResistance);
  currentResistance /= 4092;  //3950 or 4092
  currentResistance += 1.0 / (25 + 273.15);
  currentResistance = 1.0 / currentResistance;
  currentTemp = currentResistance - 273.15;
}

void printCurrent() {
  int16_t i = int(output);
  char stringout[] = " Now: XXX C  PWM: XX  ";
  stringout[19] = (char)(48 + (i % 10));
  i /= 10;
  stringout[18] = (char)(48 + (i % 10));

  i = int(currentTemp);
  stringout[8] = (char)(48 + (i % 10));
  i /= 10;
  if(i==0) {stringout[7]=' ';} else {stringout[7] = (char)(48 + (i % 10));}
  i /= 10;
  if(i==0) {stringout[6]=' ';} else {stringout[6] = (char)(48 + (i % 10));}
  stringout[20] = '\0';
  lcdprint(7, 1, stringout, NORMAL);

  char stringout2[] = "Time: XXX Seconds   ";
  i = timeCounter;
  if ((state & CURRENTRUN) && (state & CURRENTAUTOMODE)) i = lowTempProfile[3][0] - i ;
  if ((state & CURRENTRUN) && !(state & CURRENTAUTOMODE)) i = jedecProfile[3][0] - i ;
  stringout2[8] = (char)(48 + (i % 10));
  i /= 10;
  if(i==0) {stringout2[7]=' ';} else {stringout2[7] = (char)(48 + (i % 10));}
  i /= 10;
  if(i==0) {stringout2[6]=' ';} else {stringout2[6] = (char)(48 + (i % 10));}
  stringout2[18] = '\0';
  lcdprint(6, 0, stringout2, NORMAL);
}

void printModeLine() {
  if (!(state & CURRENTLINE)) {
    if (!(state & CURRENTMODE)) {
      lcdprint(2, 0, ">Mode: Manual", REVERSED);
      return;
    }
    if (!(state & CURRENTAUTOMODE)) {
      lcdprint(2, 0, ">Mode: JEDEC", REVERSED);
      return;
    }
    lcdprint(2, 0, ">Mode: Low Temp", REVERSED);
  } else {
    if (!(state & CURRENTMODE)) {
      lcdprint(2, 0, ">Mode: Manual", NORMAL);
      return;
    }
    if (!(state & CURRENTAUTOMODE)) {
      lcdprint(2, 0, ">Mode: JEDEC", NORMAL);
      return;
    }
    lcdprint(2, 0, ">Mode: Low Temp", NORMAL);
  }
}

void printTargetStart() {
  if (!(state & CURRENTMODE)) {
    uint8_t i = int(setPoint);
    char stringout[] = ">Target: XXX C ";
    stringout[11] = (char)(48 + (i % 10));
    i /= 10;
    stringout[10] = (char)(48 + (i % 10));
    i /= 10;
    stringout[9] = (char)(48 + (i % 10));
    stringout[14] = '\0';

    if (state & CURRENTLINE) {
      lcdprint(3, 0, stringout, REVERSED);
    } else {
      lcdprint(3, 0, stringout, NORMAL);
    }
    return;
  }

  if (state & CURRENTRUN) {
    if (state & CURRENTLINE) {
      lcdprint(3, 0, ">Stop Run", REVERSED);
    } else {
      lcdprint(3, 0, ">Stop Run", NORMAL);
    }
  } else {
    if (state & CURRENTLINE) {
      lcdprint(3, 0, ">Start Run", REVERSED);
    } else {
      lcdprint(3, 0, ">Start Run", NORMAL);
    }
  }
}

void setpos(uint8_t page, uint8_t col) {
  col += 30;
  digitalWrite(COMMANDDATA, COMMAND); // set to command
  digitalWrite(CHIPSELECT, LOW); // SS
  SPI.transfer(0xB0 + page);
  SPI.transfer(0x10 + ((col & 0xf0) >> 4));
  SPI.transfer(0x00 + (col & 0x0f));
  digitalWrite(CHIPSELECT, HIGH); // SS
}

void cls(uint8_t fil) {
  for (uint8_t i = 0; i < 8; i++) {
    setpos(i, 0);
    digitalWrite(COMMANDDATA, DATA); // set to data
    digitalWrite(CHIPSELECT, LOW); // SS
    for (uint8_t j = 0; j < 102; j++) SPI.transfer(fil);
    digitalWrite(CHIPSELECT, HIGH); // SS
  }
}

uint16_t lcdprint(uint8_t page, uint8_t col, char *text, uint8_t prtstyle) {
  uint16_t j, k, collength = 0;
  uint8_t textlength, normalrev, printed = 0;
  textlength = strlen(text);
  for (t = 0; t < textlength; t++) {
    j = (uint8_t)text[t] - 32;
    if (j > 96) continue;
    collength += (pgm_read_byte_near(WIDTH + j) + 1); // width
  }
  if (prtstyle & REVERSED) {
    normalrev = 0xFF;
  }
  else {
    normalrev = 0x00;
  }
  if (prtstyle & CENTERED) {
    t = collength;
    if (t > DISPLAYWIDTH) t = DISPLAYWIDTH;
    col = (DISPLAYWIDTH - t) >> 1;
    setpos(page, 0);
    digitalWrite(COMMANDDATA, DATA); // set to data
    digitalWrite(CHIPSELECT, LOW);
    for (s = 0; s < col - 1; s++) {
      SPI.transfer(0x00 ^ normalrev);
    }
  }
  else {
    setpos(page, col);
    digitalWrite(COMMANDDATA, DATA); // set to data
    digitalWrite(CHIPSELECT, LOW);
  }
  if (prtstyle & REVERSED) {
    SPI.transfer(0xFF);
    col += 1;
  } // first column needs to be black
  for (t = 0; t < textlength; t++) {
    j = (uint8_t)text[t] - 32;
    if (j > 96) continue;
    k = pgm_read_byte_near(WIDTH + j); // width
    j = (j << 2) + j; // times 5
    SPI.transfer(pgm_read_byte_near(FONT + j) ^ normalrev);
    SPI.transfer(pgm_read_byte_near(FONT + j + 1) ^ normalrev);
    if (k > 2) SPI.transfer(pgm_read_byte_near(FONT + j + 2)^ normalrev);
    if (k > 3) SPI.transfer(pgm_read_byte_near(FONT + j + 3)^ normalrev);
    if (k > 4) SPI.transfer(pgm_read_byte_near(FONT + j + 4)^ normalrev);
    SPI.transfer(0x00 ^ normalrev);
    printed += (k + 1);
    if (printed + col > DISPLAYWIDTH) break; // although the display does not appear to mind no point printing beyond end of screen
  }
  for (t = printed + col; t <= DISPLAYWIDTH; t++) SPI.transfer(0x00 ^ normalrev); // clear to end of line
  digitalWrite(CHIPSELECT, HIGH); // SS
  return collength;
}

void loop() {
  static unsigned long now = 0;
  if ((millis() - now) > LOOPTIME) {
    now = millis();
    checkTempKeys();
    processKeys();
    loopCount += 1;
    if(loopCount == 25) {timeCounter +=1; printCurrent();}
    if (loopCount == 50) {
      timeCounter += 1;
      loopCount = 0;
      myPID.Compute();
      digitalWrite(PLATE, HIGH);
      printCurrent();
      Serial.println(currentTemp);
    }
    if (loopCount == int(output)) {
      digitalWrite(PLATE, LOW);
    }
    if ((state & CURRENTRUN) && (state & CURRENTAUTOMODE)) {
      if(timeCounter > lowTempProfile[state & 0x03][0]) { 
        setPoint = lowTempProfile[state & 0x03][1]; 
        state = (state + 1) & 0xF3;
        if(!(state & 0x03)) { state &= ~CURRENTRUN; timeCounter = 0; printTargetStart();}
      }
    }
    if ((state & CURRENTRUN) && !(state & CURRENTAUTOMODE)){
      if(timeCounter > jedecProfile[state & 0x03][0]) { 
        setPoint = jedecProfile[state & 0x03][1]; 
        state = (state + 1) & 0xF3;
        if(!(state & 0x03)) { state &= ~CURRENTRUN; timeCounter = 0; printTargetStart();}
      }      
    }
  }
}

void processKeys(void) {
  if (buttons[RIGHT][ONOFF] & CHANGED) {
    buttons[RIGHT][ONOFF] &= ~CHANGED;  // switch off change flag
    if ((buttons[RIGHT][ONOFF] & PRESS) && setPoint < 250 && !(state & CURRENTMODE) && (state & CURRENTLINE)) {
      setPoint += 1;
      printTargetStart();
      return;
    }
    if (!(buttons[RIGHT][ONOFF] & PRESS) && !(state & CURRENTMODE) && (state & CURRENTLINE)) {
      timeCounter = 0;
      return;
    }
    if ((buttons[RIGHT][ONOFF] & PRESS) && !(state & CURRENTLINE) && !(state & CURRENTRUN)) {
      if (!(state & CURRENTMODE)) {
        state |= CURRENTMODE;   // manual to jedec
        state &= ~CURRENTAUTOMODE;
        printTargetStart();
        printModeLine();
        return;
      }
      if ((state & CURRENTMODE) && !(state & CURRENTAUTOMODE)) {
        state |= CURRENTAUTOMODE;  // jedec to low temp
        printTargetStart();
        printModeLine();
        return;
      }
      if ((state & CURRENTMODE) && (state & CURRENTAUTOMODE)) {
        state &= ~CURRENTMODE;  // low temp to manual
        state &= ~CURRENTAUTOMODE;
        printTargetStart();
        printModeLine();
        return;
      }
    }
    if ((buttons[RIGHT][ONOFF] & PRESS) && (state & CURRENTLINE) && (state & CURRENTMODE)) {
      if (state & CURRENTRUN) { // was running now stopped
        state &= ~CURRENTRUN;
        state &= 0xF0;
        timeCounter = 0;
        setPoint=25;
      } else { // was stopped now started
        state &= 0xF0;
        state |= CURRENTRUN;
        timeCounter = 0;
      }
      printTargetStart();
      return;
    }
  }
  if (buttons[LEFT][ONOFF] & CHANGED) {
    buttons[LEFT][ONOFF] &= ~CHANGED;  // switch off change flag
    if ((buttons[LEFT][ONOFF] & PRESS) && setPoint > 1 && !(state & CURRENTMODE) && (state & CURRENTLINE)) {
      setPoint -= 1;
      printTargetStart();
      return;
    }
    if (!(buttons[LEFT][ONOFF] & PRESS) && !(state & CURRENTMODE) && (state & CURRENTLINE)) {
      timeCounter = 0;
      return;
    }
    if ((buttons[LEFT][ONOFF] & PRESS) && !(state & CURRENTLINE)  && !(state & CURRENTRUN)) {
      if (!(state & CURRENTMODE)) {
        state |= CURRENTMODE;  // manual to low temp
        state |= CURRENTAUTOMODE;
        printTargetStart();
        printModeLine();
        return;
      }
      if ((state & CURRENTMODE) && !(state & CURRENTAUTOMODE)) {
        state &= ~CURRENTMODE;  //jedec to manual
        state &= ~CURRENTAUTOMODE;
        printTargetStart();
        printModeLine();
        return;
      }
      if ((state & CURRENTMODE) && (state & CURRENTAUTOMODE)) {
        state &= ~CURRENTAUTOMODE;  // low temp to jedec
        printTargetStart();
        printModeLine();
        return;
      }
    }
  }

  if (buttons[UP][ONOFF] & CHANGED) {
    buttons[UP][ONOFF] &= ~CHANGED;  // switch off change flag
    if (buttons[UP][ONOFF] & PRESS) {
      state ^= CURRENTLINE;
      printTargetStart();
      printModeLine();
      return;
    }
  }
  if (buttons[DOWN][ONOFF] & CHANGED) {
    buttons[DOWN][ONOFF] &= ~CHANGED;
    if (buttons[DOWN][ONOFF] & PRESS) {
      state ^= CURRENTLINE;
      printTargetStart();
      printModeLine();
      return;
    }
  }
}
