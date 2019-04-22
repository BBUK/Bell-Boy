#include <avr/pgmspace.h>
#include <Servo.h>

#define switchPin 13

#define hMax 2600
#define hMin 530

#define vMax 2600
#define vMin 530


//#define vMax 1900
//#define vMin 650

uint8_t debounce = 0 ; // debounce count for button
uint8_t buttonStatus = 0;
#define PRESS 1
#define CHANGED 2

uint8_t state = 0;
uint16_t loopCount = 0;

// store position in a single byte, hign nibble is horiz position (0-F), low nibble is vert position (0-F)

const uint8_t servoPositions[] PROGMEM = {0xC7, 0xA7, 0x87, 0x75, 0x54, 0x33, 0x22, 0x11, 0x00,
                                                0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2F,
                                                0x4F, 0x4C, 0x4A, 0x48, 0x46, 0x44, 0x42, 0x40,
                                                0x60, 0x62, 0x64, 0x66, 0x68, 0x6A, 0x6C, 0x6F,
                                                0x8F, 0x8C, 0x8A, 0x88, 0x86, 0x84, 0x82, 0x80,
                                                0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE,
                                                0xCE, 0xCC, 0xCA, 0xC8, 0xC6, 0xC4, 0xC2, 0xC1,
                                                0xD2, 0xD3, 0xD4, 0xD6, 0xD8, 0xDA, 0xDB, 0xDC,
                                                0xCC, 0xBB, 0xA9, 0xA8, 0xA7, 0xB7, 0xC7};

Servo vertServo;
Servo horizServo;

void setup() {
  digitalWrite(switchPin, HIGH);
  vertServo.attach(9,vMin,vMax);
  horizServo.attach(10,hMin,hMax);
  moveEm(1);
}

void loop() {
  delay(100); 
  if (!digitalRead(switchPin)) {
    if (debounce != 5) debounce += 1;
  } else {
    if (debounce != 0) debounce -= 1;
  }
  if ((debounce >= 3) && !(buttonStatus & PRESS)) buttonStatus = PRESS | CHANGED; // was off now on, so switch on and flag (bit 2)
  if ((debounce < 2 ) && (buttonStatus & PRESS)) buttonStatus = CHANGED; // was on now off, so switch on and flag (bit 2)

  if (buttonStatus & CHANGED) {
    buttonStatus &= ~CHANGED;  // switch off change flag
    if (buttonStatus & PRESS) {
      if (state == 0) {
        state = 1;
        loopCount = 0;
        moveEm(2); // signal start
        delay(1000);
        moveEm(1);
      } else {
        state = 0;
        moveEm(1);
      }
    }
  }

  loopCount += 1;
  if ((state == 1) && (loopCount == 550)) {
    loopCount = 0;
    state = 2;
    moveEm(state);
  }
  if ((state > 1) && (loopCount == 40)) {
    state += 1;
    loopCount = 0;
    if (state == 72) {
      state = 0;
      moveEm(1);
    } else {
      moveEm(state);
    }
  }
}

void moveEm(uint8_t moveTo) {
  uint8_t readByte = pgm_read_byte_near(servoPositions + moveTo - 1);
  uint8_t hPos = readByte >> 4;
  uint8_t vPos = readByte & 0x0F;
  vertServo.write(vPos * 11);
  horizServo.write(hPos * 11);
}
