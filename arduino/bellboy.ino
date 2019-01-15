#include "LowPower.h"

#define piPowerEnablePin XX  // output - enables power to the Pi
#define batteryChargeEnablePin XX // output enables power to the battery charger
//#define voltageMeasurementPin XX
#define piPowerDownPin XX // input detects whether Pi is shut down (LOW=UP HIGH=POWERED_DOWN)(to be implemented)
#define piShutdownSignalPin XX //output sends signal to Pi to shutdown (to be implemented) 
#define piRunPin XX // output - forces pi not to boot when powered
#define chargeStatusPin XX // input detects status of battery charge
#define vSwitchDetectPin XX //input detects status of power slide switch (to be implemented)

// https://www.arduino.cc/en/Tutorial/ArduinoToBreadboard
// https://github.com/rocketscream/Low-Power/blob/master/LowPower.h

uint8_t STATE = 0; // 

// STATE 0 = unknown power source, Pi run pin = low
// STATE 1 = external power source, Pi run pin = low, battery charging
// STATE 2 = external power source, Pi run pin = low, battery charge complete
// STATE 3 = on battery power battery not measured
// STATE 4 = on battery power, battery OK, start booting (flashing red)
// STATE 5 = on battery power, battery OK, Pi booting (flashing green)
// STATE 6 = on battery power, Pi booted (solid green)
// STATE 7 = ordered shutdown mode
// STATE 8 = shut down

void setup{
    pinMode(piPowerEnablePin, OUTPUT);
    digitalWrite(piPowerEnablePin, LOW);
    
    pinMode(batteryChargeEnablePin, OUTPUT);
    digitalWrite(batteryChargeEnablePin, LOW);
    
    pinMode(piShutDownSignalPin,OUTPUT);
    digitalWrite(piShutdownSignalPin, LOW);
    
    pinMode(piRunPin,OUTPUT);
    digitalWrite(piRunPin,LOW);  // stop Pi from booting

    ledOff();

    // set the internal voltage reference to VCC (this is either battery voltage
    // or 5V from the Pi.  By measuring 3.3V from the Pi we can work out if the
    // Pi is being powered from its microUSB.  We can also use this to monitor
    // battery voltage.
    ADMUX = _BV(REFS0); // reference to VCC (PC0/ADC0 being measured)
}

void loop{
    uint16_t volts = 0;
    uint8_t led = 0;
    uint8_t count = 0;
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
    switch(STATE){
        case 0: // startUp
            volts = measureVcc();
            if(volts < 750  && volts > 100){ // must be on external power
                digitalWrite(batteryChargeEnablePin,HIGH); // enable charging
                STATE = 1;
            } else {
                digitalWrite(piPowerEnablePin, HIGH); // on battery power, and switch on, send power to Pi
                STATE = 3;
            }
            break;
        case 1: // external power, battery charging
            volts = measureVcc();
            if(volts > 749  || volts < 101) { STATE = 8; break; } // power removed
            if(digitalRead(chargeStatusPin) == HIGH) { STATE = 2; digitalWrite(batteryChargeEnablePin,LOW); break; } // charge complete
            ledRedOn();
            break;
        case 2: // external power, charge complete
            volts = measureVcc();
            if(volts > 749  || volts < 101) { STATE = 8; break; } // power removed go into shutdown mode
            ledGreenChirrup();
            break;
        case 3: // battery power,   pi powered, run still low
            volts = measureVcc();
            if(volts < 950) { STATE = 4; pinMode(piRunPin,INPUT); break; } // battery OK release run pin
            STATE = 8; // battery not OK, take power off Pi, go into shutdown loop
            break;
 //  dtoverlay=gpio-poweroff,gpiopin=6 (GPIO 6 has a "natural" pull high)
        case 4: // battery power, pi powered and running wait for Pi to start booting
            if(digitalRead(piPowerDownPin) == LOW) { STATE = 5; break; }// Pi now booting
            if(led == 0){
                led = 1;
                ledRedOn();
            } else {
                led = 0;
                ledOff();
            }
            return;  // do this to get a nice 0.5s on/off flash
        case 5: // battery power, pi powered and running wait Pi booting, wait for booting to finish
            if(digitalRead(piUpPin) == LOW) { STATE = 6; ledGreenOn(); break; }// Pi now up
            if(led == 0){
                led = 1;
                ledGreenOn();
            } else {
                led = 0;
                ledOff();
            }
            return;  // do this to get a nice 0.5s on/off flash
        case 6: // steady state operation
            if(digitalRead(piPowerDownPin) == HIGH) { // Pi shut itself down
                STATE = 8;
                ledOff();
                break; 
            }
            // FIXME power switched off //
            volts = measureVcc();
            if(volts < 750 && volts > 100) {// power plugged in -> go into order shutdown mode
                digitalWrite(piShutDownSignalPin,HIGH);
                count = 0;
                STATE = 7;
            }
            if(volts > 980){ // battery low -> go into order shutdown mode
                digitalWrite(piShutDownSignalPin,HIGH);
                count = 0;
                STATE = 7;
            }
            break;
        case 7: // ordered shutdown
            if(digitalRead(piPowerDownPin) == HIGH) { // Pi shut down
                STATE=8;
                break; 
            }
            count++;
            if(count < 30){ // flash red for 75 seconds then force shutdown
                if(led == 0){
                    led = 1;
                    ledRedOn();
                } else {
                    led = 0;
                    ledOff();
                }
                return;
            }
            STATE=8;
            break;
        case 8: // shutdown
            digitalWrite(batteryChargeEnablePin,LOW);
            digitalWrite(piShutDownSignalPin,LOW);
            pinMode(piRunPin,OUTPUT);
            digitalWrite(piRunPin,LOW);
            digitalWrite(piPowerEnablePin, LOW);
            ledRedChirrup();  // should switch off after above line but power button could be depressed ...
    }
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
}

// (3.3/5.0)*1024 = 676 = connected to external power
// (3.3/4.2)*1024 = 804 = internal power battery full
// (3.3/3.5)*1024 = 965 = internal power battery low
// (3.3/3.4)*1024 = 993 = internal power battery empty
//                    0 = pi not powered

uint16_t measureVcc(void){
    uint8_t low;
    LowPower.adcNoiseReduction(SLEEP_FOREVER, ADC_ON, TIMER2_ON);
//    sbi(ADCSRA, ADSC);
//    while (bit_is_set(ADCSRA,ADSC));
    low  = ADCL;
    return (ADCH << 8) | low;
}

void ledRedOn(void){
    digitalWrite(8,HIGH);
    digitalWrite(9,LOW);
}

void ledGreenOn(void){
    digitalWrite(8,LOW);
    digitalWrite(9,HIGH);
}

void ledOff(void){
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
}

void ledRedChirrup(void){
    ledRedOn();
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
    ledOff();
}

void ledGreenChirrup(void){
    ledGreenOn();
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
    ledOff();
}