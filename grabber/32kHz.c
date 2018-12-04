//gcc 32kHz.c -o 32kHz -lbcm2835

/*
 * Copyright (c) 2017,2018 Peter Budd. All rights reserved
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
    * The above copyright notice and this permission notice shall be included in all copies or substantial
      portions of the Software.
    * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
      BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
      IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
      WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
      SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER,
      ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY
      FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW

* This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
* bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
* hardware is currently a Pi Zero running Arch Linux.

* I pulled snippets of code from loads of places so if any of you recognise anything you wrote - thanks!

* This program is creates a 32KHz square wave on GPIO 12 (BCM18).  It is used to replace a crystal for the BNO080
* 

*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <linux/types.h>
#include <sys/types.h>
#include "BNO080.h"
#include <bcm2835.h>

#define INTPIN 23
#define RESETPIN 24
#define WAKPS0PIN 25

int main(int argc, char const *argv[]){

    if (!bcm2835_init()){
        printf("Unable to inititalise bcm2835\n");
        exit(1);
    }
    bcm2835_gpio_fsel(RESETPIN, BCM2835_GPIO_FSEL_OUTP); // reset
    bcm2835_gpio_clr(RESETPIN);
    bcm2835_gpio_fsel(WAKPS0PIN, BCM2835_GPIO_FSEL_OUTP); // WAKPS0
    bcm2835_gpio_set(WAKPS0PIN);
    bcm2835_gpio_fsel(INTPIN, BCM2835_GPIO_FSEL_INPT); // INT
    bcm2835_gpio_set_pud(INTPIN, BCM2835_GPIO_PUD_UP);

// setup clock
    bcm2835_gpio_fsel(18,BCM2835_GPIO_FSEL_ALT5); // set gpio 18 (pin12) to alt 5 (pwm on channel 0)
    bcm2835_pwm_set_clock(293); // 19,200,000 / 293 = 65529 
    bcm2835_pwm_set_range(0,2); // channel 0, range 2
    bcm2835_pwm_set_data(0, 1); // channel 0, data 1 (50:50 pwm)
    bcm2835_pwm_set_mode(0,1,1); // channel 0, mark-space=true, enable=true2

    bcm2835_close();
    return 0;
}

