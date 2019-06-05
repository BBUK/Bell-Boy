//gcc powermonitor.c -o pm -lbcm2835

/*
 * Copyright (c) 2018,2019 Peter Budd. All rights reserved
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

* This program monitors the power reports from the Arduino and executes shutdown if requested.
* It is necessary as the grabber may not be operating so this program and the grabber program hand off
* control to each other via the presence (or otherwise) of the /tmp/BBlive file.  The /tmp/BBlive is created by
* the grabber so that this program knows not to query the Arduino whenever the grabber is active.

* Another thing that might not be obvious is that the wifi network systemd service file creates a /tmp/HAPDstart
* file.  This signals this program that the wifi network is up and this program then signals the Arduino of that.
* The Arduino uses this to switch from the regular on-off green flash of the LED to an intermittent flash.  The
* overall idea being that the Arduino can trigger a shutdown if the network does not come up.

*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <signal.h>
#include <errno.h>
#include <linux/types.h>
#include <sys/types.h>
#include <dirent.h>
#include <bcm2835.h>


volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

int main(int argc, char const *argv[]){
    char i2cBuffer[4];
    struct sigaction sig_action;
    int result = 0;

    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);

    if (!bcm2835_init()){
        printf("Unable to inititalise bcm2835\n");
        exit(1);
    }
    if (!bcm2835_i2c_begin()){
        printf("Unable to inititalise i2c\n");
        exit(1);
    }
    bcm2835_i2c_set_baudrate(100000);
    bcm2835_i2c_setSlaveAddress(0x10);

    i2cBuffer[0]=2;
    bcm2835_i2c_write(i2cBuffer,1); usleep(100); // set register 2 (power)

    while(!sig_exit){
        if(0 == system("pidof -x hostapd > /dev/null") ){
            i2cBuffer[0]=1;
            bcm2835_i2c_write(i2cBuffer,1); usleep(100); // set register 1 - indicates sucessful boot to Arduino
            break;	
        }
        bcm2835_i2c_read(i2cBuffer,2); usleep(100);
        result = (i2cBuffer[0]<<8)+i2cBuffer[1];
        if(result & 0x8000) system("/usr/bin/sync && /usr/bin/shutdown -P now");
        usleep(500000); // check every half second
    }
  
    while(!sig_exit){
        if (0 != system("pidof -x grabber > /dev/null")){ // don't run if grabber is running (it will do its own checks)
            i2cBuffer[0]=2;
            bcm2835_i2c_write(i2cBuffer,1); usleep(100); // set register 2 (power)
            bcm2835_i2c_read(i2cBuffer,2); usleep(100);
            result = (i2cBuffer[0]<<8)+i2cBuffer[1];
            if(result & 0x8000) system("/usr/bin/sync && /usr/bin/shutdown -P now");
        }
        usleep(5000000); // check every 5 seconds
    }
    bcm2835_i2c_end(); 
    bcm2835_close();
    return 0;
}


