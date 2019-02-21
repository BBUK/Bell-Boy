//gcc powermonitor.c -o pm -lbcm2835

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

* This program tests the i2c power monitoring stuff

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
    bcm2835_i2c_set_baudrate(400000);
    bcm2835_i2c_setSlaveAddress(0x10);
/*
    i2cBuffer[0]=11;
    bcm2835_i2c_write(i2cBuffer,1);
    usleep(1000);
    if(bcm2835_i2c_read(i2cBuffer,2) != BCM2835_I2C_REASON_OK ) printf("F");
    printf("FirstVcc: %d\n", (i2cBuffer[0]<<8)+i2cBuffer[1]); 
    usleep(1000);
    i2cBuffer[0]=12;
    bcm2835_i2c_write(i2cBuffer,1);    
    usleep(1000);
    if(bcm2835_i2c_read(i2cBuffer,2) != BCM2835_I2C_REASON_OK ) printf("F");
    printf("SecondVcc: %d\n", (i2cBuffer[0]<<8)+i2cBuffer[1]); 
    i2cBuffer[0]=13;
    usleep(1000);
    bcm2835_i2c_write(i2cBuffer,1);
    usleep(1000);
    if(bcm2835_i2c_read(i2cBuffer,2) != BCM2835_I2C_REASON_OK ) printf("F");
    printf("ThirdVcc: %d\n", (i2cBuffer[0]<<8)+i2cBuffer[1]); 
    usleep(1000);
*/  
    i2cBuffer[0]=2;
    bcm2835_i2c_write(i2cBuffer,1);
    int count = 0;
    int result =0;
    usleep(1000);
    while(!sig_exit){
        if(bcm2835_i2c_read(i2cBuffer,2) != BCM2835_I2C_REASON_OK ) printf("F");
        result = (i2cBuffer[0]<<8)+i2cBuffer[1];
        if(result & 0x8000){
            printf("Shutdown signal...%d\n",result - 0x8000);
            system("/usr/bin/sync && /usr/bin/shutdown -P now");
        }
        if(!(count % 10))printf("At %d minutes, received %d\n",count/10,result);
        count += 1;
        usleep(6000000); // check every six seconds
    }
    bcm2835_i2c_end(); 
    bcm2835_close();
    return 0;
}


