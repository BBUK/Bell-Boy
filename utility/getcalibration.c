//gcc getcalibration.c -o getcalibration -lbcm2835

/*
 * Copyright (c) 2017,2018,2019 Peter Budd. All rights reserved
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
*
* This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
* bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
* hardware is currently a Pi Zero running Arch Linux.

* This program pulls calibration data from the Arduino. 
* 
* */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <sys/time.h>
#include <linux/types.h>
#include <bcm2835.h>
#include "ICM20689.h"

#ifndef NULL
#define NULL 0
#endif

unsigned char I2C_BUFFER[16];

int main(int argc, char const *argv[]){
    if (!bcm2835_init()){
        printf("Unable to inititalise bcm2835\n");
        exit(1);
    }
	//setup I2C
    if (!bcm2835_i2c_begin()){
        printf("Unable to inititalise i2c\n");
        exit(1);
    }
    bcm2835_i2c_set_baudrate(100000);
    bcm2835_i2c_setSlaveAddress(0x10);

/*
    (registers 	10=samplePeriod, 11=accBiasX,
    12=accBiasY, 13=accBiasZ, 14=accScale00, 15=accScale01, 16=accScale02,
    17=accScale10, 18=accScale11, 19=accScale12, 20=accScale20, 21=accScale21,
    22=accScale22, 23=gyroBiasX, 24=gyroBiasY, 25=gyroBiasZ, 26=gyroScale00,
    27=gyroScale01, 28=gyroScale02, 29=gyroScale10, 30=gyroScale11, 31=gyroScale12, 
    32=gyroScale20, 33=gyroScale21, 34=gyroScale22, 
    200=accBiasXYZ(all in one go 12 bytes, three floats), 
    201=accScale0(all in one go 12 bytes, three floats),
    202=accScale1 (all in one go 12 bytes, three floats),
    203=accScale2 (all in one go 12 bytes, three floats),
    204=gyroBiasXYZ(all in one go 12 bytes, three floats), 
    205=gyroScale0(all in one go 12 bytes, three floats),
    206=gyroScale1 (all in one go 12 bytes, three floats),
    207=gyroScale2 (all in one go 12 bytes, three floats).

*/

    I2C_BUFFER[0]=10;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 10 (samplePeriod)
    bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
    printf("SamplePeriod: %f\n", extractFloat(0));

    I2C_BUFFER[0]=200;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); 
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("accBiasX: %f\n", extractFloat(0));
    printf("accBiasY: %f\n", extractFloat(4));
    printf("accBiasZ: %f\n", extractFloat(8));

    I2C_BUFFER[0]=201;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("accTranform: %f %f %f\n", extractFloat(0), extractFloat(4), extractFloat(8));
 
    I2C_BUFFER[0]=202;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("             %f %f %f\n", extractFloat(0), extractFloat(4), extractFloat(8));
 
    I2C_BUFFER[0]=203;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("             %f %f %f\n", extractFloat(0), extractFloat(4), extractFloat(8));
 
    I2C_BUFFER[0]=204;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("gyroBiasX: %f\n", extractFloat(0));
    printf("gyroBiasY: %f\n", extractFloat(4));
    printf("gyroBiasZ: %f\n", extractFloat(8));

    I2C_BUFFER[0]=205;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("gyroTranform: %f %f %f\n", extractFloat(0), extractFloat(4), extractFloat(8));

    I2C_BUFFER[0]=206;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("              %f %f %f\n", extractFloat(0), extractFloat(4), extractFloat(8));

    I2C_BUFFER[0]=207;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100);
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    printf("              %f %f %f\n", extractFloat(0), extractFloat(4), extractFloat(8));
}

float extractFloat(uint8_t index){
    union {
    float    _float;
    uint8_t  _bytes[sizeof(float)];
    } floatConv;

    floatConv._bytes[0] = I2C_BUFFER[index];
    floatConv._bytes[1] = I2C_BUFFER[index+1];
    floatConv._bytes[2] = I2C_BUFFER[index+2];
    floatConv._bytes[3] = I2C_BUFFER[index+3];
    return floatConv._float;
}

