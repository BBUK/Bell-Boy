//gcc calibrate.c -o calibrate -lbcm2835

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

* This program executes an initial calibration of the ICM20689 device, in terms of sample rate and
* gyro and accelerometer misalignment, bias and scale.  It uses a rather good method of calibration disclosed in these papers
* http://www.dis.uniroma1.it/~pretto/papers/tpm_icra2014.pdf
* http://www.dis.uniroma1.it/~pretto/papers/pg_imeko2014.pdf

* Code and installation instructions for the imu_tk calibration routines this program relies on are here:
* https://bitbucket.org/alberto_pretto/imu_tk.  My own notes on installation are in the NotesOnInstallingimu_tk.docx
* file in this directory.  My installation is performed on the full version of Raspbian.  Pi Zero can be used but is slow.
* 
* The general idea is that this program is run (via sudo) and a SAMP: command is executed a few times until you are happy
* that the result is stable.  The CALI: command is then executed and this starts pushing a timestamp and accelerometer 
* data to /tmp/CALIBRATIONDATA.  The device is kept still (i.e. resting on a table) for no 
* less than 50 seconds and is then put into around 30-40 different resting positions each lasting about 4 seconds.
* After this process, issue the STCA: command.  This program will then calculate the calibrations.  
* Takes about 5 minutes to crunch the numbers on a Pi Zero.  Upon successful calibration issue the SAVE: command and 
* this will save the calibration dats to the Arduino PROM memory.  From there it is accessed whenever the grabber
* program is started. 
* 
* Notes on outputs of imu_tk
* Triad model:
*         
* Misalignment matrix: 
* general case:
* 
*     [    1     -mis_yz   mis_zy  ]
* T = [  mis_xz     1     -mis_zx  ]
*     [ -mis_xy   mis_yx     1     ]
* 
* "body" frame special case:
* 
*     [  1     -mis_yz   mis_zy  ]
* T = [  0        1     -mis_zx  ]
*     [  0        0        1     ]
* 
* Scale matrix:
* 
*     [  s_x      0        0  ]
* K = [   0      s_y       0  ]
*     [   0       0       s_z ]
* 
* Bias vector:
* 
*     [ b_x ]
* B = [ b_y ]
*     [ b_z ]
* 
* Given a raw sensor reading X (e.g., the acceleration ), the calibrated "unbiased" reading X' is obtained
* 
* X' = T*K*(X - B)
* 
* with B the bias (variable) + offset (constant, possibly 0).  From sensor reading, subtract bias and then
* multiply T*K
* This program multiplies T and K before saving to the Arduino (no need to keep them separate) 
*/

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

void save(void);
void arduinoWriteFloat(uint8_t reg,float value);

#ifndef NULL
#define NULL 0
#endif


#define g0 9.8137 // Manchester estimation.  Use gcalculator.xlsx to work out for your latitude.  
#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 

FILE *fdCalibrationData;
double calibrationTimestamp = 0.0;
int CALIBRATING = 0;
unsigned char SPI_BUFFER[256];
unsigned char I2C_BUFFER[16];
int local_count = 0;
char local_outbuf[4000];

const unsigned int LOOPSLEEP = 50000;  // main loop processes every 0.05 sec.  Max number of samples in FIFO is 341.  Should be OK.

struct {
    float samplePeriod;
    float accBiasX;
    float accBiasY;
    float accBiasZ;
    float accTransformMatrix[9];
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;
    float gyroTransformMatrix[9];
} calibrationData = {.samplePeriod = 0.008, .accBiasX=0, .accBiasY=0, .accBiasZ = 0,
                    .accTransformMatrix[0]=1, .accTransformMatrix[1]=0, .accTransformMatrix[2]=0, .accTransformMatrix[3]=0,
                    .accTransformMatrix[4]=1, .accTransformMatrix[5]=0, .accTransformMatrix[6]=0, .accTransformMatrix[7]=0,
                    .accTransformMatrix[8]=1, .gyroBiasX=0, .gyroBiasY=0, .gyroBiasZ=0, .gyroTransformMatrix[0]=1, 
                    .gyroTransformMatrix[1]=0, .gyroTransformMatrix[2]=0, .gyroTransformMatrix[3]=0, .gyroTransformMatrix[4]=1,
                    .gyroTransformMatrix[5]=0, .gyroTransformMatrix[6]=0, .gyroTransformMatrix[7]=0, .gyroTransformMatrix[8]=1 };

union {
    float    _float;
    uint8_t  _bytes[sizeof(float)];
} floatConv;
 
volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

int main(int argc, char const *argv[]){
    char linein[50];
    char command[6];
    char details[42];
    char oscommand[90];
    int entries;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
    
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
 
    setup();
    fifoTimer();
    printf("Sample period: %f\n",calibrationData.samplePeriod);
	
    while(!sig_exit){
        usleep(LOOPSLEEP);
        pullData();
        while(fgets(linein, sizeof(linein), stdin ) != NULL) {
            entries = sscanf(linein, "%5s%[^\n]", command, details);
            if(entries < 1) {
                fprintf( stderr, "\nError reading: %s \n", linein );
                continue;
            }
            if(strcmp("SAMP:", command) == 0) {
                fifoTimer();
                printf("SAMP:%f\n",calibrationData.samplePeriod);
                continue;
            }
            if(strcmp("CALI:", command) == 0) {
                calibrationTimestamp = 0.0;
                fdCalibrationData = fopen("/tmp/CALIBRATIONDATA","w");
                if(fdCalibrationData == NULL) {
                    fprintf(stderr, "Could not open calibration file for writing\n");
                    printf("ESTR:\n");
                    continue;
                }
				CALIBRATING = 1;
                continue;
            }
            if(strcmp("STCA:", command) == 0) {
				if(fdCalibrationData != NULL) {
                    if(local_count != 0) {fputs(local_outbuf, fdCalibrationData); local_count = 0;}
					fflush(fdCalibrationData);
					fclose(fdCalibrationData);
					fdCalibrationData = NULL;
				}
				CALIBRATING = 0;
				printf("Starting calibration (will take a while)...\n");
				system("./BBimu_tk /tmp/CALIBRATIONDATA");
				printf("Done calibrating\n");
                continue;
            }
			if(strcmp("SAVE:", command) == 0) {
				save();
                continue;
            }
            printf("ESTR:\n");
            fprintf(stderr, "Unrecognised command: %s \n", command);
        }
    }
    return 0;
}
/*
    (registers 	10=samplePeriod, 11=accBiasX,
    12=accBiasY, 13=accBiasZ, 14=accScale00, 15=accScale01, 16=accScale02,
    17=accScale10, 18=accScale11, 19=accScale12, 20=accScale20, 21=gyroScale21,
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
void save(void){
    // Load up calibration files made by imu_tk.  All units are SI.
    printf("Loading calibration...\n");
    FILE *fdCalib;
    unsigned char linein[100];
    float scaleMatrix[9];
    float misalignMatrix[9];
    float transformMatrix[9];
    fdCalib = fopen("BBacc.calib","r");
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &misalignMatrix[0], &misalignMatrix[1], &misalignMatrix[2]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &misalignMatrix[3], &misalignMatrix[4], &misalignMatrix[5]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &misalignMatrix[6], &misalignMatrix[7], &misalignMatrix[8]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return; // blank line in file

    sscanf(linein, "%f %f %f", &scaleMatrix[0], &scaleMatrix[1], &scaleMatrix[2]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &scaleMatrix[3], &scaleMatrix[4], &scaleMatrix[5]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &scaleMatrix[6], &scaleMatrix[7], &scaleMatrix[8]);

    // create single transform matrix from scale and misalignment matrices
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            calibrationData.accTransformMatrix[3*i+j] =  misalignMatrix[3*i+0]*scaleMatrix[j+0];
            calibrationData.accTransformMatrix[3*i+j] += misalignMatrix[3*i+1]*scaleMatrix[j+3];
            calibrationData.accTransformMatrix[3*i+j] += misalignMatrix[3*i+2]*scaleMatrix[j+6];
        }
    }

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.accBiasX);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.accBiasY);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.accBiasZ);

    fclose(fdCalib);

    fdCalib = fopen("BBgyro.calib","r");

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &misalignMatrix[0], &misalignMatrix[1], &misalignMatrix[2]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &misalignMatrix[3], &misalignMatrix[4], &misalignMatrix[5]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &misalignMatrix[6], &misalignMatrix[7], &misalignMatrix[8]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &scaleMatrix[0], &scaleMatrix[1], &scaleMatrix[2]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &scaleMatrix[3], &scaleMatrix[4], &scaleMatrix[5]);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &scaleMatrix[6], &scaleMatrix[7], &scaleMatrix[8]);

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            calibrationData.gyroTransformMatrix[3*i+j] =  misalignMatrix[3*i+0]*scaleMatrix[j+0];
            calibrationData.gyroTransformMatrix[3*i+j] += misalignMatrix[3*i+1]*scaleMatrix[j+3];
            calibrationData.gyroTransformMatrix[3*i+j] += misalignMatrix[3*i+2]*scaleMatrix[j+6];
        }
    }

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.gyroBiasX);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.gyroBiasY);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.gyroBiasZ);

    fclose(fdCalib);
    
    printf("SamplePeriod: %f\n", calibrationData.samplePeriod);

    printf("accBiasX: %f\n", calibrationData.accBiasX);
    printf("accBiasY: %f\n", calibrationData.accBiasY);
    printf("accBiasZ: %f\n", calibrationData.accBiasZ);
    printf("accTransform: %f %f %f\n", calibrationData.accTransformMatrix[0],calibrationData.accTransformMatrix[1],calibrationData.accTransformMatrix[2]);
    printf("              %f %f %f\n", calibrationData.accTransformMatrix[3],calibrationData.accTransformMatrix[4],calibrationData.accTransformMatrix[5]);
    printf("              %f %f %f\n", calibrationData.accTransformMatrix[6],calibrationData.accTransformMatrix[7],calibrationData.accTransformMatrix[8]);
    printf("gyroBiasX: %f\n", calibrationData.gyroBiasX);
    printf("gyroBiasY: %f\n", calibrationData.gyroBiasY);
    printf("gyroBiasZ: %f\n", calibrationData.gyroBiasZ);
    printf("gyroTransform: %f %f %f\n", calibrationData.gyroTransformMatrix[0],calibrationData.gyroTransformMatrix[1],calibrationData.gyroTransformMatrix[2]);
    printf("               %f %f %f\n", calibrationData.gyroTransformMatrix[3],calibrationData.gyroTransformMatrix[4],calibrationData.gyroTransformMatrix[5]);
    printf("               %f %f %f\n", calibrationData.gyroTransformMatrix[6],calibrationData.gyroTransformMatrix[7],calibrationData.gyroTransformMatrix[8]);

    char answer;
    printf("Do values look sane? (CTRL_C to exit in the next 20 secs) \n");
    for(int i = 0; i<20; ++i){
        usleep(1000000);
        if(sig_exit) exit(0);
    }
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

	printf("Writing to prom...\n");
    arduinoWriteFloat(10,calibrationData.samplePeriod);
    arduinoWriteFloat(11,calibrationData.accBiasX);
    arduinoWriteFloat(12,calibrationData.accBiasY);
    arduinoWriteFloat(13,calibrationData.accBiasZ);
    arduinoWriteFloat(14,calibrationData.accTransformMatrix[0]);
    arduinoWriteFloat(15,calibrationData.accTransformMatrix[1]);
    arduinoWriteFloat(16,calibrationData.accTransformMatrix[2]);
    arduinoWriteFloat(17,calibrationData.accTransformMatrix[3]);
    arduinoWriteFloat(18,calibrationData.accTransformMatrix[4]);
    arduinoWriteFloat(19,calibrationData.accTransformMatrix[5]);
    arduinoWriteFloat(20,calibrationData.accTransformMatrix[6]);
    arduinoWriteFloat(21,calibrationData.accTransformMatrix[7]);
    arduinoWriteFloat(22,calibrationData.accTransformMatrix[8]);

    arduinoWriteFloat(23,calibrationData.gyroBiasX);
    arduinoWriteFloat(24,calibrationData.gyroBiasY);
    arduinoWriteFloat(25,calibrationData.gyroBiasZ);
    arduinoWriteFloat(26,calibrationData.gyroTransformMatrix[0]);
    arduinoWriteFloat(27,calibrationData.gyroTransformMatrix[1]);
    arduinoWriteFloat(28,calibrationData.gyroTransformMatrix[2]);
    arduinoWriteFloat(29,calibrationData.gyroTransformMatrix[3]);
    arduinoWriteFloat(30,calibrationData.gyroTransformMatrix[4]);
    arduinoWriteFloat(31,calibrationData.gyroTransformMatrix[5]);
    arduinoWriteFloat(32,calibrationData.gyroTransformMatrix[6]);
    arduinoWriteFloat(33,calibrationData.gyroTransformMatrix[7]);
    arduinoWriteFloat(34,calibrationData.gyroTransformMatrix[8]);

    printf("Verifying...\n");
    I2C_BUFFER[0]=10;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 10 (samplePeriod)
    bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
    if(calibrationData.samplePeriod != extractFloat(0)) printf("SamplePeriod: sent %f received %f \n", calibrationData.samplePeriod, extractFloat(0));

    I2C_BUFFER[0]=200;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 200 (accBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.accBiasX != extractFloat(0)) printf("accBiasX: sent %f received %f \n", calibrationData.accBiasX, extractFloat(0));
    if(calibrationData.accBiasY != extractFloat(4)) printf("accBiasY: sent %f received %f \n", calibrationData.accBiasY, extractFloat(4));
    if(calibrationData.accBiasZ != extractFloat(8)) printf("accBiasZ: sent %f received %f \n", calibrationData.accBiasZ, extractFloat(8));

    I2C_BUFFER[0]=201;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 201 (accTransform0)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.accTransformMatrix[0] != extractFloat(0)) printf("accScale00: sent %f received %f \n", calibrationData.accTransformMatrix[0], extractFloat(0));
    if(calibrationData.accTransformMatrix[1] != extractFloat(4)) printf("accScale01: sent %f received %f \n", calibrationData.accTransformMatrix[1], extractFloat(4));
    if(calibrationData.accTransformMatrix[2] != extractFloat(8)) printf("accScale02: sent %f received %f \n", calibrationData.accTransformMatrix[2], extractFloat(8));

    I2C_BUFFER[0]=202;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 202 (accTransform1)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.accTransformMatrix[3] != extractFloat(0)) printf("accScale10: sent %f received %f \n", calibrationData.accTransformMatrix[3], extractFloat(0));
    if(calibrationData.accTransformMatrix[4] != extractFloat(4)) printf("accScale11: sent %f received %f \n", calibrationData.accTransformMatrix[4], extractFloat(4));
    if(calibrationData.accTransformMatrix[5] != extractFloat(8)) printf("accScale12: sent %f received %f \n", calibrationData.accTransformMatrix[5], extractFloat(8));

    I2C_BUFFER[0]=203;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 203 (accTransform2)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.accTransformMatrix[6] != extractFloat(0)) printf("accScale20: sent %f received %f \n", calibrationData.accTransformMatrix[6], extractFloat(0));
    if(calibrationData.accTransformMatrix[7] != extractFloat(4)) printf("accScale21: sent %f received %f \n", calibrationData.accTransformMatrix[7], extractFloat(4));
    if(calibrationData.accTransformMatrix[8] != extractFloat(8)) printf("accScale22: sent %f received %f \n", calibrationData.accTransformMatrix[8], extractFloat(8));

    I2C_BUFFER[0]=204;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 204 (gyroBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.gyroBiasX != extractFloat(0)) printf("gyroBiasX: sent %f received %f \n", calibrationData.gyroBiasX, extractFloat(0));
    if(calibrationData.gyroBiasY != extractFloat(4)) printf("gyroBiasY: sent %f received %f \n", calibrationData.gyroBiasY, extractFloat(4));
    if(calibrationData.gyroBiasZ != extractFloat(8)) printf("gyroBiasZ: sent %f received %f \n", calibrationData.gyroBiasZ, extractFloat(8));

    I2C_BUFFER[0]=205;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 205 (gyroTransform0)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.gyroTransformMatrix[0] != extractFloat(0)) printf("gyroScale00: sent %f received %f \n", calibrationData.gyroTransformMatrix[0], extractFloat(0));
    if(calibrationData.gyroTransformMatrix[1] != extractFloat(4)) printf("gyroScale01: sent %f received %f \n", calibrationData.gyroTransformMatrix[1], extractFloat(4));
    if(calibrationData.gyroTransformMatrix[2] != extractFloat(8)) printf("gyroScale02: sent %f received %f \n", calibrationData.gyroTransformMatrix[2], extractFloat(8));

    I2C_BUFFER[0]=206;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 206 (gyroTransform1)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.gyroTransformMatrix[3] != extractFloat(0)) printf("gyroScale10: sent %f received %f \n", calibrationData.gyroTransformMatrix[3], extractFloat(0));
    if(calibrationData.gyroTransformMatrix[4] != extractFloat(4)) printf("gyroScale11: sent %f received %f \n", calibrationData.gyroTransformMatrix[4], extractFloat(4));
    if(calibrationData.gyroTransformMatrix[5] != extractFloat(8)) printf("gyroScale12: sent %f received %f \n", calibrationData.gyroTransformMatrix[5], extractFloat(8));

    I2C_BUFFER[0]=207;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 207 (gyroTransforme2)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.gyroTransformMatrix[6] != extractFloat(0)) printf("gyroScale20: sent %f received %f \n", calibrationData.gyroTransformMatrix[6], extractFloat(0));
    if(calibrationData.gyroTransformMatrix[7] != extractFloat(4)) printf("gyroScale21: sent %f received %f \n", calibrationData.gyroTransformMatrix[7], extractFloat(4));
    if(calibrationData.gyroTransformMatrix[8] != extractFloat(8)) printf("gyroScale22: sent %f received %f \n", calibrationData.gyroTransformMatrix[8], extractFloat(8));
 
    printf("Verification complete.\n");
}

void arduinoWriteFloat(uint8_t reg,float value){
    I2C_BUFFER[0]=reg;
    floatConv._float = value;
    I2C_BUFFER[1] = floatConv._bytes[0];
    I2C_BUFFER[2] = floatConv._bytes[1];
    I2C_BUFFER[3] = floatConv._bytes[2];
    I2C_BUFFER[4] = floatConv._bytes[3];
    bcm2835_i2c_write(I2C_BUFFER,5); 
    usleep(750000); // do a big delay as Arduino main loop has a 500ms sleep - I have not tested whether the sleep is interrupted by i2c activity as it should be, so this is really just a just in case
}

float extractFloat(uint8_t index){
    floatConv._bytes[0] = I2C_BUFFER[index];
    floatConv._bytes[1] = I2C_BUFFER[index+1];
    floatConv._bytes[2] = I2C_BUFFER[index+2];
    floatConv._bytes[3] = I2C_BUFFER[index+3];
    return floatConv._float;
}


// This function works out how quickly samples are pushed out by the ICM20689s
// by counting how quickly samples are pushed out to the FIFOs.
void fifoTimer(void){
    float dummy[6];
    calibrationData.samplePeriod = timer();
    while (readFIFOcount() != 0) readFIFO(dummy);
}

float timer(void){
    struct timeval start, stop;
    int cco, loops;
    float dummy[6];
    float result = 0.0;
    loops = 4;
    for(int i = 0; i < loops; ++i){
        while (readFIFOcount() != 0) readFIFO(dummy);
        while (readFIFOcount() == 0);
        gettimeofday(&start, NULL);
        readFIFO(dummy);
        while (readFIFOcount() < (196 * 12)) usleep(4000);  // sleep for about 2 periods
        while(1){
            usleep(100);
            cco = readFIFOcount();
            if(cco == (200 * 12)) {
                gettimeofday(&stop, NULL);
                result += (float)((stop.tv_sec-start.tv_sec)+(float)((stop.tv_usec-start.tv_usec)-50)/1000000.0);
                break;
            }
            if(cco > (200 * 12)) {
                i -= 1;
                break;
            }
        }
        usleep(5000);
    }
    return (result/(200*loops));
}

void pullData(void){
    float fifoData[6];
    char local_outbuf_line[150];

    int count = readFIFOcount();

    if (count > 4000){ // overflow (or nearly so).  If you see this then stop the calibration and start again
        printf("\nEOVF:\n");
        //clear data paths and reset FIFO (keep i2c disabled)
        writeRegister(ICM20689_USER_CTRL,0x15);
        usleep(1000);
 
        //start FIFO - keep i2c disabled
        writeRegister(ICM20689_USER_CTRL,0x50);
    }
    if(count_1 < 48) return;
    
    while(count >= 48){
        count -= 48;

        readFIFO(fifoData);
        if(CALIBRATING){
            sprintf(local_outbuf_line,"%+.8e %+.8e %+.8e %+.8e %+.8e %+.8e %+.8e\n", calibrationTimestamp, fifoData[3] * DEGREES_TO_RADIANS_MULTIPLIER,fifoData[4] * DEGREES_TO_RADIANS_MULTIPLIER,fifoData[5] * DEGREES_TO_RADIANS_MULTIPLIER, fifoData[0] * g0 ,fifoData[1] * g0,fifoData[2] * g0);
            if (local_count + strlen(local_outbuf_line) > (sizeof local_outbuf -2)) {
                fputs(local_outbuf, fdCalibrationData);
                fflush(fdCalibrationData);
                local_count = 0;
            }
            local_count += sprintf(&local_outbuf[local_count],local_outbuf_line);
            calibrationTimestamp += calibrationData.samplePeriod;
        }
    }
}

void setup(void){
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
	
	//  setup SPI
    if (!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. \n");
      exit(1);
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    bcm2835_spi_set_speed_hz(8000000);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);

   //reset device
    writeRegisterBits(ICM20689_PWR_MGMT_1,0xFF,0x80);
    usleep(100000);
 
   //disable i2c
    writeRegisterBits(ICM20689_USER_CTRL,0xFF,0x10);

    if(readRegister(ICM20689_WHO_AM_I) != 0x98){
      printf("Unable to read from device. \n");
      exit(1);        
    }
    // bring out of sleep, set clksel (to PLL) and disable temperature sensor
    writeRegister(ICM20689_PWR_MGMT_1,0x09);
    usleep(30000); 

    //configure DLPF
    writeRegister(ICM20689_CONFIG,0x01);

    //full scale accelerometer range to 4g
    writeRegister(ICM20689_ACCEL_CONFIG,0x08);

    //full scale gyro range to 500deg/s
    writeRegister(ICM20689_GYRO_CONFIG,0x08);

    //set sample rate divider we want 500Hz assuming base clock is 1KHz
    writeRegister(ICM20689_SMPLRT_DIV,0x01);

    //set FIFO size to 4K (this setting does not appear in the V1 datasheet!)
    writeRegister(ICM20689_ACCEL_CONFIG_2,0xC0);
    
    //clear data paths and reset FIFO (keep i2c disabled)
    writeRegister(ICM20689_USER_CTRL,0x15);
    usleep(1000);

    //select registers for FIFO
    writeRegister(ICM20689_FIFO_EN,0x78);
    
    //start FIFO - keep i2c disabled
    writeRegister(ICM20689_USER_CTRL,0x50);
}

uint16_t readFIFOcount(){
    return ((uint16_t)readRegister(ICM20689_FIFO_COUNTH) << 8) + readRegister(ICM20689_FIFO_COUNTL); 
}

void readFIFO(float* values){
    float temp0;
    SPI_BUFFER[0] = ICM20689_FIFO_R_W | 0x80;
    for(uint8_t i = 1; i<13; ++i){
        SPI_BUFFER[i] = 0;  
    }
    bcm2835_spi_transfern(SPI_BUFFER,13);
    values[0]=((uint16_t)SPI_BUFFER[1] << 8) + SPI_BUFFER[2]; 
    values[1]=((uint16_t)SPI_BUFFER[3] << 8) + SPI_BUFFER[4];
    values[2]=((uint16_t)SPI_BUFFER[5] << 8) + SPI_BUFFER[6];
    values[3]=((uint16_t)SPI_BUFFER[7] << 8) + SPI_BUFFER[8]; 
    values[4]=((uint16_t)SPI_BUFFER[9] << 8) + SPI_BUFFER[10];
    values[5]=((uint16_t)SPI_BUFFER[11] << 8) + SPI_BUFFER[12];

    // remapping of axes X->Z Y->X  X->Y
    temp0=values[0];
    values[0]=values[1];
    values[1]=values[2];
    values[2]=temp0;

    temp0=values[3];
    values[3]=values[4];
    values[4]=values[5];
    values[5]=temp0;

    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] *= (4.0/32768.0); // 4G full scale
    }
  
    for(int i=3; i<6; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] *= (500.0/32768.0); // 500deg/sec full scale
    }
}

void writeRegister(uint8_t reg, uint8_t value){
    SPI_BUFFER[0] = reg;
    SPI_BUFFER[1] = value;
    bcm2835_spi_transfern(SPI_BUFFER,2);
}

void writeRegisterBits(uint8_t reg, uint8_t mask, uint8_t value){
    uint8_t readValue;
    readValue = readRegister(reg);
    SPI_BUFFER[0] = reg; // & 0x7F;  // clear high bit for a write
    SPI_BUFFER[1] = (readValue & mask) | value;
    bcm2835_spi_transfern(SPI_BUFFER,2);
}

uint8_t readRegister(uint8_t reg){
    SPI_BUFFER[0] = reg | 0x80;  //set high bit for a read
    SPI_BUFFER[1] = 0;
    bcm2835_spi_transfern(SPI_BUFFER,2);
    return SPI_BUFFER[1];
}
