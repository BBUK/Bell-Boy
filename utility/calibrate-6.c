//gcc calibrate-6.c -o calibrate -lm -lbcm2835

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
* The above licence does not apply to the following functions:
* doAccelerometerCalibration, sensorCalibration_ForwardSubstitution, addPositionMeans,
* sensorCalibrationPushSampleForOffsetCalculation, sensorCalibrationPushSampleForScaleCalculation,
* sensorCalibration_BackwardSubstitution, sensorCalibration_SolveLGS,
* sensorCalibration_gaussLR, sensorCalibrationSolveForOffset and
* sensorCalibrationSolveForScale.  
*
* Those functions have been adapted from Cleanflight.
* https://github.com/iNavFlight/inav and are issued under the following
* licence.
*
* Cleanflight is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Cleanflight is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
* 
* This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
* bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
* hardware is currently a Pi Zero running Arch Linux.

* This program executes an initial calibration of the ICM20689 device, in terms of sample rate,
* gyro bias and accelerometer bias and scale.

* The general idea is that this program is run and a SAMP: command is executed a few times until you are happy
* that the result is stable.  The CALI: command is then executed and this starts the calibration process (follow
* on screen instructions).  To stop the calibration process early issue the STCA: command.  
* Upon successful calibration issue the SAVE: command and  this will save the calibration dats to the Arduino 
* PROM memory.  From there it is accessed whenever the grabber program is started. 
* 
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

#define ICM20689_SMPLRT_DIV          (0x19)
#define ICM20689_CONFIG              (0x1A)
#define ICM20689_GYRO_CONFIG         (0x1B)
#define ICM20689_ACCEL_CONFIG        (0x1C)
#define ICM20689_ACCEL_CONFIG_2      (0x1D)
#define ICM20689_FIFO_EN             (0x23) 
#define ICM20689_USER_CTRL           (0x6A)
#define ICM20689_PWR_MGMT_1          (0x6B)
#define ICM20689_PWR_MGMT_2          (0x6C)
#define ICM20689_FIFO_COUNTH         (0x72)
#define ICM20689_FIFO_COUNTL         (0x73)
#define ICM20689_FIFO_R_W            (0x74)
#define ICM20689_WHO_AM_I            (0x75)

void setup(void);
void doCalibration(void);
void readFIFO(float* values);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void writeRegisterBits(uint8_t reg, uint8_t mask, uint8_t value);
uint16_t readFIFOcount(void);
float extractFloat(uint8_t index);
void save(void);
void arduinoWriteFloat(uint8_t reg,float value);
void takeSamples(void);
void fifoTimer(void);
float timer(void);
float getSD(float* samples);
void doAccelerometerCalibration(uint8_t position);
void sensorCalibration_ForwardSubstitution(float LR[4][4], float y[4], float b[4]);
void addPositionMeans(int position);
void sensorCalibrationPushSampleForOffsetCalculation(float sample[3]);
void sensorCalibrationPushSampleForScaleCalculation(int axis, float sample[3], int target);
void sensorCalibration_BackwardSubstitution(float LR[4][4], float x[4], float y[4]);
static void sensorCalibration_SolveLGS(float A[4][4], float x[4], float b[4]);
static void sensorCalibration_gaussLR(float mat[4][4]);
void sensorCalibrationSolveForOffset(float result[3]);
void sensorCalibrationSolveForScale(float result[3]);

#ifndef NULL
#define NULL 0
#endif

int FILLING = 0;
char SPI_BUFFER[256];
char I2C_BUFFER[16];

const unsigned int LOOPSLEEP = 30000;  // main loop processes every 0.03 sec.  Max number of samples in FIFO is 341.  Should be OK.

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
    float averagedAccelerometers[6][3];
    unsigned int calibrationState;
    float lastAccX;
    float lastAccY;
    float lastAccZ;
    int stable;
    float stateXtY[4];
    float stateXtX[4][4];
    unsigned int positionsDone;
    float accSamples[6][3];
} calibrationData = {.samplePeriod = 0.002, .accBiasX=0, .accBiasY=0, .accBiasZ = 0,
                    .accTransformMatrix[0]=1, .accTransformMatrix[1]=0, .accTransformMatrix[2]=0, .accTransformMatrix[3]=0,
                    .accTransformMatrix[4]=1, .accTransformMatrix[5]=0, .accTransformMatrix[6]=0, .accTransformMatrix[7]=0,
                    .accTransformMatrix[8]=1, .gyroBiasX=0, .gyroBiasY=0, .gyroBiasZ=0, .gyroTransformMatrix[0]=1, 
                    .gyroTransformMatrix[1]=0, .gyroTransformMatrix[2]=0, .gyroTransformMatrix[3]=0, .gyroTransformMatrix[4]=1,
                    .gyroTransformMatrix[5]=0, .gyroTransformMatrix[6]=0, .gyroTransformMatrix[7]=0, .gyroTransformMatrix[8]=1,
                    .calibrationState=0, .lastAccX = 0.0, .lastAccY=0.0, .lastAccZ = 0.0, .stable = 0, .positionsDone = 0};

//defines for positionsDone
#define POWERSWDOWN 1
#define POWERSWUP 2
#define SDDOWN 4
#define SDUP 8
#define LOGODOWN 16
#define LOGOUP 32

union {
    float    _float;
    uint8_t  _bytes[sizeof(float)];
} floatConv;
 
#define BUFFERSIZE 1200
struct {
    unsigned int numberOfSamplesTaken;
    float gx[BUFFERSIZE];
    float gy[BUFFERSIZE];
    float gz[BUFFERSIZE];
    float ax[BUFFERSIZE];
    float ay[BUFFERSIZE];
    float az[BUFFERSIZE];
} buffer;

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
    int entries;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
    
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    if (0 == system("pidof -x grabber > /dev/null")){
        printf("This can not be run at the same time as a Bell-Boy browser window is open.\nPlease close he window and re-run this program.\n");
        exit(1);
    }
 
    setup();
    fifoTimer();
    printf("Sample period: %f\n",calibrationData.samplePeriod);
	
    while(!sig_exit){
        usleep(LOOPSLEEP);
        takeSamples();
        doCalibration();
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
                calibrationData.calibrationState = 1;
                continue;
            }
            if(strcmp("STCA:", command) == 0) {
                calibrationData.calibrationState = 0;
				printf("Stopped calibration\n");
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

float getSD(float* samples){
    float mean = 0;
    float SD = 0;
    int i;
    for(i = 0; i<BUFFERSIZE; ++i) mean += samples[i];
    mean /= BUFFERSIZE;
    for(i = 0; i<BUFFERSIZE; ++i) SD = SD +(samples[i]-mean)*(samples[i]-mean);
    SD /= BUFFERSIZE;
    return sqrtf(SD);
}

void doCalibration(void){
    static float meangx = 0;
    static float meangy = 0;
    static float meangz = 0;
    switch (calibrationData.calibrationState) {
        case 0:
            break;
        case 1:
            calibrationData.positionsDone = 0;
            calibrationData.gyroBiasX = calibrationData.gyroBiasY = calibrationData.gyroBiasZ = 0.0;
            printf("Now calibrating gyro - Bell-Boy should be kept absolutely still\n");
            FILLING = 1;
            buffer.numberOfSamplesTaken = 0;
            calibrationData.calibrationState = 2;
            meangx = meangy = meangz = 0.0; 
            break;
        case 2:
            if(FILLING) break; // do nothing while buffer filling
//                printf("GyroSDX:%f\n",getSD(buffer.gx));
            if(getSD(buffer.gx) > 0.1 || getSD(buffer.gy) > 0.1 || getSD(buffer.gz) > 0.1){
                printf("Too much variability in gyro readings - was Bell-Boy still?\nTrying again\n");
                calibrationData.calibrationState = 1;
                break;
            }
            for(int i = 0; i<BUFFERSIZE; ++i) {
                meangx += buffer.gx[i];
                meangy += buffer.gy[i];
                meangz += buffer.gz[i];
            }
            FILLING = 1;
            buffer.numberOfSamplesTaken = 0;
            calibrationData.calibrationState = 21;
            break;
        case 21:
            if(FILLING) break; // do nothing while buffer filling
            if(getSD(buffer.gx) > 0.1 || getSD(buffer.gy) > 0.1 || getSD(buffer.gz) > 0.1){
                printf("Too much variability in gyro readings - was Bell-Boy still?\nTrying again\n");
                calibrationData.calibrationState = 1;
                break;
            }
            for(int i = 0; i<BUFFERSIZE; ++i) {
                meangx += buffer.gx[i];
                meangy += buffer.gy[i];
                meangz += buffer.gz[i];
            }
            FILLING = 1;
            buffer.numberOfSamplesTaken = 0;
            calibrationData.calibrationState = 25;
            break;
        case 25:
            if(FILLING) break; // do nothing while buffer filling
            if(getSD(buffer.gx) > 0.1 || getSD(buffer.gy) > 0.1 || getSD(buffer.gz) > 0.1){
                printf("Too much variability in gyro readings - was Bell-Boy still?\nTrying again\n");
                calibrationData.calibrationState = 1;
                break;
            }
            for(int i = 0; i<BUFFERSIZE; ++i) {
                meangx += buffer.gx[i];
                meangy += buffer.gy[i];
                meangz += buffer.gz[i];
            }
            calibrationData.gyroBiasX = meangx/(3.0*BUFFERSIZE);
            calibrationData.gyroBiasY = meangy/(3.0*BUFFERSIZE);
            calibrationData.gyroBiasZ = meangz/(3.0*BUFFERSIZE);       
            printf("Gyro calibration done.  Now calibrating accelerometers.\nMove Bell-Boy so that it is resting on one of its six faces.\n");
//            printf("%f %f %f\n", calibrationData.gyroBiasX, calibrationData.gyroBiasY, calibrationData.gyroBiasZ);
            calibrationData.stable = 0;
            calibrationData.calibrationState = 3;
            break;
        case 3:
            if(!calibrationData.stable) break;
            calibrationData.stable = 0;
            calibrationData.calibrationState = 4;
            break;
        case 4:
            if(!calibrationData.stable) break;
            printf("Bell-Boy is stable ");
            if(calibrationData.lastAccZ >  0.8 && fabs(calibrationData.lastAccY) < 0.2 && fabs(calibrationData.lastAccX) < 0.2) { printf("and is positioned power switch face up.\n"); calibrationData.calibrationState = 100; break;}
            if(calibrationData.lastAccZ < -0.8 && fabs(calibrationData.lastAccY) < 0.2 && fabs(calibrationData.lastAccX) < 0.2) { printf("and is positioned power switch face down.\n"); calibrationData.calibrationState = 200; break;}
            if(calibrationData.lastAccX >  0.8 && fabs(calibrationData.lastAccY) < 0.2 && fabs(calibrationData.lastAccZ) < 0.2) { printf("and is positioned SD card face down.\n"); calibrationData.calibrationState = 300; break;}
            if(calibrationData.lastAccX < -0.8 && fabs(calibrationData.lastAccY) < 0.2 && fabs(calibrationData.lastAccZ) < 0.2) { printf("and is positioned SD card face up.\n"); calibrationData.calibrationState = 400; break;}
            if(calibrationData.lastAccY >  0.8 && fabs(calibrationData.lastAccX) < 0.2 && fabs(calibrationData.lastAccZ) < 0.2) { printf("and is positioned logo face down.\n"); calibrationData.calibrationState = 500; break;}
            if(calibrationData.lastAccY < -0.8 && fabs(calibrationData.lastAccX) < 0.2 && fabs(calibrationData.lastAccZ) < 0.2) { printf("and is positioned logo face up.\n"); calibrationData.calibrationState = 600; break;}
            printf("but is not resting on one of its faces. Try again.\n");
//            printf("%f %f %f \n",calibrationData.lastAccX,calibrationData.lastAccY,calibrationData.lastAccZ);
            calibrationData.calibrationState = 99;
            break;
        case 5:
            if(calibrationData.positionsDone != 63) {
                if(calibrationData.positionsDone == 62 || calibrationData.positionsDone == 61 || calibrationData.positionsDone == 59 || calibrationData.positionsDone == 55 || calibrationData.positionsDone == 47 || calibrationData.positionsDone == 31){
                    printf("Done. Now move Bell-Boy to the following position:\n");                     
                } else {
                    printf("Done. Now move Bell-Boy to one of the following positions:\n"); 
                }
                if(!(calibrationData.positionsDone & POWERSWUP)) printf("\t- power switch up\n");
                if(!(calibrationData.positionsDone & POWERSWDOWN)) printf("\t- power switch down\n");
                if(!(calibrationData.positionsDone & SDUP)) printf("\t- SD card up\n");
                if(!(calibrationData.positionsDone & SDDOWN)) printf("\t- SD card down\n");
                if(!(calibrationData.positionsDone & LOGOUP)) printf("\t- logo up\n");
                if(!(calibrationData.positionsDone & LOGODOWN)) printf("\t- logo down\n");
                calibrationData.calibrationState = 99; 
                break;
            }
            for (int i = 0; i < 4; i++){
                for (int j = 0; j < 4; j++) calibrationData.stateXtX[i][j] = 0.0;
                calibrationData.stateXtY[i] = 0.0;
            }
            for (int position = 0; position < 6; position++) sensorCalibrationPushSampleForOffsetCalculation(calibrationData.accSamples[position]);
            float accTmp[3];
            sensorCalibrationSolveForOffset(accTmp);
            calibrationData.accBiasX = accTmp[0];
            calibrationData.accBiasY = accTmp[1];
            calibrationData.accBiasZ = accTmp[2];
            for (int i = 0; i < 4; i++){
                for (int j = 0; j < 4; j++) calibrationData.stateXtX[i][j] = 0.0;
                calibrationData.stateXtY[i] = 0.0;
            }
            for (int position = 0; position < 6; position++) {
                float accSample[3];
                accSample[0] = calibrationData.accSamples[position][0] - calibrationData.accBiasX;
                accSample[1] = calibrationData.accSamples[position][1] - calibrationData.accBiasY;
                accSample[2] = calibrationData.accSamples[position][2] - calibrationData.accBiasZ;
                sensorCalibrationPushSampleForScaleCalculation(position / 2, accSample, 1);
            }
            sensorCalibrationSolveForScale(accTmp);
            calibrationData.accTransformMatrix[0] = accTmp[0];
            calibrationData.accTransformMatrix[4] = accTmp[1];
            calibrationData.accTransformMatrix[8] = accTmp[2];
            printf("Calibration done!\n");
            calibrationData.calibrationState = 0;
            break;
        case 99:
            FILLING = 0;
            buffer.numberOfSamplesTaken = 0;
            calibrationData.stable = 0;
            calibrationData.calibrationState = 3;
            break;
        case 100:
            if(calibrationData.positionsDone & POWERSWUP) {printf("This position already done!\nPlease move Bell-Boy so it is resting on one of its other faces.\n"); calibrationData.calibrationState = 99; break;}
            buffer.numberOfSamplesTaken = 0;
            FILLING = 1;
            calibrationData.calibrationState = 101;
            break;
        case 101:
            doAccelerometerCalibration(POWERSWUP);
            break;
        case 200:
            if(calibrationData.positionsDone & POWERSWDOWN) {printf("This position already done!\nPlease move Bell-Boy so it is resting on one of its other faces.\n"); calibrationData.calibrationState = 99; break;}
            buffer.numberOfSamplesTaken = 0;
            FILLING = 1;
            calibrationData.calibrationState = 201;
            break;
        case 201:
            doAccelerometerCalibration(POWERSWDOWN);
            break;
        case 300:
            if(calibrationData.positionsDone & SDDOWN) {printf("This position already done!\nPlease move Bell-Boy so it is resting on one of its other faces.\n"); calibrationData.calibrationState = 99; break;}
            buffer.numberOfSamplesTaken = 0;
            FILLING = 1;
            calibrationData.calibrationState = 301;
            break;
        case 301:
            doAccelerometerCalibration(SDDOWN);
            break;
        case 400:
            if(calibrationData.positionsDone & SDUP) {printf("This position already done!\nPlease move Bell-Boy so it is resting on one of its other faces.\n"); calibrationData.calibrationState = 99; break;}
            buffer.numberOfSamplesTaken = 0;
            FILLING = 1;
            calibrationData.calibrationState = 401;
            break;
        case 401:
            doAccelerometerCalibration(SDUP);
            break;
        case 500:
            if(calibrationData.positionsDone & LOGODOWN) {printf("This position already done!\nPlease move Bell-Boy so it is resting on one of its other faces.\n"); calibrationData.calibrationState = 99; break;}
            buffer.numberOfSamplesTaken = 0;
            FILLING = 1;
            calibrationData.calibrationState = 501;
            break;
        case 501:
            doAccelerometerCalibration(LOGODOWN);
            break;
        case 600:
            if(calibrationData.positionsDone & LOGOUP) {printf("This position already done!\nPlease move Bell-Boy so it is resting on one of its other faces.\n"); calibrationData.calibrationState = 99; break;}
            buffer.numberOfSamplesTaken = 0;
            FILLING = 1;
            calibrationData.calibrationState = 601;
            break;
        case 601:
            doAccelerometerCalibration(LOGOUP);
            break;
    }
}

void doAccelerometerCalibration(uint8_t position){
    if(!calibrationData.stable) {
        printf("Device moved. Bell-Boy must be kept stable. Retrying\n");
        calibrationData.calibrationState = 99;
        return;
    }
    if(FILLING) return; // do nothing while buffer filling
    if(getSD(buffer.ax) > 0.01 || getSD(buffer.ay) > 0.01 || getSD(buffer.az) > 0.01){
        printf("Too much variability in accelerometer readings. Trying again\n");
        calibrationData.calibrationState = 99;
        return;
    }
    switch(position){
        case POWERSWUP:
            addPositionMeans(0);
            break;
        case POWERSWDOWN:
            addPositionMeans(1);
            break;
        case SDDOWN:
            addPositionMeans(2);
            break;
        case SDUP:
            addPositionMeans(3);
            break;
        case LOGODOWN:
            addPositionMeans(4);
            break;
        case LOGOUP:
            addPositionMeans(5);
            break;
    }
    calibrationData.positionsDone |= position;
    calibrationData.calibrationState = 5;
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

void takeSamples(void){
    static float peaks[3] = {0};
    static unsigned int loopcount = 0;

    float fifoData[6];
    int count = readFIFOcount();
    if (count > 4000){ // overflow (or nearly so).  If you see this then stop the calibration and start again
        printf("\nEOVF:\n");
        //clear data paths and reset FIFO (keep i2c disabled)
        writeRegister(ICM20689_USER_CTRL,0x15);
        usleep(1000);
 
        //start FIFO - keep i2c disabled
        writeRegister(ICM20689_USER_CTRL,0x50);
    }
    if(count < 12) return;
    
    while(count >= 12){
        count -= 12;
        readFIFO(fifoData);
        
        fifoData[3] -= calibrationData.gyroBiasX;
        fifoData[4] -= calibrationData.gyroBiasY;
        fifoData[5] -= calibrationData.gyroBiasZ;

        calibrationData.lastAccX = 0.9*calibrationData.lastAccX + 0.1*fifoData[0]; // used for position analysis in doCalibration()
        calibrationData.lastAccY = 0.9*calibrationData.lastAccY + 0.1*fifoData[1];
        calibrationData.lastAccZ = 0.9*calibrationData.lastAccZ + 0.1*fifoData[2];

        for(int k = 0; k < 3; ++k){
            if(fabs(fifoData[3+k]) > peaks[k]) peaks[k] = fabs(fifoData[3+k]);
            if(peaks[k] > 0.9) calibrationData.stable = 0;
        }
        if(FILLING){
            buffer.gx[buffer.numberOfSamplesTaken] = fifoData[3];
            buffer.gy[buffer.numberOfSamplesTaken] = fifoData[4];
            buffer.gz[buffer.numberOfSamplesTaken] = fifoData[5];
            buffer.ax[buffer.numberOfSamplesTaken] = fifoData[0];
            buffer.ay[buffer.numberOfSamplesTaken] = fifoData[1];
            buffer.az[buffer.numberOfSamplesTaken] = fifoData[2];
            buffer.numberOfSamplesTaken += 1;
            if(buffer.numberOfSamplesTaken == BUFFERSIZE) FILLING = 0;
            calibrationData.lastAccX = fifoData[0];
            calibrationData.lastAccY = fifoData[1];
            calibrationData.lastAccZ = fifoData[2];
        }
        if(!(loopcount % 2500)){
            if(peaks[0] < 0.9 && peaks[1] < 0.9 && peaks[2] < 0.9){
                calibrationData.stable = 1; // flag that device is stable
            }
            for(int h = 0; h < 3; ++h) peaks[h] = 0.0;
        }
        loopcount += 1;
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
    bcm2835_i2c_set_baudrate(50000);
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
    SPI_BUFFER[0] = ICM20689_FIFO_R_W | 0x80;
    for(uint8_t i = 1; i<13; ++i){
        SPI_BUFFER[i] = 0;  
    }
    bcm2835_spi_transfern(SPI_BUFFER,13);
    // remapping of axes X->Z Y->X  Z->Y
    values[2]=((uint16_t)SPI_BUFFER[1] << 8) + SPI_BUFFER[2]; 
    values[0]=((uint16_t)SPI_BUFFER[3] << 8) + SPI_BUFFER[4];
    values[1]=((uint16_t)SPI_BUFFER[5] << 8) + SPI_BUFFER[6];
    values[5]=((uint16_t)SPI_BUFFER[7] << 8) + SPI_BUFFER[8]; 
    values[3]=((uint16_t)SPI_BUFFER[9] << 8) + SPI_BUFFER[10];
    values[4]=((uint16_t)SPI_BUFFER[11] << 8) + SPI_BUFFER[12];

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

/*
 * The functions below have been adapted from Cleanflight.
 * https://github.com/iNavFlight/inav and are issued under the following
 * licence.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

void addPositionMeans(int position){
    float mean = 0.0;
    int i;
    for(i = 0; i<BUFFERSIZE; ++i) mean += buffer.ax[i];
    calibrationData.accSamples[position][0] = mean/BUFFERSIZE;
    mean = 0.0;
    for(i = 0; i<BUFFERSIZE; ++i) mean += buffer.ay[i];
    calibrationData.accSamples[position][1] = mean/BUFFERSIZE;
    mean = 0.0;
    for(i = 0; i<BUFFERSIZE; ++i) mean += buffer.az[i];
    calibrationData.accSamples[position][2] = mean/BUFFERSIZE;
}

void sensorCalibrationPushSampleForOffsetCalculation(float sample[3]) {
    calibrationData.stateXtX[0][0] += (float)sample[0] * sample[0];
    calibrationData.stateXtX[0][1] += (float)sample[0] * sample[1];
    calibrationData.stateXtX[0][2] += (float)sample[0] * sample[2];
    calibrationData.stateXtX[0][3] += (float)sample[0];

    calibrationData.stateXtX[1][0] += (float)sample[1] * sample[0];
    calibrationData.stateXtX[1][1] += (float)sample[1] * sample[1];
    calibrationData.stateXtX[1][2] += (float)sample[1] * sample[2];
    calibrationData.stateXtX[1][3] += (float)sample[1];

    calibrationData.stateXtX[2][0] += (float)sample[2] * sample[0];
    calibrationData.stateXtX[2][1] += (float)sample[2] * sample[1];
    calibrationData.stateXtX[2][2] += (float)sample[2] * sample[2];
    calibrationData.stateXtX[2][3] += (float)sample[2];

    calibrationData.stateXtX[3][0] += (float)sample[0];
    calibrationData.stateXtX[3][1] += (float)sample[1];
    calibrationData.stateXtX[3][2] += (float)sample[2];
    calibrationData.stateXtX[3][3] += 1;

    float squareSum = ((float)sample[0] * sample[0]) + ((float)sample[1] * sample[1]) + ((float)sample[2] * sample[2]);
    calibrationData.stateXtY[0] += sample[0] * squareSum;
    calibrationData.stateXtY[1] += sample[1] * squareSum;
    calibrationData.stateXtY[2] += sample[2] * squareSum;
    calibrationData.stateXtY[3] += squareSum;
}

void sensorCalibrationPushSampleForScaleCalculation(int axis, float sample[3], int target){
    for (int i = 0; i < 3; i++) {
        float scaledSample = sample[i] / (float)target;
        calibrationData.stateXtX[axis][i] += scaledSample * scaledSample;
        calibrationData.stateXtX[3][i] += scaledSample * scaledSample;
    }

    calibrationData.stateXtX[axis][3] += 1;
    calibrationData.stateXtY[axis] += 1;
    calibrationData.stateXtY[3] += 1;
}

void sensorCalibrationSolveForOffset(float result[3]){
    float beta[4];
    sensorCalibration_SolveLGS(calibrationData.stateXtX, beta, calibrationData.stateXtY);
    for (int i = 0; i < 3; i++) {
        result[i] = beta[i] / 2;
    }
}

void sensorCalibrationSolveForScale(float result[3]){
    float beta[4];
    sensorCalibration_SolveLGS(calibrationData.stateXtX, beta, calibrationData.stateXtY);
    for (int i = 0; i < 3; i++) {
        result[i] = sqrtf(beta[i]);
    }
}

// solve linear equation
// https://en.wikipedia.org/wiki/Gaussian_elimination
static void sensorCalibration_SolveLGS(float A[4][4], float x[4], float b[4]) {
    int i;
    float y[4];

    sensorCalibration_gaussLR(A);

    for (i = 0; i < 4; ++i) {
        y[i] = 0;
    }

    sensorCalibration_ForwardSubstitution(A, y, b);
    sensorCalibration_BackwardSubstitution(A, x, y);
}

static void sensorCalibration_gaussLR(float mat[4][4]) {
    uint8_t n = 4;
    int i, j, k;
    for (i = 0; i < 4; i++) {
        // Determine R
        for (j = i; j < 4; j++) {
            for (k = 0; k < i; k++) {
                mat[i][j] -= mat[i][k] * mat[k][j];
            }
        }
        // Determine L
        for (j = i + 1; j < n; j++) {
            for (k = 0; k < i; k++) {
                mat[j][i] -= mat[j][k] * mat[k][i];
            }
            mat[j][i] /= mat[i][i];
        }
    }
}

void sensorCalibration_ForwardSubstitution(float LR[4][4], float y[4], float b[4]) {
    int i, k;
    for (i = 0; i < 4; ++i) {
        y[i] = b[i];
        for (k = 0; k < i; ++k) {
            y[i] -= LR[i][k] * y[k];
        }
        //y[i] /= MAT_ELEM_AT(LR,i,i); //Do not use, LR(i,i) is 1 anyways and not stored in this matrix
    }
}

void sensorCalibration_BackwardSubstitution(float LR[4][4], float x[4], float y[4]) {
    int i, k;
    for (i = 3 ; i >= 0; --i) {
        x[i] = y[i];
        for (k = i + 1; k < 4; ++k) {
            x[i] -= LR[i][k] * x[k];
        }
        x[i] /= LR[i][i];
    }
}

