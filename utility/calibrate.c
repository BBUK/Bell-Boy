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

* This program executes an initial calibration of the ICM20689 devices, both in terms of sample rate and
* accelerometer bias and scale.  It uses a rather good method of calibration disclosed in these papers
* http://www.dis.uniroma1.it/~pretto/papers/tpm_icra2014.pdf
* http://www.dis.uniroma1.it/~pretto/papers/pg_imeko2014.pdf

* Code and installation instructions for the imu_tk calibration routines this program relies on are here:
* https://bitbucket.org/alberto_pretto/imu_tk 
* 
* The general idea is that this program is run and a SAMP: command is executed a few times until you are happy
* that the result is stable.  The CALI: command is then executed and this pushes a timestamp and accelerometer 
* data to /data/samples/CALIBRATIONDATA.  The device is kept still (i.e. resting on a table) for no 
* less than 50 seconds and is then put into around 30-40 different resting positions each lasting about 4 seconds.
* After this process, issue the STCA: command.  This program will then calculate the calibration data and save
* it to the Arduino EEPROM memory.  From there it is accessed whenever the grabber file is started. 
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

const unsigned int LOOPSLEEP = 50000;  // main loop processes every 0.05 sec.  25 samples should be in FIFO.  Max number of samples in FIFO is 341.  Should be OK.

struct {
    float samplePeriod;
    float accBiasX;
    float accBiasY;
    float accBiasZ;
    float accScaleX;
    float accScaleY;
    float accScaleZ;
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;
    float gyroScaleX;
    float gyroScaleY;
    float gyroScaleZ;
} calibrationData = {.samplePeriod = 0.008, .accBiasX=0, .accBiasY=0, .accBiasZ = 0,
                    .accScaleX=1, .accScaleY=1, .accScaleZ=1, .gyroBiasX=0, .gyroBiasY=0,
                    .gyroBiasZ=0, .gyroScaleX=1, .gyroScaleY=1, .gyroScaleZ=1};

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
                fdCalibrationData = fopen("CALIBRATIONDATA","w");
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
					fflush(fdCalibrationData);
					fclose(fdCalibrationData);
					fdCalibrationData = NULL;
				}
				CALIBRATING = 0;
				printf("Starting calibration...\n");
				system("./BBimu_tk CALIBRATIONDATA");
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
    12=accBiasY, 13=accBiasZ, 14=accScaleX, 15=accScaleY, 16=accScaleZ,
    17=gyroBiasX, 18=gyroBiasY, 19=gyroBiasZ, 20=gyroScaleX, 21=gyroScaleY,
    22=gyroScaleZ, 5=accBiasXYZ(all in one go 12 bytes), 6=accScaleXYZ(all in one go 12 bytes)
    7=gyroBiasXYZ(all in one go 12 bytes), 8=gyroScaleXYZ(all in one go 12 bytes))
*/
void save(void){
    printf("Loading calibration...\n");
    FILE *fdCalib;
    unsigned char linein[100];
    float dummy1,dummy2;
    fdCalib = fopen("BBacc.calib","r");
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &calibrationData.accScaleX, &dummy1, &dummy2);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &dummy1, &calibrationData.accScaleY, &dummy2);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &dummy1, &dummy2, &calibrationData.accScaleZ);

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
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &calibrationData.gyroScaleX, &dummy1, &dummy2);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &dummy1, &calibrationData.gyroScaleY, &dummy2);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f %f %f", &dummy1, &dummy2, &calibrationData.gyroScaleZ);

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;

    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.gyroBiasX);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.gyroBiasY);
    if(fgets(linein, sizeof(linein), fdCalib) == NULL) return;
    sscanf(linein, "%f", &calibrationData.gyroBiasZ);

    fclose(fdCalib);
    
    printf("Sample period %f\n",calibrationData.samplePeriod);
    printf("Acc bias %f %f %f\n",calibrationData.accBiasX,calibrationData.accBiasY,calibrationData.accBiasZ);
    printf("Acc scale %f %f %f\n",calibrationData.accScaleX,calibrationData.accScaleY,calibrationData.accScaleZ);
    printf("Gyro bias %f %f %f\n",calibrationData.gyroBiasX,calibrationData.gyroBiasY,calibrationData.gyroBiasZ);
    printf("Gyro scale %f %f %f\n",calibrationData.gyroScaleX,calibrationData.gyroScaleY,calibrationData.gyroScaleZ);

    char answer;
    printf("Do values look sane? (CTRL_C to exit in the next 20 secs) \n");
    usleep(20000000);
    
	printf("Writing to prom...\n");
    arduinoWriteFloat(10,calibrationData.samplePeriod);
    arduinoWriteFloat(11,calibrationData.accBiasX);
    arduinoWriteFloat(12,calibrationData.accBiasY);
    arduinoWriteFloat(13,calibrationData.accBiasZ);
    arduinoWriteFloat(14,calibrationData.accScaleX);
    arduinoWriteFloat(15,calibrationData.accScaleY);
    arduinoWriteFloat(16,calibrationData.accScaleZ);
    arduinoWriteFloat(17,calibrationData.gyroBiasX);
    arduinoWriteFloat(18,calibrationData.gyroBiasY);
    arduinoWriteFloat(19,calibrationData.gyroBiasZ);
    arduinoWriteFloat(20,calibrationData.gyroScaleX);
    arduinoWriteFloat(21,calibrationData.gyroScaleY);
    arduinoWriteFloat(22,calibrationData.gyroScaleZ);

    printf("Verifying...\n");
    I2C_BUFFER[0]=10;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 10 (samplePeriod)
    bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
    if(calibrationData.samplePeriod != extractFloat(0)) printf("SamplePeriod: sent %f received %f \n", calibrationData.samplePeriod, extractFloat(0));

    I2C_BUFFER[0]=5;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 5 (accBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.accBiasX != extractFloat(0)) printf("accBiasX: sent %f received %f \n", calibrationData.accBiasX, extractFloat(0));
    if(calibrationData.accBiasY != extractFloat(4)) printf("accBiasY: sent %f received %f \n", calibrationData.accBiasY, extractFloat(4));
    if(calibrationData.accBiasZ != extractFloat(8)) printf("accBiasZ: sent %f received %f \n", calibrationData.accBiasZ, extractFloat(8));

    I2C_BUFFER[0]=6;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 6 (accScaleXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.accScaleX != extractFloat(0)) printf("accScaleX: sent %f received %f \n", calibrationData.accScaleX, extractFloat(0));
    if(calibrationData.accScaleY != extractFloat(4)) printf("accScaleY: sent %f received %f \n", calibrationData.accScaleY, extractFloat(4));
    if(calibrationData.accScaleZ != extractFloat(8)) printf("accScaleZ: sent %f received %f \n", calibrationData.accScaleZ, extractFloat(8));

    I2C_BUFFER[0]=7;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 7 (gyroBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.gyroBiasX != extractFloat(0)) printf("gyroBiasX: sent %f received %f \n", calibrationData.gyroBiasX, extractFloat(0));
    if(calibrationData.gyroBiasY != extractFloat(4)) printf("gyroBiasY: sent %f received %f \n", calibrationData.gyroBiasY, extractFloat(4));
    if(calibrationData.gyroBiasZ != extractFloat(8)) printf("gyroBiasZ: sent %f received %f \n", calibrationData.gyroBiasZ, extractFloat(8));

    I2C_BUFFER[0]=8;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 8 (gyroScaleXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    if(calibrationData.gyroScaleX != extractFloat(0)) printf("gyroScaleX: sent %f received %f \n", calibrationData.gyroScaleX, extractFloat(0));
    if(calibrationData.gyroScaleY != extractFloat(4)) printf("gyroScaleY: sent %f received %f \n", calibrationData.gyroScaleY, extractFloat(4));
    if(calibrationData.gyroScaleZ != extractFloat(8)) printf("gyroScaleZ: sent %f received %f \n", calibrationData.gyroScaleZ, extractFloat(8));
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
    usleep(750000); 
}

float extractFloat(uint8_t index){
    floatConv._bytes[0] = I2C_BUFFER[index];
    floatConv._bytes[1] = I2C_BUFFER[index+1];
    floatConv._bytes[2] = I2C_BUFFER[index+2];
    floatConv._bytes[3] = I2C_BUFFER[index+3];
    return floatConv._float;
}

void fifoTimer(void){
    float result1 = 0.0, result2 = 0.0;
    float dummy[6];
    result1 = timer(BCM2835_SPI_CS0);
    result2 = timer(BCM2835_SPI_CS1);
    while (readFIFOcount(BCM2835_SPI_CS0) != 0) readFIFO(BCM2835_SPI_CS0, dummy);
    while (readFIFOcount(BCM2835_SPI_CS1) != 0) readFIFO(BCM2835_SPI_CS1, dummy);
   
    calibrationData.samplePeriod = (result1 > result2) ? result1 : result2;
}

float timer(uint8_t device){
    struct timeval start, stop;
    int cco, loops;
    float dummy[6];
    float result = 0.0;
    loops = 4;
    for(int i = 0; i < loops; ++i){
        while (readFIFOcount(device) != 0) readFIFO(device, dummy);
        while (readFIFOcount(device) == 0);
        gettimeofday(&start, NULL);
        readFIFO(device, dummy);
        while (readFIFOcount(device) < (196 * 12)) usleep(4000);  // sleep for about 2 periods
        while(1){
            usleep(100);
            cco = readFIFOcount(device);
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
    return (result/(200*loops))*4;
}

void pullData(void){
    float fifo_data_1[6];
    float fifo_data_2[6];
    float combined_data[6];
    static unsigned int angleCorrection = 0;

    int count_1 = readFIFOcount(BCM2835_SPI_CS0);
    int count_2 = readFIFOcount(BCM2835_SPI_CS1);

    if (count_1 > 4000 || count_2 >= 4000){ // overflow (or nearly so)
        printf("\nEOVF:\n");
        // some sort of recovery algorithm would be nice...
    }
    // ditch a sample if one fifo is running faster then the other
    if ((count_1 - count_2) >=24){
        readFIFO(BCM2835_SPI_CS0, fifo_data_1);
        count_1 -= 12;
    } else if ((count_2 - count_1) >=24){
        readFIFO(BCM2835_SPI_CS1, fifo_data_2);
        count_2 -= 12;
    }

    if(count_1 < 48 || count_2 < 48) return;
    
    while(count_1 >=48 && count_2 >= 48){
        count_1 -= 48;
        count_2 -= 48;
        for(int i = 0; i < 6; ++i) combined_data[i] = 0;
        for(int i = 0; i < 4; ++i){
            readFIFO(BCM2835_SPI_CS0, fifo_data_1);
            readFIFO(BCM2835_SPI_CS1, fifo_data_2);
            combined_data[0] += (fifo_data_2[0] + fifo_data_1[0]) / 2.0; // Ax
            combined_data[1] += (fifo_data_2[1] + fifo_data_1[1]) / 2.0; // Ay
            combined_data[2] += (fifo_data_2[2] + fifo_data_1[2]) / 2.0; // Az
            combined_data[3] += (fifo_data_2[3] + fifo_data_1[3]) / 2.0; // Gx
            combined_data[4] += (fifo_data_2[4] + fifo_data_1[4]) / 2.0; // Gy
            combined_data[5] += (fifo_data_2[5] + fifo_data_1[5]) / 2.0; // Gz
        }
        for(int i = 0; i < 6; ++i) combined_data[i] /= 4;
        
        if (CALIBRATING) fprintf (fdCalibrationData,"%+.8e %+.8e %+.8e %+.8e %+.8e %+.8e %+.8e\n", calibrationTimestamp, combined_data[3] * DEGREES_TO_RADIANS_MULTIPLIER,combined_data[4] * DEGREES_TO_RADIANS_MULTIPLIER,combined_data[5] * DEGREES_TO_RADIANS_MULTIPLIER, combined_data[0] * g0 ,combined_data[1] * g0,combined_data[2] * g0);
        calibrationTimestamp += calibrationData.samplePeriod;
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
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);

   //reset devices
    writeRegisterBits(BCM2835_SPI_CS0,ICM20689_PWR_MGMT_1,0xFF,0x80);
    writeRegisterBits(BCM2835_SPI_CS1,ICM20689_PWR_MGMT_1,0xFF,0x80);
    usleep(100000);
 
   //disable i2c
    writeRegisterBits(BCM2835_SPI_CS0,ICM20689_USER_CTRL,0xFF,0x10);
    writeRegisterBits(BCM2835_SPI_CS1,ICM20689_USER_CTRL,0xFF,0x10);

    if(readRegister(BCM2835_SPI_CS0,ICM20689_WHO_AM_I) != 0x98){
      printf("Unable to read from device 0. \n");
      exit(1);        
    }
    if(readRegister(BCM2835_SPI_CS1,ICM20689_WHO_AM_I) != 0x98){
      printf("Unable to read from device 1. \n");
      exit(1);        
    }
    // bring out of sleep, set clksel (to PLL) and disable temperature sensor
    writeRegister(BCM2835_SPI_CS0,ICM20689_PWR_MGMT_1,0x09);
    writeRegister(BCM2835_SPI_CS1,ICM20689_PWR_MGMT_1,0x09);
    usleep(30000); 

    //configure DLPF
    writeRegister(BCM2835_SPI_CS0,ICM20689_CONFIG,0x01);
    writeRegister(BCM2835_SPI_CS1,ICM20689_CONFIG,0x01);

    //full scale accelerometer range to 4g
    writeRegister(BCM2835_SPI_CS0,ICM20689_ACCEL_CONFIG,0x08);
    writeRegister(BCM2835_SPI_CS1,ICM20689_ACCEL_CONFIG,0x08);

    //full scale gyro range to 500deg/s
    writeRegister(BCM2835_SPI_CS0,ICM20689_GYRO_CONFIG,0x08);
    writeRegister(BCM2835_SPI_CS1,ICM20689_GYRO_CONFIG,0x08);

    //set sample rate divider we want 500Hz assuming base clock is 1KHz
    writeRegister(BCM2835_SPI_CS0,ICM20689_SMPLRT_DIV,0x01);
    writeRegister(BCM2835_SPI_CS1,ICM20689_SMPLRT_DIV,0x01);

    //set FIFO size to 4K (this setting does not appear in the V1 datasheet!)
    writeRegister(BCM2835_SPI_CS0,ICM20689_ACCEL_CONFIG_2,0xC0);
    writeRegister(BCM2835_SPI_CS1,ICM20689_ACCEL_CONFIG_2,0xC0);
    
    //clear data paths and reset FIFO (keep i2c disabled)
    writeRegister(BCM2835_SPI_CS0,ICM20689_USER_CTRL,0x15);
    writeRegister(BCM2835_SPI_CS1,ICM20689_USER_CTRL,0x15);
    usleep(1000);

    //select registers for FIFO
    writeRegister(BCM2835_SPI_CS0,ICM20689_FIFO_EN,0x78);
    writeRegister(BCM2835_SPI_CS1,ICM20689_FIFO_EN,0x78);
    
    //start FIFO - keep i2c disabled
    writeRegister(BCM2835_SPI_CS0,ICM20689_USER_CTRL,0x50);
    writeRegister(BCM2835_SPI_CS1,ICM20689_USER_CTRL,0x50);
    
}

uint16_t readFIFOcount(uint8_t device){
    return ((uint16_t)readRegister(device,ICM20689_FIFO_COUNTH) << 8) + readRegister(device,ICM20689_FIFO_COUNTL); 
}

void readFIFO(uint8_t device, float* values){
    float temp0;
    bcm2835_spi_chipSelect(device);
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

void writeRegister(uint8_t device, uint8_t reg, uint8_t value){
    bcm2835_spi_chipSelect(device);
    SPI_BUFFER[0] = reg; //& 0x7F;  // set high bit for a write
    SPI_BUFFER[1] = value;
    bcm2835_spi_transfern(SPI_BUFFER,2);
}

void writeRegisterBits(uint8_t device, uint8_t reg, uint8_t mask, uint8_t value){
    uint8_t readValue;
    readValue = readRegister(device,reg);
    bcm2835_spi_chipSelect(device);
    SPI_BUFFER[0] = reg; // & 0x7F;  // clear high bit for a write
    SPI_BUFFER[1] = (readValue & mask) | value;
    bcm2835_spi_transfern(SPI_BUFFER,2);
}

uint8_t readRegister(uint8_t device, uint8_t reg){
    bcm2835_spi_chipSelect(device);
    SPI_BUFFER[0] = reg | 0x80;  //set high bit for a read
    SPI_BUFFER[1] = 0;
    bcm2835_spi_transfern(SPI_BUFFER,2);
    return SPI_BUFFER[1];
}
