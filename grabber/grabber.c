//gcc grabber.c -o grabber -lm -lbcm2835 -Wall -O3

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
* This program forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
* bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
* hardware is currently a Pi Zero running Arch Linux.
*
* The results of the "engineering" and "experimentation" work on the bell system are described here.
* Although it will work straight from the command line, this program is intended to be started 
* by Websocketd https://github.com/joewalnes/websocketd
* On startup, it initialises the IMU (setup() function which, checks if the device is already calibrated by checking
* the getSavedSata() function and checking if the device has awoken from sleep) and the main loop is entered. 
* The main loop starts calling doCalibration() if the device is uncalibrated and pushData() if it is.
* If uncalibrated, there is first a zero calibration (needs about 30 seconds of stability to bias-adjust the gyros and to zero
* the bell position (this is done by the pullAndTransform() function, called by doCalibration()). After zeroing, 
* the user is invited to pull up and set the bell.  Once the bell is set the user is invited to start ringing a few times.
* One the bell is being rung full circle, doCalibration() starts saving data. Once the bell has gone through the calibration 
* routine, doCalibration() calculates the effect of gravity and the rope on the bell.  See calculateError() and calculateArea(). 
* Once this calibration is complete pushData() is called in the main loop.  pushData() calls populateBuffer() 
* which takes data from the IMU and converts it to calibrated orientation and rotational velocity.
* Although this data is not pushed to the browser until the user signals that they want to record a run, 
* the data is always processed as it allows for some gyro calibration (pullAndTransform() function) and for the filters to settle down.
*  
* The main loop:
*       *  checks the Arduino for battery life and reports this to the browser
*       *  checks stdin for commands from the browser (see below) and processes these commands
*       *  calls pushData() to see if any data needs processing and (if a run is being recorded), pushing samples to the browser
* setup:
*       * configures the IMU device over SPI
*       * pulls calibration data from the Arduino over I2C
* pushData
*       * calls pullData
*       * pushes angle, rate and acceleration to the browser
* populateBuffer is called by pushData
*       * checks to see if enough data is in the two IMUs' FIFOs to be pulled and processed
*       * calls pullAndTransform to actually pull the data and process it into angles
*       * puts processed data (angle, angular rate and acceleration) into a circular buffer
*       * acceleration is calculated using a Savitsky-Golay filter ( savGol() )
* savgol
*       * processes angular rate data to create angular acceleration data
* pullAndTransform
*       * called by pullData and by doCalibration(), takes data from the IMUs (over SPI) averages the data 
*       * from each of them and then adjusts for bias, scale and misalignment, checks to see if the bell 
*       * has been stationary for a while and if it has it takes the opportunity to recalculate the gyro biasses
*       * calls calculate() function to update rotation quaternion and Euler angles
* calculate
*       * called by pullAndTransform()
*       * straightforward Mahony complementary filter as wriiten in quaternion form by Madgwick
* startRun
*       * called when the user has indicated that a recording should be started
*       * works out which way the bell went up (the device can be mounted either side of the headstock)
*       * initialises the circular buffer
*       * sets the RUNNING global to true (signals to pushData() that data can be pushed to browser) 
* 
* There are the following possible command sent by the user's browser.  
* Commands are received on stdin and output is on stdout (which is what Websocketd expects):
* (a)   STRT:[filename] Start recording a session with a bell.  The bell
*       must be at stand and (reasonably) stationary otherwise this
*       will return ESTD: or EMOV: either of which signals the browser 
*       to stop expecting data.  If Filename is not provided then a
*       default filename of "Unnamed_[date]" is used.
* (b)   STOP: stops a recording in progress.  This also saves off the
*       collected data into the relevant file in /data/samples/
* (c)   LDAT: a request for data from the browser, only works when there
*       is a recording in progress - deprecated.  Does nothing.
* (d)   FILE: get a listing of the previous recordings stored in
*       /data/samples/ .  Used to display the selection box to the user.
* (e)   LOAD:[filename] used to transmit a previous recording to the browser
* (f)   DATE:[string] sets the date on the device to the date on the browser 
*             string is unixtime (microsecs since 1/1/70)
* (g)   SAMP: requests the current sample period
* (h)   SHDN: tells the device to shutdown
* (i)   EYEC: starts eye candy demo
* (j)   STEC: stops eye candy demo
* (k)   STND: standalone (playback-only) operation.
* (l)   PLRD: play and record operation.
* (m)   BELS: bell data. Format: numberofbells(int),CPM(float),openHandstroke(float), manualtimefromBDC(float)
* (o)   SLEP:[number] go to sleep for [number] of half hour units.
*
* There are the following responses back to the user's browser:
* (i)    STPD: tells the browser that the device has sucessfully stopped sampling
* (ii)   ESTP: signals an error in the stop process (an attempt to stopped when not started)
* (iii)  LIVE:[string] the string contains the data recorded by the device in the format
*             "A:[angle],R:[rate],C:[acceleration],D:[data],V:[value]".  Angle, rate and 
*             acceleration are ordinary floating point numbers.  Angle is in degrees, rate is in degrees/sec
*             and acceleration is in degrees/sec/sec.  Data and value indicate certain other charateristics,
*             such as BDC being reached, a reversal in rotational rate (i.e. at the top) and clapper
*             strikes (see pushData() for details).
* (iv)   NDAT: indicates that there is no current data to send to the browser (deprecated)
* (v)    SAMP: returns the sample period
* (vi)   EIMU: IMU not detected or some IMU related failure
* (vii)  ESTD: bell not at stand (aborts started session)
* (viii) EMOV: bell moving when session started (the bell has to be stationary at start)
* (ix)   ESTR: internal error related to data / filenames (shouldn't happen)!
* (x)    DATA:[string]  chunk of data from a previously stored file.  In same format as LIVE:
* (xi)   EOVF: fifo overflow flagged.
* (xii)  FILE:[filename] in response to FILE: a list of the previous recordings in /data/samples
* (xiii) STRT: in response to STRT: indicates a sucessful start.
* (xiv)  LFIN:[number] indicates the end of a file download and the number of samples sent
* (xv)   TEST:[number] current orientation
* (xvi)  BATT:[number] battery percentage
* (xvii) EYEC:[roll],[pitch].[yaw],[qw],[qx],[qy],[qz] returns angles and quaternion for eye candy demo 
* (xviii)EWAI: Wait for initial calibration (zeroing)
* (xix)  EPUL: Wait for bell to be rung up
* (xx)   ESWI: Bell being pulled up - recording for gravity calibration
* (xxi)  ESET: Ask user to set bell
* (xxii) EFIL: end sending of FILE: data
* (xxiii)EDEF: Some problem with loading calibration data. Default (uncalibrated) valued used.
*/

#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>
#include <linux/types.h>
#include <stdint.h>
#include <dirent.h>
#include "ICM20689.h"
#include <bcm2835.h>

#ifndef NULL
#define NULL 0
#endif

// Manchester estimation.  Use gcalculator.xlsx to work out for your latitude.
#define g0 9.8137   

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 

FILE *fd_write_out;

// calculated by calculate() function
float q0 = 1;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

int RUNNING = 0;
int EYECANDY = 0;

char SPI_BUFFER[256];
char I2C_BUFFER[16];

// used by pushData() to create a single string to push to the browser and to save locally
char READ_OUTBUF[1500];
char READ_OUTBUF_LINE[150];
int READ_OUTBUF_COUNT;
char local_outbuf[4000];
int local_count = 0;

// used to hold the filename of the session that the user wants to load
char FILENAME[60];

const unsigned int LOOPSLEEP = 50000;  // main loop processes every 0.05 sec.  25 samples should be in FIFO.  Max number of samples in FIFO is 341.  Should be OK.
unsigned int LOOPCOUNT = 0;

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
    float tareValue;
    unsigned int torn;
    float *calibrationData;
    unsigned int calibrationDataCount;
    float gravityValue;
    float ropeValueH;
    float ropeValueB;
    int direction;
    unsigned int calibrationState;
    unsigned int calibrationStrokeCount;
    unsigned int stable;
    unsigned int lastBDC;
    float lastAngularVelocity;
    float predictedHandstrokeStrikeAngle;
    float predictedBackstrokeStrikeAngle;
    unsigned int manualStrike;
    unsigned int manualTimefromBDC;
    unsigned int pushCounter;
    unsigned int lastStrikePushCounter;
    unsigned int lastStrike; // 2 = last chime was at backstroke, 1= last chime at handstroke
    unsigned int bellNumber;
    float CPM;
    float openHandstroke;
} grabberData = {.samplePeriod = 0.002, .accBiasX=0, .accBiasY=0, .accBiasZ = 0,
                    .accTransformMatrix[0]=1, .accTransformMatrix[1]=0, .accTransformMatrix[2]=0, .accTransformMatrix[3]=0,
                    .accTransformMatrix[4]=1, .accTransformMatrix[5]=0, .accTransformMatrix[6]=0, .accTransformMatrix[7]=0,
                    .accTransformMatrix[8]=1, .gyroBiasX=0, .gyroBiasY=0, .gyroBiasZ=0, .gyroTransformMatrix[0]=1, 
                    .gyroTransformMatrix[1]=0, .gyroTransformMatrix[2]=0, .gyroTransformMatrix[3]=0, .gyroTransformMatrix[4]=1,
                    .gyroTransformMatrix[5]=0, .gyroTransformMatrix[6]=0, .gyroTransformMatrix[7]=0, .gyroTransformMatrix[8]=1,
                    .tareValue = 0.0 , .torn = 0, .gravityValue = 0.0, .ropeValueH = 0, .ropeValueB = 0, .direction = 0, .calibrationState = 0, 
                    .calibrationDataCount = 0, .calibrationStrokeCount = 0, .stable = 0, .bellNumber = 6, .CPM = 31.0, 
                    .openHandstroke = 1.0, .manualStrike = 0, .manualTimefromBDC = 300, .lastStrike = 2, .predictedHandstrokeStrikeAngle=300.0,
                    .predictedBackstrokeStrikeAngle=60.0};

// only really important when run from command line
volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

// number of samples taken before pushing out - only a quarter of these are actually pushed out
// because the sample rate here (500Hz) is four times larger than the rate data is sent to the browser
#define PUSHBATCH 80
// must be bigger than SAVGOLHALF + PUSHBATCH
#define BUFFERSIZE 512
// circular buffer used by populateBuffer() to add samples and pushData() or doCalibration() to consume them
struct {
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    float angleBuffer[BUFFERSIZE];
    float rateBuffer[BUFFERSIZE];
    float accelBuffer[BUFFERSIZE];
} fifo;

// used in savGol() (called by the populateBuffer() function) and doCalibration() and pushData()
// see savGol() for coefficients and justification for this particular filter length.  
#define SAVGOLLENGTH  61
#define SAVGOLHALF 30 // = floor(SAVGOLLENGTH/2.0)

int main(int argc, char const *argv[]){
    char linein[50];
    char command[6];
    char details[42];
    char oscommand[125];
    int entries;
    float smoothedBattery = 0.0;

    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
    
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
 
    setup();
    if(grabberData.calibrationState !=7){
        if(!grabberData.calibrationData) grabberData.calibrationData = (float *)malloc(sizeof(float) * 24000 * 3);  // allocate buffer for later (see doCalibration() - buffer freed there)
        if(!grabberData.calibrationData){
            printf("Unable to allocate memory for calibration storage.");
            exit(1);
        }
}
    while(!sig_exit){
        usleep(LOOPSLEEP);
		LOOPCOUNT += 1;
		if(LOOPCOUNT == 1 || !(LOOPCOUNT % 200)){ // check every 10 secs but also the first time through
			I2C_BUFFER[0]=2;
			bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 2 (power)
			bcm2835_i2c_read(I2C_BUFFER,2); usleep(100);
			unsigned int result = (I2C_BUFFER[0]<<8)+I2C_BUFFER[1];
            if(result & 0x8000) { system("/usr/bin/sync && /usr/bin/shutdown -P now"); system("/usr/bin/touch /boot/battlow");}
            if(LOOPCOUNT == 1) {
                smoothedBattery = (result & 0x7FFF);
            } else {
                smoothedBattery = 0.8*smoothedBattery + 0.2*(result & 0x7FFF);
            }
			if(smoothedBattery > 530){
				printf("BATT:5\n");
			} else if(smoothedBattery > 519){
				printf("BATT:10\n");
			} else if(smoothedBattery > 509){
				printf("BATT:20\n");
			} else if(smoothedBattery > 503){
				printf("BATT:30\n");
			} else if(smoothedBattery > 500){
				printf("BATT:40\n");
			} else if(smoothedBattery > 494){
				printf("BATT:50\n");
			} else if(smoothedBattery > 491){
				printf("BATT:60\n");
			} else if(smoothedBattery > 481){
				printf("BATT:70\n");
			} else if(smoothedBattery > 473){
				printf("BATT:80\n");
			} else if(smoothedBattery > 464){
				printf("BATT:90\n");
			} else if(smoothedBattery > 456){
				printf("BATT:95\n");
			} else {
				printf("BATT:100\n");
			}
		}
      
        if(grabberData.calibrationState == 7){
            pushData();
        } else {
            doCalibration();
        }

        if(EYECANDY) printf("EYEC:%+06.1f, %+06.1f, %+06.1f, %+07.4f, %+07.4f, %+07.4f, %+07.4f\n", roll, pitch, yaw, q0, q1, q2, q3);

        while(fgets(linein, sizeof(linein), stdin ) != NULL) {
            entries = sscanf(linein, "%5s%[^\n]", command, details);
            if(entries < 1) {
                fprintf( stderr, "\nError reading: %s \n", linein );
                continue;
            }
            if(strcmp("DATE:", command) == 0 && entries == 2  && strlen(details) < 12){
                sprintf(oscommand,"/usr/bin/date --set=\"$(date --date='@%s')\" && /usr/bin/touch /var/lib/systemd/clock", details);
                system(oscommand);
                continue;
            }
            
            if(strcmp("SAMP:", command) == 0) {
                printf("SAMP:%f\n",grabberData.samplePeriod*4.0);
                continue;
            }
            
            if(strcmp("FILE:", command) == 0) {
                DIR *d;
                struct dirent *dir;
                d = opendir("/data/samples");
                if(d){
                    while ((dir = readdir(d)) != NULL) {
                        if(dir->d_type == DT_REG) printf("FILE:%s\n", dir->d_name);
                    }
                closedir(d);
                printf("EFIL:\n");
                }
                continue;
            }
            if(strcmp("EYEC:", command) == 0) {
                if(RUNNING) continue;
                EYECANDY = 1;
                continue;
            }
            if(strcmp("STEC:", command) == 0) {
                EYECANDY = 0;
                continue;
            }
            if(strcmp("STND:", command) == 0) {  // standalone operation
                grabberData.gravityValue = 0;
                grabberData.calibrationState = 7;
                grabberData.torn = 3;
                continue;
            }
            if(strcmp("PLRD:", command) == 0) {  // play and record operation
                grabberData.gravityValue = 0;
                grabberData.calibrationState = 0;
                grabberData.torn = 0;
                getSavedData();
                continue;
            }
            if(strcmp("BELS:", command) == 0) {  // set number of bells.  Not used here but recorded in file
                if(entries == 2  && strlen(details) > 0){
                    grabberData.bellNumber = 0;
                    grabberData.CPM = 0;
                    grabberData.openHandstroke = 1;
                    char *ent = strtok(details,","); 
                    if(ent) grabberData.bellNumber = atoi(ent);
                    ent = strtok(NULL,",");
                    if(ent) grabberData.CPM = atof(ent);
                    ent = strtok(NULL,",");
                    if(ent) grabberData.openHandstroke = atof(ent);
                    ent = strtok(NULL,",");
                    if(ent) grabberData.manualTimefromBDC = atoi(ent); // if "Auto" is sent then that results in zero here
                    if(grabberData.manualTimefromBDC == 0){
                        grabberData.manualStrike = 0;
                    } else {
                        grabberData.manualStrike = 1;
                    }
                }
                continue;
            }

            if(strcmp("STRT:", command) == 0) {
                if((grabberData.torn < 3) || (grabberData.calibrationState != 7)){
//                    fprintf(stderr, "Still calibrating...\n");
                    printf("ESTR:\n");
                    printf("STPD:\n");                    
                }
                if(entries == 2  && strlen(details) < 34){  // use chosen filename
                    sprintf(FILENAME, "/data/samples/%s", details);
                } else {                                    // otherwise make a filename up
                    struct tm *timenow;
                    time_t now = time(NULL);
                    timenow = gmtime(&now);
                    strftime(FILENAME, sizeof(FILENAME), "/data/samples/Unnamed_%d-%m-%y_%H%M", timenow);
                }
                fd_write_out = fopen(FILENAME,"w");
                if(fd_write_out == NULL) {
                    fprintf(stderr, "Could not open file for writing\n");
                    printf("ESTR:\n");
                    printf("STPD:\n");
                    continue;
                }
                startRun();
                continue;
            }
            if(strcmp("STOP:", command) == 0) {
                if(fd_write_out != NULL){
                    if(local_count != 0) {fputs(local_outbuf, fd_write_out); local_count = 0;}
                    fflush(fd_write_out);
                    fclose(fd_write_out);
                    fd_write_out = NULL;
                }
                RUNNING = 0;
                printf("STPD:%d\n", grabberData.pushCounter/4);
                continue;
            }
            if(strcmp("LOAD:", command) == 0 && RUNNING == 0) {
                if(entries == 2  && strlen(details) < 34){
                    sprintf(FILENAME, "/data/samples/%s", details);
                } else {
                    printf("ESTR:\n");
                    continue;
                }
                FILE *fd_read_in;
                fd_read_in = fopen(FILENAME,"r");
                if(fd_read_in == NULL) {
                    fprintf(stderr, "Could not open file for reading\n");
                    printf("ESTR:\n");
                    continue;
                } else {
                    int counter = 0;
                    READ_OUTBUF_COUNT = 0;
                    while(fgets(READ_OUTBUF_LINE, sizeof(READ_OUTBUF_LINE), fd_read_in ) != NULL) {
                        size_t ln = strlen(READ_OUTBUF_LINE);
                        if(ln > 0 && READ_OUTBUF_LINE[ln -1] == '\n') READ_OUTBUF_LINE[ln - 1] = '\0';   // get rid of \n
//                        READ_OUTBUF_LINE[strcspn(READ_OUTBUF_LINE, "\n")] = 0;
                        if (READ_OUTBUF_COUNT + strlen(READ_OUTBUF_LINE) + 7 > (sizeof(READ_OUTBUF) -2)) {
                            printf("%s\n", READ_OUTBUF);
                            READ_OUTBUF_COUNT = 0;
                        }
                        READ_OUTBUF_COUNT += sprintf(&READ_OUTBUF[READ_OUTBUF_COUNT],"DATA:%s",READ_OUTBUF_LINE);
                        counter += 1;
                        if((counter % 50) == 0) pushData(); // collect data from FIFO whilst doing this
                    }
                    if(READ_OUTBUF_COUNT != 0) printf("%s\n",READ_OUTBUF);
                    printf("LFIN:%d\n", counter);
                }
                fclose(fd_read_in);
                continue;
            }
            if(strcmp("SHDN:", command) == 0) {
                system("/usr/bin/sync && /usr/bin/shutdown -P now");
                system("/usr/bin/touch /boot/shutdowncommand");
                continue;
            }
            if(strcmp("SLEP:", command) == 0) {
                uint8_t sleepTime = atoi(details);
                union {
                    float    _float;
                    uint8_t  _bytes[sizeof(float)];
                } floatConv;


                floatConv._float = grabberData.ropeValueH;
                I2C_BUFFER[0]=6; // ropeValueH
                I2C_BUFFER[1] = floatConv._bytes[0];
                I2C_BUFFER[2] = floatConv._bytes[1];
                I2C_BUFFER[3] = floatConv._bytes[2];
                I2C_BUFFER[4] = floatConv._bytes[3];
                bcm2835_i2c_write(I2C_BUFFER,5); 
                usleep(750000);

                floatConv._float = grabberData.ropeValueB;
                I2C_BUFFER[0]=7; // ropeValueB
                I2C_BUFFER[1] = floatConv._bytes[0];
                I2C_BUFFER[2] = floatConv._bytes[1];
                I2C_BUFFER[3] = floatConv._bytes[2];
                I2C_BUFFER[4] = floatConv._bytes[3];
                bcm2835_i2c_write(I2C_BUFFER,5); 
                usleep(750000);

                if(grabberData.calibrationState == 7){
                    floatConv._float = grabberData.gravityValue;
                } else {
                    floatConv._float = -360; // nothing recorded, null value flagged
                }
                I2C_BUFFER[0]=8; // gravityValue
                I2C_BUFFER[1] = floatConv._bytes[0];
                I2C_BUFFER[2] = floatConv._bytes[1];
                I2C_BUFFER[3] = floatConv._bytes[2];
                I2C_BUFFER[4] = floatConv._bytes[3];
                bcm2835_i2c_write(I2C_BUFFER,5); 
                usleep(750000);

                if(grabberData.torn == 3){
                    floatConv._float = grabberData.tareValue;
                } else {
                    floatConv._float = -360; // nothing recorded, null value flagged
                }
                I2C_BUFFER[0]=9; // tareValue
                I2C_BUFFER[1] = floatConv._bytes[0];
                I2C_BUFFER[2] = floatConv._bytes[1];
                I2C_BUFFER[3] = floatConv._bytes[2];
                I2C_BUFFER[4] = floatConv._bytes[3];
                bcm2835_i2c_write(I2C_BUFFER,5); 
                usleep(750000);

                I2C_BUFFER[0]=2; // go to sleep
                I2C_BUFFER[1] = (char)sleepTime;
                bcm2835_i2c_write(I2C_BUFFER,2); 
                system("/usr/bin/sync && /usr/bin/shutdown -P now");
                continue;
            }

            printf("ESTR:\n");
            fprintf(stderr, "Unrecognised command: %s \n", command);
        }
    }
    system("/usr/bin/touch /var/lib/systemd/clock");
    return 0;
}

void doCalibration(void){
    static unsigned int count = 0;

    int FIFOcount = readFIFOcount();
    if (FIFOcount > 4000){
        printf("\nEOVF:\n");
        writeRegister(ICM20689_USER_CTRL,0x15); // reset FIFO
        usleep(1000);
        writeRegister(ICM20689_USER_CTRL,0x50);
        return;
    }

    if(FIFOcount < 48) return;
   
    while(FIFOcount >=12){
        FIFOcount -= 12;
        pullAndTransform();
        count += 1;

        switch (grabberData.calibrationState) {
            case 0:
                if(grabberData.stable && grabberData.torn == 3){ // bell is stable and zeroed tell user to ring bell up
                    grabberData.calibrationState = 1;
                }
                if(!(count % 2500)) printf("EWAI:\n");  // sends signal to browser to indicate that the sensor is still calibrating zero
                break;
            case 1:
                if((roll-grabberData.tareValue) < -30.0 || (roll-grabberData.tareValue) > +30.0){ // bell is ringing up, tell user to set bell normally
                    grabberData.calibrationState = 2;
                }
                if(!(count % 2500)) printf("EPUL:\n");  // sends signal to browser to indicate that bell needs to be pulled up
                break;
            case 2:
                if(grabberData.stable && fabs(roll-grabberData.tareValue) > 160 && fabs(roll-grabberData.tareValue) < 178) {
                    grabberData.calibrationState = 3; // bell now rung up and stable tell user to start ringing
                    if(roll-grabberData.tareValue < 0) {
                        grabberData.direction = -1;
                    } else {
                        grabberData.direction = +1;
                    }
                    for(int i = 0; i< BUFFERSIZE; ++i){ // initialise with dummy data for savgol alignment
                        fifo.rateBuffer[i] = 0;
                        fifo.accelBuffer[i] = 0;
                        fifo.angleBuffer[i] = grabberData.direction*(roll-grabberData.tareValue)-180.0;
                    }
                    fifo.head = SAVGOLHALF;
                    fifo.tail = 0;
                }
                if(!(count % 2500)) printf("ESET:\n");  // sends signal to browser to indicate that the bell needs to be set at handstroke
                break;
            case 3:
                if(!grabberData.stable && grabberData.lastAngularVelocity > 100) {
                    grabberData.calibrationState = 4; // bell now swinging, start collecting data
                    grabberData.calibrationStrokeCount = 0; 
                }
                if(!(count % 2500)) printf("ESWI:%d\n",(5-grabberData.calibrationStrokeCount) == 0 ? 1 : (5-grabberData.calibrationStrokeCount));  // sends signal to browser to indicate that bell needs to be swung
                break;
            case 4:
                populateBuffer();
                if(!(count % 2500)) printf("ESWI:%d\n",(5-grabberData.calibrationStrokeCount) == 0 ? 1 : (5-grabberData.calibrationStrokeCount));  // sends signal to browser to indicate that swings need to continue
                if(fifo.available < SAVGOLLENGTH) break;
                for(int counter = fifo.available; counter > SAVGOLHALF; --counter){
                    if((fifo.angleBuffer[fifo.tail] < 345 && fifo.angleBuffer[fifo.tail] > 315 && fifo.rateBuffer[fifo.tail] < 0) ||
                        (fifo.angleBuffer[fifo.tail] < 280 && fifo.angleBuffer[fifo.tail] > 260 && fifo.rateBuffer[fifo.tail] < 0) ||
                        (fifo.angleBuffer[fifo.tail] < 45 && fifo.angleBuffer[fifo.tail] > 15 && fifo.rateBuffer[fifo.tail] > 0)){
                        if(grabberData.calibrationDataCount < 24000){
                            grabberData.calibrationData[grabberData.calibrationDataCount*3] = fifo.angleBuffer[fifo.tail];
                            grabberData.calibrationData[(grabberData.calibrationDataCount*3)+1] = fifo.rateBuffer[fifo.tail];
                            grabberData.calibrationData[(grabberData.calibrationDataCount*3)+2] = fifo.accelBuffer[fifo.tail];
                            grabberData.calibrationDataCount += 1;
                        }
                    }
                    if(fifo.accelBuffer[fifo.tail] * fifo.accelBuffer[(fifo.tail == 0) ? BUFFERSIZE-1 : fifo.tail -1] <= 0.0 && 
                        fifo.angleBuffer[fifo.tail] > 170.0 && fifo.angleBuffer[fifo.tail] < 190.0) {
                        grabberData.calibrationStrokeCount +=1;
                        if(grabberData.calibrationStrokeCount > 5) grabberData.calibrationState = 5;  // requisite number of strokes done, tell user to set bell
                    }
                    fifo.tail = (fifo.tail + 1) % BUFFERSIZE;
                    fifo.available -= 1;
                }
                break;
            case 5:
                if(grabberData.stable && fabs(roll-grabberData.tareValue) > 160 && fabs(roll-grabberData.tareValue) < 178) {
                    grabberData.calibrationState = 6; // bell now set again, proceed to calculations
                }
                if(!(count % 2500)) printf("ESET:\n");  // sends signal to browser to indicate that the bell can be set at handstroke
                break;
            case 6:
                grabberData.gravityValue = 0;
                for(int guessLog = 14; guessLog >= 0; --guessLog){
                    if(calculateError(grabberData.gravityValue + pow(2,guessLog)) < 0) grabberData.gravityValue += pow(2,guessLog);
                }
                pullAndTransform(); // just to empty any data held on IMU
                float currentBestArea=calculateAreaB(64);
                float high,low;
                grabberData.ropeValueB = 64.0;
                for(int guessLog = 5; guessLog >= 0; --guessLog){
                    high = calculateAreaB(grabberData.ropeValueB + pow(2,guessLog));
                    low  = calculateAreaB(grabberData.ropeValueB - pow(2,guessLog));
                    if(low <= high){
                        if(low <= currentBestArea){
                            grabberData.ropeValueB -= pow(2,guessLog);
                            currentBestArea = low;
                        }
                    } else {
                        if(high < currentBestArea){
                            grabberData.ropeValueB += pow(2,guessLog);
                            currentBestArea = high;
                        }
                    }
                }

                pullAndTransform();

                currentBestArea=calculateAreaH(64);
                grabberData.ropeValueH = 64.0;
                for(int guessLog = 5; guessLog >= 0; --guessLog){
                    high = calculateAreaH(grabberData.ropeValueH + pow(2,guessLog));
                    low  = calculateAreaH(grabberData.ropeValueH - pow(2,guessLog));
                    if(low <= high){
                        if(low <= currentBestArea){
                            grabberData.ropeValueH -= pow(2,guessLog);
                            currentBestArea = low;
                        }
                    } else {
                        if(high < currentBestArea){
                            grabberData.ropeValueH += pow(2,guessLog);
                            currentBestArea = high;
                        }
                    }
                }
                grabberData.calibrationState = 7;
                pullAndTransform();

                if(grabberData.calibrationData) {
                    free(grabberData.calibrationData);
                    grabberData.calibrationData = NULL;
                }
                FILE *fdGravity;
                fdGravity = fopen("/tmp/BBgravity","w");
                if(fdGravity != NULL) {
                    fprintf(fdGravity,"%+08.1f,%+08.1f,%+08.1f\n", grabberData.gravityValue, grabberData.ropeValueB, grabberData.ropeValueH);
                    fclose(fdGravity);
                }
        }
    }
}

void startRun(void){
    if(fabs(roll-grabberData.tareValue) < 160 || fabs(roll-grabberData.tareValue) > 178){
        printf("ESTD:%+07.1f\n",roll-grabberData.tareValue); // bell not at stand
        return;
    }

    if((roll-grabberData.tareValue) < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        grabberData.direction = -1;
    } else {
        grabberData.direction = +1;
    }

    for(int i = 0; i< BUFFERSIZE; ++i){ // initialise with dummy data for savgol alignment
        fifo.rateBuffer[i] = 0;
        fifo.accelBuffer[i] = 0;
        fifo.angleBuffer[i] = grabberData.direction*(roll-grabberData.tareValue)-180.0;
    }
    fifo.head = SAVGOLHALF;
    fifo.tail = 0;
    fifo.available = SAVGOLHALF;
    grabberData.pushCounter = 0;
    grabberData.lastStrikePushCounter = 0;
    grabberData.lastStrike = 2;
    grabberData.lastBDC = 1000000; // just a random large number until first BDC is recorded

    RUNNING = 1;
	EYECANDY = 0;
    return;
}

void pushData(void){
    char local_outbuf_line[150];
    char remote_outbuf_line[150];
    static char remote_outbuf[4000]; // not necessary to be static as this buffer is cleared before function exits
    int remote_count = 0;
    int strike;
    float accn, taccn;
    static float nudgeAngle= 0;  // delete this and code below if not needed
    static int nudgeCount = 0;
    static int switchCount = 0;
    static char detailsString[32];
    static unsigned int data = 0;
    static float value = 0.0;

    int count = readFIFOcount();

    if (count > 4000){ // overflow (or nearly so)
        printf("\nEOVF:\n"); //shouldn't happen
        //clear data paths and reset FIFO (keep i2c disabled)
        writeRegister(ICM20689_USER_CTRL,0x15);
        usleep(1000);
 
        //restart FIFO - keep i2c disabled
        writeRegister(ICM20689_USER_CTRL,0x50);
        return;
    }

    if(count < 12) return;

    while(count >=12){
        count -= 12;
        pullAndTransform();
        if(RUNNING) populateBuffer();
    }

    if(!RUNNING) return;

    // Need to have at least SAVGOLHALF + 1 samples ready in buffer before sending anything out (see populateBuffer() for reasoning) 
    // but don't go through this unless there are also an additional PUSHBATCH samples available (the value of PUSHBATCH has been 
    // chosen to minimise processor usage, by sending data in batches rather than piecemeal).
    // Samples are pushed into the rateBuffer and angleBuffer circular buffers by populateBuffer() function.
    // Available and tail variables are are used to keep track of circular buffer (as is the head
    // variable but that is updated by the populateBuffer() function)
    if(fifo.available <= PUSHBATCH + SAVGOLHALF) return;

    for(int counter = fifo.available; counter > SAVGOLHALF; --counter){ // always leave SAVGOLHALF in fifo because acceleration data is that amount behind the other data

        unsigned int lastTail = (fifo.tail == 0) ? BUFFERSIZE-1 : fifo.tail -1;

        // This bit allowed for slight angle drift.  It does this knowing that BDC = 180 degrees.
        // and that is the point when the bell's angular acceleration is zero.  Drift is so low
        // that this is more trouble than it's worth.  Left in and reported but not used.
        // Also testing shows BDC angle is slightly off (0.5 degree) than that recorded when the bell is at rest.  Rope weight?
        taccn = fifo.accelBuffer[fifo.tail];
        accn = fifo.accelBuffer[lastTail];
        if(taccn * accn <= 0.0 && fifo.angleBuffer[fifo.tail] > 170.0 && fifo.angleBuffer[fifo.tail] < 190.0  && nudgeCount == 0) {
            float nudgeFactor = accn /(accn - taccn);
            nudgeAngle = (fifo.angleBuffer[lastTail] + nudgeFactor*(fifo.angleBuffer[fifo.tail]-fifo.angleBuffer[lastTail])) - 180.0;
            nudgeCount = 10;
            grabberData.lastBDC = grabberData.pushCounter;
            value = nudgeAngle; // report BDC point
            data = 8;
        } 
        if(nudgeCount != 0) nudgeCount -= 1; // don't do this twice in one half stroke - allows for a bit of noise, not really needed
        
        float raccn = 0.0;
        if (fifo.angleBuffer[fifo.tail] > 270.0) {
            raccn = taccn - grabberData.gravityValue*sin(fifo.angleBuffer[fifo.tail]*DEGREES_TO_RADIANS_MULTIPLIER) - ((270.0-fifo.angleBuffer[fifo.tail])*grabberData.ropeValueB/120.0);
        } else if (fifo.angleBuffer[fifo.tail] < 90.0){
            raccn = taccn - grabberData.gravityValue*sin(fifo.angleBuffer[fifo.tail]*DEGREES_TO_RADIANS_MULTIPLIER) - ((90.0 -fifo.angleBuffer[fifo.tail])*grabberData.ropeValueH/120.0);
        } else {
            raccn = taccn - grabberData.gravityValue*sin(fifo.angleBuffer[fifo.tail]*DEGREES_TO_RADIANS_MULTIPLIER);
        }
        
        if(fifo.rateBuffer[fifo.tail] * fifo.rateBuffer[lastTail] <= 0.0 && switchCount == 0 && grabberData.pushCounter > 40  && (fifo.angleBuffer[fifo.tail] > 270 || fifo.angleBuffer[fifo.tail] < 90)) { // detect when bell changes direction
            data = 7;
            value = (grabberData.pushCounter)/4;
            switchCount = 5;
        } 
        if (switchCount != 0 && fabs(fifo.rateBuffer[fifo.tail]) > 100.0) switchCount -= 1;

        strike = dingDong();
        if(strike != 0){
            data = strike;
            value = (float)(grabberData.pushCounter - grabberData.lastStrikePushCounter)*grabberData.samplePeriod;
            grabberData.lastStrikePushCounter = grabberData.pushCounter;
        }

        if(!(grabberData.pushCounter % 4)) { // only push out once every 4
            switch(grabberData.pushCounter/4){
                case 0:
                    strcpy(detailsString, ", #Gravity");
                    value = grabberData.gravityValue;
                    break;
                case 1:
                    strcpy(detailsString, ", #Number of Bells");
                    value = grabberData.bellNumber;
                    break;
                case 2:
                    strcpy(detailsString, ", #CPM");
                    value = grabberData.CPM;
                    break;
                case 3:
                    strcpy(detailsString, ", #Open handstroke factor");
                    value = grabberData.openHandstroke;
                    break;
                case 4:
                    strcpy(detailsString, ", #Strike time from BDC");
                    value = grabberData.manualTimefromBDC;
                    break;
                case 5:
                    strcpy(detailsString, ", #Tare value");
                    value = grabberData.tareValue;
                    break;
                case 6:
                    strcpy(detailsString, ", #Rope value back");
                    value = grabberData.ropeValueB;
                    break;
                case 7:
                    strcpy(detailsString, ", #Rope value hand");
                    value = grabberData.ropeValueH;
                    break;
                case 8:
                    strcpy(detailsString, "");
            }

            sprintf(remote_outbuf_line, "LIVE:A:%+6.1f,R:%+6.1f,C:%+7.1f,D:%d,V:%+10.2f\n", fifo.angleBuffer[fifo.tail], fifo.rateBuffer[fifo.tail], raccn, data, value);
            if (remote_count + strlen(remote_outbuf_line) > (sizeof remote_outbuf -2)) {
                printf("%s",remote_outbuf);
                remote_count = 0;
            }

            remote_count += sprintf(&remote_outbuf[remote_count],remote_outbuf_line);     
            sprintf(local_outbuf_line,"A:%+6.1f,R:%+6.1f,C:%+7.1f,D:%d,V:%+10.2f %s\n", fifo.angleBuffer[fifo.tail], fifo.rateBuffer[fifo.tail], raccn, data, value, detailsString);
            if (local_count + strlen(local_outbuf_line) > (sizeof local_outbuf -2)) {
                fputs(local_outbuf, fd_write_out);
                fflush(fd_write_out);
                local_count = 0;
            }
            local_count += sprintf(&local_outbuf[local_count],local_outbuf_line);
            data = 0;
            value = 0.0;
        }

        fifo.tail = (fifo.tail + 1) % BUFFERSIZE;
        fifo.available -= 1;
        grabberData.pushCounter += 1;

//        fprintf(fd_write_out,"A:%+07.1f,R:%+07.1f,C:%+07.1f,P:%+07.1f,Y:%+07.1f,AX:%+06.3f,AY:%+06.3f,AZ:%+06.3f,GX:%+07.1f,GY:%+07.1f,GZ:%+07.1f,X:%+06.3f,Y:%+06.3f,Z:%+06.3f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang, pitch, yaw, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2],a[0],a[1],a[2]);
    }
    if(remote_count != 0) {printf("%s",remote_outbuf); remote_count = 0;}
}

void populateBuffer(void){
    static unsigned int angleCorrection = 0;

    // need to correct reported angles to match the system we are 
    // using 0 degrees = bell at balance at handstroke, 
    // 360 degrees = bell at balance at backstroke. 180 degrees = BDC. 

    unsigned int lastHead = (fifo.head == 0) ? BUFFERSIZE-1 : fifo.head -1;
    float angle = grabberData.direction*(roll-grabberData.tareValue)-180.0;
    angle += angleCorrection;
    if((fifo.angleBuffer[lastHead] - angle) > 270){
        if(angleCorrection < 720 ) { // should always be true
            angleCorrection += 360; 
            angle += 360;
        }
    } else if ((fifo.angleBuffer[lastHead] - angle) < -270){
        if(angleCorrection > 0){ // should always be true
        angleCorrection -= 360; 
        angle -= 360;
        }
    }

    // I calculate angular accelerations from the rates reported by gyro using a Savitsky-Golay
    // filter set to differentiate.  Optimal filter length determined by experiment.
    // Use of S-G filters means that the acceleration measurement is SAVGOLHALF
    // behind the angle and rate measurements.  As a result the fifo buffer should not be emptied
    // so that there are fewer than SAVGOLHALF samples left in it ( see pushBatch() and doCalibration() )
    // In addition there is a delay introduced between the bell moving and an acceleration result 
    // being available for sending to the browser of (mean average) 0.002s * SAVGOLHALF.
    fifo.rateBuffer[fifo.head] = grabberData.lastAngularVelocity * grabberData.direction;
    fifo.angleBuffer[fifo.head] = angle;
    unsigned int currentAccelPosition = (fifo.head < SAVGOLHALF) ? (fifo.head + BUFFERSIZE) - SAVGOLHALF: fifo.head - SAVGOLHALF;
    fifo.accelBuffer[currentAccelPosition] = savGol(currentAccelPosition);
    fifo.head = (fifo.head + 1) % BUFFERSIZE;
    fifo.available += 1;
    if(fifo.available >= BUFFERSIZE) {
        printf("EOVF:\n");  // circular buffer full - should never happen
        fifo.available -= 1;
    }
}

void pullAndTransform(void){
    static float peaks[3] = {0};
    static float averages[3] = {0};
    static unsigned int count = 0;
    float fifoData[6];

    // Although the browser get samples at 125Hz, the ICM20689s are set up 
    // to push data samples out at 500Hz.  The angles and quaternion 
    // are computed at the higher rate as some filters operate better at
    // higher sample rates (although I have not tested whether the Mahony filter
    // (see calculate function) is actually any better).  Four sets
    // of samples are processed by this function at a time.  The browser will only see data
    // derived from last of these samples.
    readFIFO(fifoData);
    
    // get average gyro rates - used for dynamic bias compensation calculation
    for(int j = 0; j < 3; ++j){
        averages[j] += fifoData[3+j];
    }

    fifoData[0] -= grabberData.accBiasX;
    fifoData[1] -= grabberData.accBiasY;
    fifoData[2] -= grabberData.accBiasZ;

    fifoData[3] -= grabberData.gyroBiasX;
    fifoData[4] -= grabberData.gyroBiasY;
    fifoData[5] -= grabberData.gyroBiasZ;

    // used for bias compensation calculation, checks to see if bell is at rest
    // by recording a peak gyro movement over the measurement period (4000 samples = 8 secs)
    for(int k = 0; k < 3; ++k){
        if(fabs(fifoData[3+k]) > peaks[k]) peaks[k] = fabs(fifoData[3+k]);
        if(peaks[k] > 2.1) grabberData.stable = 0;
    }

    // transform data using transform matrix (from imu_tk and collected from Arduino via setup function)
    fifoData[0] = fifoData[0]*grabberData.accTransformMatrix[0]
                + fifoData[1]*grabberData.accTransformMatrix[1]
                + fifoData[2]*grabberData.accTransformMatrix[2];

    fifoData[1] = fifoData[0]*grabberData.accTransformMatrix[3]
                + fifoData[1]*grabberData.accTransformMatrix[4]
                + fifoData[2]*grabberData.accTransformMatrix[5];

    fifoData[2] = fifoData[0]*grabberData.accTransformMatrix[6]
                + fifoData[1]*grabberData.accTransformMatrix[7]
                + fifoData[2]*grabberData.accTransformMatrix[8];

    fifoData[3] = fifoData[3]*grabberData.gyroTransformMatrix[0]
                + fifoData[4]*grabberData.gyroTransformMatrix[1]
                + fifoData[5]*grabberData.gyroTransformMatrix[2];

    fifoData[4] = fifoData[3]*grabberData.gyroTransformMatrix[3]
                + fifoData[4]*grabberData.gyroTransformMatrix[4]
                + fifoData[5]*grabberData.gyroTransformMatrix[5];

    fifoData[5] = fifoData[3]*grabberData.gyroTransformMatrix[6]
                + fifoData[4]*grabberData.gyroTransformMatrix[7]
                + fifoData[5]*grabberData.gyroTransformMatrix[8];

    count +=1;

    if(!(count % 4000)){ // check bias every 8 seconds
        if(peaks[0] < 2.1 && peaks[1] < 2.1 && peaks[2] < 2.1){  // no peaks in gyro rate so can take the last 8 seconds as stationary
            grabberData.gyroBiasX = averages[0]/4000.0;  // adjust biasses
            grabberData.gyroBiasY = averages[1]/4000.0;
            grabberData.gyroBiasZ = averages[2]/4000.0;
            grabberData.stable = 1; // flag that bell is stable
            // Initial (bell down) gyro calibration is done in four stages (each lasting 8 seconds). 
            // First, measure biasses.
            // Second, apply measured biasses.
            // Third, a further period to allow the readings to settle.
            // Fourth, take "zero" tare value and save.
            // The measured angles take a few seconds to settle down once the biasses are applied, hence the third stage.

            if (grabberData.torn < 3 && fabs(roll) < 20.0) { // if bell is down (or within 20 degrees of vertical) take that as the tare value
                grabberData.torn +=1;
                if (grabberData.torn == 3) {
                    grabberData.tareValue = roll;
                    FILE *fdTare;
                    fdTare = fopen("/tmp/BBtare","w");
                    if(fdTare != NULL) {
                        fprintf(fdTare,"%+07.4f\n", grabberData.tareValue);
                        fclose(fdTare);
                    }
                }
            }
        }
        for(int h = 0; h < 3; ++h) { averages[h] = 0.0; peaks[h] = 0.0; }
    }

    // This function calculates angles and a quaternion from the gyro and accelerometer measurements. The q0, q1,q2,q3, roll, pitch and yaw globals
    // are updated by this function (hence no return value)
    calculate(fifoData[3],fifoData[4],fifoData[5], fifoData[0],fifoData[1],fifoData[2],grabberData.samplePeriod);
    grabberData.lastAngularVelocity = fifoData[3]; // "x" gyro rate (roll) needed elsewhere (which is either pushData() or doCalibration()).
    return;  
}

float savGol(unsigned int currentPosition){

// from scipy.signal import savgol_coeffs
// savgol_coeffs(61,2,deriv=1,use="dot")
static float savGolCoefficients[] = {
       -1.58646219e-03, -1.53358012e-03, -1.48069804e-03, -1.42781597e-03,
       -1.37493390e-03, -1.32205182e-03, -1.26916975e-03, -1.21628768e-03,
       -1.16340561e-03, -1.11052353e-03, -1.05764146e-03, -1.00475939e-03,
       -9.51877314e-04, -8.98995241e-04, -8.46113168e-04, -7.93231095e-04,
       -7.40349022e-04, -6.87466949e-04, -6.34584876e-04, -5.81702803e-04,
       -5.28820730e-04, -4.75938657e-04, -4.23056584e-04, -3.70174511e-04,
       -3.17292438e-04, -2.64410365e-04, -2.11528292e-04, -1.58646219e-04,
       -1.05764146e-04, -5.28820730e-05,  1.47215954e-16,  5.28820730e-05,
        1.05764146e-04,  1.58646219e-04,  2.11528292e-04,  2.64410365e-04,
        3.17292438e-04,  3.70174511e-04,  4.23056584e-04,  4.75938657e-04,
        5.28820730e-04,  5.81702803e-04,  6.34584876e-04,  6.87466949e-04,
        7.40349022e-04,  7.93231095e-04,  8.46113168e-04,  8.98995241e-04,
        9.51877314e-04,  1.00475939e-03,  1.05764146e-03,  1.11052353e-03,
        1.16340561e-03,  1.21628768e-03,  1.26916975e-03,  1.32205182e-03,
        1.37493390e-03,  1.42781597e-03,  1.48069804e-03,  1.53358012e-03,
        1.58646219e-03};

    unsigned int windowStartPosition = (currentPosition < SAVGOLHALF) ? (currentPosition + BUFFERSIZE) - SAVGOLHALF: currentPosition - SAVGOLHALF;
    float savGolResult=0;
    for(int i = 0; i < SAVGOLLENGTH; ++i){
        savGolResult += savGolCoefficients[i] * fifo.rateBuffer[(windowStartPosition +i) % BUFFERSIZE];
    }
    if(grabberData.samplePeriod != 0) savGolResult /= grabberData.samplePeriod;
    return(savGolResult);
}

// These functions calculate the area under the acceleration profile of the bell.  There is an effect (probably) caused by the 
// weight of the rope.  the difference (error) between a sine wave of amplitude "guess" and the
// acceleration profile of the bell.  Used for calibration during ringing-up.
float calculateAreaB(float guess){
    float area = 0.0;
    float angleWidth,adjustedHeight;
    for(unsigned int i = 0; i < grabberData.calibrationDataCount-1; ++i){
        // Only calculate the rope weight effect going down at backstroke 
        if(grabberData.calibrationData[(i*3)] < 340 && grabberData.calibrationData[(i*3)] > 320 && grabberData.calibrationData[(i*3)+1] < 0){
            angleWidth = fabs(grabberData.calibrationData[(i*3)] - grabberData.calibrationData[(i*3)+3]);
            adjustedHeight = grabberData.calibrationData[(i*3)+2] - grabberData.gravityValue*sin(grabberData.calibrationData[(i*3)]*DEGREES_TO_RADIANS_MULTIPLIER) - (270.0-grabberData.calibrationData[(i*3)])*guess/120.0;
            if(adjustedHeight > 0) adjustedHeight *= 50.0;
//            printf("%d, %f, %f, %f, %f %f\n", i, angleWidth,adjustedHeight, grabberData.calibrationData[(i*3)], grabberData.calibrationData[(i*3)+3], grabberData.calibrationData[(i*3)] - grabberData.calibrationData[(i*3)+3]);
            area += fabs(angleWidth*adjustedHeight);
        }
    }
    return area;
}

float calculateAreaH(float guess){
    float area = 0.0;
    float angleWidth,adjustedHeight;
    for(unsigned int i = 0; i < grabberData.calibrationDataCount-1; ++i){
        // Only calculate the rope weight effect going down at handstroke
        if(grabberData.calibrationData[(i*3)] > 20 && grabberData.calibrationData[(i*3)] < 40 && grabberData.calibrationData[(i*3)+1] > 0){
            angleWidth = fabs(grabberData.calibrationData[(i*3)] - grabberData.calibrationData[(i*3)+3]);
            adjustedHeight = grabberData.calibrationData[(i*3)+2] - grabberData.gravityValue*sin(grabberData.calibrationData[(i*3)]*DEGREES_TO_RADIANS_MULTIPLIER) - (90.0-grabberData.calibrationData[(i*3)])*guess/120.0;
            if(adjustedHeight < 0) adjustedHeight *= 50.0;
//            printf("%d, %f, %f, %f, %f %f\n", i, angleWidth,adjustedHeight, grabberData.calibrationData[(i*3)], grabberData.calibrationData[(i*3)+3], grabberData.calibrationData[(i*3)] - grabberData.calibrationData[(i*3)+3]);
            area += fabs(angleWidth*adjustedHeight);
        }
    }
    return area;
}

// This function calculates the difference (error) between a sine wave of amplitude "guess" and the
// acceleration profile of the bell.  Used for calibration during ringing-up.
float calculateError(float guess){
    float count = 0.0;
    float error = 0.0;
    for(unsigned int i = 0; i < grabberData.calibrationDataCount; ++i){
        // Only calculate gravity effect going down at handstroke about where the rope is fully unwound because of fewer other effects.
        // such as vibration from strike and pulley/garter hole/rope weight interaction. 
        if(grabberData.calibrationData[(i*3)] < 280 && grabberData.calibrationData[(i*3)] > 260 && grabberData.calibrationData[(i*3)+1] < 0){
            count += 1;
            error += -guess*sin(grabberData.calibrationData[(i*3)]*DEGREES_TO_RADIANS_MULTIPLIER) + grabberData.calibrationData[(i*3)+2];
        }
    }
    if(count != 0){
        error /= count;
    } else {
        error = 0;
    }
    return error;
}

// This function works out when the bell has been struck by the clapper
// The data is passed to the browser which works out inter-strike timings.
// Quite a a bit of experimentation here.  The function works out a base level
// volatility (rate of change of acceleration) of the bell system and when 
// this jumps suddenly (by 1.5x), a strike is recorded.
// Tested on heavy and light bells and produces sane and consistent results +- 10ms or so.
// Need to test on plain bearing bells
// returns 0 = nothing, 1=handstroke, 2=backstroke, 3=fake handstroke, 4=fake backstroke
unsigned int dingDong(){
    static float rollingValues[5] = {8,8,8,8,8};
    static float rollingRates[5] = {0.0,0.0,0.0,0.0,0.0};
    static unsigned int rollingPosition = 0;
    static float baseSD = 0.0;
    static unsigned int strikeCount = 0;

// savgol_coeffs(5,2,deriv=2,use="dot")
    static float dingDongCoefficients[] = { 0.28571429, -0.14285714, -0.28571429, -0.14285714,  0.28571429 };

    if(grabberData.manualStrike){ // doing manual chimes (eg clapper tied)
        if(((float)(grabberData.pushCounter - grabberData.lastBDC)*grabberData.samplePeriod) > (grabberData.manualTimefromBDC/1000.0)){
            if((grabberData.lastStrike == 1) && (fifo.angleBuffer[fifo.tail] < 160) && (fifo.rateBuffer[fifo.tail] < -20)){
                grabberData.lastStrike = 2;
                return(2);
            }
            if((grabberData.lastStrike == 2) && (fifo.angleBuffer[fifo.tail] > 200) && (fifo.rateBuffer[fifo.tail] > 20)){
                grabberData.lastStrike = 1;
                return(1);
            }
        }
        return(0);
    }
    
    rollingRates[(rollingPosition + 2) % 5] = fifo.rateBuffer[fifo.tail];
    
    float savGolResult = 0;
    
    for(int i = 0; i < 5; ++i){
        savGolResult += dingDongCoefficients[i] * rollingRates[(rollingPosition + i) % 5];
    }

    savGolResult *= 50;
    
    rollingValues[rollingPosition] = savGolResult;
    rollingPosition = (rollingPosition + 1) % 5;
    
    float SD = sqrt(rollingValues[0]*rollingValues[0] + rollingValues[1]*rollingValues[1] + rollingValues[2]*rollingValues[2] + rollingValues[3]*rollingValues[3] + rollingValues[4]*rollingValues[4]);
    
    // calculate base level of volatility and apply a bit of smoothing
    if(fifo.angleBuffer[fifo.tail] > 100 && fifo.angleBuffer[fifo.tail] < 260) baseSD = 0.8*baseSD + 0.2*SD;

    // assumption here is that the bell will always strike at the same angle
    if((strikeCount % 2) && (SD > 1.5*baseSD) && (fifo.angleBuffer[fifo.tail] < 70) && (fifo.rateBuffer[fifo.tail] < -20)){
        strikeCount += 1;
        if(strikeCount < 16){
            grabberData.predictedBackstrokeStrikeAngle = (grabberData.predictedBackstrokeStrikeAngle + fifo.angleBuffer[fifo.tail])/2.0; // move predicted angle faster for the first few swings
        } else {
            grabberData.predictedBackstrokeStrikeAngle = 0.9*grabberData.predictedBackstrokeStrikeAngle + 0.1*fifo.angleBuffer[fifo.tail]; // more faith in result of prediction now so prefer predicted angle
        }
    } else if(!(strikeCount % 2) && (SD > 1.5*baseSD) && (fifo.angleBuffer[fifo.tail] > 290) && (fifo.rateBuffer[fifo.tail] > 20)) {
        strikeCount += 1;
        if(strikeCount < 16){
            grabberData.predictedHandstrokeStrikeAngle = (grabberData.predictedHandstrokeStrikeAngle + fifo.angleBuffer[fifo.tail])/2.0;
        } else {
            grabberData.predictedHandstrokeStrikeAngle = 0.9*grabberData.predictedHandstrokeStrikeAngle + 0.1*fifo.angleBuffer[fifo.tail];
        }
    }

    if((grabberData.lastStrike == 1) && (fifo.angleBuffer[fifo.tail] < grabberData.predictedBackstrokeStrikeAngle)){
        grabberData.lastStrike = 2;
        return(2);
    }
    if((grabberData.lastStrike == 2) && (fifo.angleBuffer[fifo.tail] > grabberData.predictedHandstrokeStrikeAngle)){
        grabberData.lastStrike = 1;
        return(1);
    }
    return(0);
}

// Madgwick's implementation of the Mahony filter.
// http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Neat as it does not have an internal state that needs to be maintained
// like the Kalman filters (other than the quaternion of course).
// Stripped out all the Ki stuff as we don't need to worry about gyro biases :)
// Units here are degrees per second for the gyro.  It does not matter
// what units the accelerometers are in as the measurements are normalised.
// h is sample interval (time in seconds since the last sample)
// The q0, q1, q2, q3, roll, pitch and yaw outputs are globals 
void calculate(float gx, float gy, float gz, float ax, float ay, float az, float h) {
    const float twoKp = 0.7;  // Magic number determined by experimentation
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx *= DEGREES_TO_RADIANS_MULTIPLIER;
    gy *= DEGREES_TO_RADIANS_MULTIPLIER;
    gz *= DEGREES_TO_RADIANS_MULTIPLIER;

    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
        recipNorm = sqrt(ax * ax + ay * ay + az * az);
        ax /= recipNorm;
        ay /= recipNorm;
        az /= recipNorm;
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5 + q3 * q3;
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
	}
    gx *= (0.5 * h);
    gy *= (0.5 * h);
    gz *= (0.5 * h);
    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= recipNorm;
    q1 /= recipNorm;
    q2 /= recipNorm;
    q3 /= recipNorm;

    roll = RADIANS_TO_DEGREES_MULTIPLIER * atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
    pitch = RADIANS_TO_DEGREES_MULTIPLIER * asin(-2.0 * (q1*q3 - q0*q2));
    yaw = RADIANS_TO_DEGREES_MULTIPLIER * atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
}

void getSavedData(){
    FILE *fdTare;
    char linein[50];
    fdTare = fopen("/tmp/BBtare","r");
    if(fdTare != NULL) {
        if (fgets(linein, sizeof(linein), fdTare) != NULL) {
            grabberData.tareValue = atof(linein);
            grabberData.torn = 3;
        }
        fclose(fdTare);
    }

    FILE *fdGravity;
    fdGravity = fopen("/tmp/BBgravity","r");
    if(fdGravity != NULL) {
        if (fgets(linein, sizeof(linein), fdGravity) != NULL) {
            char *ent = strtok(linein,","); 
            if(ent) grabberData.gravityValue = atof(ent);
            ent = strtok(NULL,",");
            if(ent) grabberData.ropeValueB = atof(ent);
            ent = strtok(NULL,",");
            if(ent) grabberData.ropeValueH = atof(ent);
            grabberData.calibrationState = 7;
        }
        fclose(fdGravity);
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
      printf("Unable to read from IMU device. \n");
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
    
    //select registers for FIFO
    writeRegister(ICM20689_FIFO_EN,0x78);

    //clear data paths and reset FIFO (keep i2c disabled)
    writeRegister(ICM20689_USER_CTRL,0x15);

	// load up calibration data from Arduino.  Calibration data uses
    // the method of D. Tedaldi, A. Pretto and E. Menegatti, 
    // "A Robust and Easy to Implement Method for IMU Calibration 
    // without External Equipments". In: Proceedings of the IEEE International 
    // Conference on Robotics and Automation (ICRA 2014), May 31 - June 7, 2014
    // Hong Kong, China, Page(s): 3042 - 3049
    // See https://bitbucket.org/alberto_pretto/imu_tk
    // notes on installing imu_tk and its dependencies and code to implement
    // the method are in this repository's utilities directory
/*
    (Arduino "registers" 10=samplePeriod, 11=accBiasX,
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
    float result;
    int fail = 0;
    result = extractFloat(0);
    if(!isnan(result)) {
        grabberData.samplePeriod = result;
    } else {
        grabberData.samplePeriod = 0.002;
        printf("EDEF:\n");
    }
    
    I2C_BUFFER[0]=200;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 200 (accBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.accBiasX = result/g0;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.accBiasY = result/g0;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.accBiasZ = result/g0;} else {fail =1;}

    if(fail){
        fail = 0;
        grabberData.accBiasX = 0.0;
        grabberData.accBiasY = 0.0;
        grabberData.accBiasZ = 0.0;
        printf("EDEF:\n");
    }
	
    I2C_BUFFER[0]=201;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 201 (accTransform first row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.accTransformMatrix[0] = result;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.accTransformMatrix[1] = result;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.accTransformMatrix[2] = result;} else {fail =1;}

    I2C_BUFFER[0]=202;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 201 (accTransform second row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.accTransformMatrix[3] = result;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.accTransformMatrix[4] = result;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.accTransformMatrix[5] = result;} else {fail =1;}

    I2C_BUFFER[0]=203;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 203 (accTransform bottom row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.accTransformMatrix[6] = result;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.accTransformMatrix[7] = result;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.accTransformMatrix[8] = result;} else {fail =1;}

    if(fail){
        fail = 0;
        grabberData.accTransformMatrix[0] = 1.0;
        grabberData.accTransformMatrix[1] = 0.0;
        grabberData.accTransformMatrix[2] = 0.0;
        grabberData.accTransformMatrix[3] = 0.0;
        grabberData.accTransformMatrix[4] = 1.0;
        grabberData.accTransformMatrix[5] = 0.0;
        grabberData.accTransformMatrix[6] = 0.0;
        grabberData.accTransformMatrix[7] = 0.0;
        grabberData.accTransformMatrix[8] = 1.0;
        printf("EDEF:\n");
    }

    I2C_BUFFER[0]=204;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 204 (gyroBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.gyroBiasX = result*RADIANS_TO_DEGREES_MULTIPLIER;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.gyroBiasY = result*RADIANS_TO_DEGREES_MULTIPLIER;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.gyroBiasZ = result*RADIANS_TO_DEGREES_MULTIPLIER;} else {fail =1;}

    if(fail){
        fail = 0;
        grabberData.gyroBiasX = 0.0;
        grabberData.gyroBiasY = 0.0;
        grabberData.gyroBiasZ = 0.0;
        printf("EDEF:\n");
    }

    I2C_BUFFER[0]=205;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 205 (gyroTransform first row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[0] = result;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[1] = result;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[2] = result;} else {fail =1;}

    I2C_BUFFER[0]=206;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 206 (gyroTransform second row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[3] = result;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[4] = result;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[5] = result;} else {fail =1;}

    I2C_BUFFER[0]=207;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 207 (gyroTransform bottom row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[6] = result;} else {fail =1;}
    result = extractFloat(4);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[7] = result;} else {fail =1;}
    result = extractFloat(8);
    if(!isnan(result)) {grabberData.gyroTransformMatrix[8] = result;} else {fail =1;}

    if(fail){
        grabberData.gyroTransformMatrix[0] = 1.0;
        grabberData.gyroTransformMatrix[1] = 0.0;
        grabberData.gyroTransformMatrix[2] = 0.0;
        grabberData.gyroTransformMatrix[3] = 0.0;
        grabberData.gyroTransformMatrix[4] = 1.0;
        grabberData.gyroTransformMatrix[5] = 0.0;
        grabberData.gyroTransformMatrix[6] = 0.0;
        grabberData.gyroTransformMatrix[7] = 0.0;
        grabberData.gyroTransformMatrix[8] = 1.0;
        printf("EDEF:\n");
    }

    getSavedData();
    if(grabberData.calibrationState != 7){ // no gravity/rope data received from getSavedData()
        I2C_BUFFER[0]=8;
        bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 8 (gravityValue)
        bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
        result = extractFloat(0);
        
        I2C_BUFFER[0]=7;
        bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 7 (ropeValueB)
        bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
        float result2 = extractFloat(0);

        I2C_BUFFER[0]=6;
        bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 6 (ropeValueH)
        bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
        float result3 = extractFloat(0);

        if(!isnan(result) && !isnan(result2) && !isnan(result3) && result != -360){
            grabberData.gravityValue = result;
            grabberData.ropeValueB = result2;
            grabberData.ropeValueH = result3;
            grabberData.calibrationState = 7;
            FILE *fdGravity;
            fdGravity = fopen("/tmp/BBgravity","w");
            if(fdGravity != NULL) {
                fprintf(fdGravity,"%+08.1f,%+08.1f,%+08.1f\n", grabberData.gravityValue, grabberData.ropeValueB, grabberData.ropeValueH);
                fclose(fdGravity);
            }
        }
    }

    if(grabberData.torn !=3){ // no zero data received from getSavedData()
        I2C_BUFFER[0]=9;
        bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 9 (tareValue)
        bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
        result = extractFloat(0);
        if(!isnan(result) && result != -360) {
            grabberData.tareValue = result;
            grabberData.torn = 3;
            FILE *fdTare;
            fdTare = fopen("/tmp/BBtare","w");
            if(fdTare != NULL) {
                fprintf(fdTare,"%+07.4f\n", grabberData.tareValue);
                fclose(fdTare);
            }
        }
    }
    //start FIFO - keep i2c disabled
    writeRegister(ICM20689_USER_CTRL,0x50);
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
