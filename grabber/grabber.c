//gcc grabber.c -o grabber -lm -lbcm2835

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

// these are used by the extended Kalman filter
#define g0 9.8137 // Manchester estimation.  Use gcalculator.xlsx to work out for your latitude.  
#define g0_2 (g0*g0)
#define q_dcm2 (0.1*0.1)
#define q_gyro_bias2 (0.0001*0.0001)
#define r_acc2 (0.5*0.5)
#define r_a2 (10*10)
#define q_dcm2_init (1*1)
#define q_gyro_bias2_init (0.1*0.1)

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 

FILE *fd_write_out;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float direction = 0;
float tareValue = 0.0;
float a[3];
float xGyroBias = 0.0;

int ODR = 125;
int RUNNING = 0;
int EYECANDY = 0;
int OUT_COUNT = 0;

unsigned char SPI_BUFFER[256];
unsigned char I2C_BUFFER[16];

char READ_OUTBUF[1500];
char READ_OUTBUF_LINE[150];
int READ_OUTBUF_COUNT;

char FILENAME[50];

const unsigned int LOOPSLEEP = 50000;  // main loop processes every 0.05 sec.  25 samples should be in FIFO.  Max number of samples in FIFO is 341.  Should be OK.
unsigned int LOOPCOUNT = 0;

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

volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}
/*
# There are fifteen possible command sent by the user's browser:
# (a)   STRT:[filename] Start recording a session with a bell.  The bell
#       must be at stand and (reasonably) stationary otherwise this
#       will return ESTD: or EMOV: either of which signals the browser 
#       to stop expecting data.  If Filename is not provided then a
#       default filename of "CurrentRecording [date]" is used.
# (b)   STOP: stops a recording in progress.  This also saves off the
#       collected data into a file in /data/samples/
# (c)   LDAT: a request for data from the browser, only works when there
#       is a recording in progress - deprecated.  Does nothing.
# (d)   FILE: get a listing of the previous recordings stored in
#       /data/samples/ .  Used to display the selection box to the user.
# (e)   LOAD:[filename] used to transmit a previous recording to the browser
# (f)   DATE:[string] sets the date on the device to the date on the browser 
#             string is unixtime (microsecs since 1/1/70)
# (g)   SAMP: requests the current sample period
# (h)   SHDN: tells the device to shutdown
# (i)   TEST: returns the current basic orientation (for testing)
# (j)   TARE: tares the bell (bell must be down and stable  - not confirmed by code)
# (k)   EYEC: starts eye candy demo
# (l)   STEC: stops eye candy demo
#
# There are the following responses back to the user's browser:
# (i)   STPD: tells the browser that the device has sucessfully stopped sampling
# (ii)  ESTP: signals an error in the stop process (an attempt to stopped when not started)
# (iii) LIVE:[string] the string contains the data recorded by the device in the format
#             "A:[angle]R:[rate]C:[acceleration]".  Angle, rate and acceleration are 
#             ordinary floating point numbers.  Angle is in degrees, rate is in degrees/sec
#             and acceleration is in degrees/sec/sec.
# (iv)  NDAT: indicates that there is no current data to send to the browser (deprecated)
# (v)   SAMP: returns the sample period
# (vi)  EIMU: IMU not detected or some IMU related failure
# (vii) ESTD: bell not at stand (aborts started session)
# (viii)EMOV: bell moving when session started (the bell has to be stationary at start)
# (ix)  ESAM: sample period needs to be measured first (deprecated)
# (x)   ESTR: internal error related to data / filenames (shouldn't happen)!
# (xi)  DATA:[string]  chunk of data from a previously stored file.  In same format as LIVE:
# (xii) EOVF: (sent by dcmimu) fifo overflow flagged.
# (xiii)FILE:[filename] in response to FILE: a list of the previous recordings in /data/samples
# (xiv) STRT: in response to STRT: indicates a sucessful start.
# (xv)  LFIN:[number] indicates the end of a file download and the number of samples sent
# (xvi) TEST:[number] current orientation
# (xvii)BATT:[number] battery percentage
*
* Commands are received on stdin and output is on stdout.
* Will work standalone but intended to be interfaced with websocketd https://github.com/joewalnes/websocketd
*/

#define PUSHBATCH 20 // number of samples taken before pushing out
#define SAVGOLLENGTH  15
#define SAVGOLHALF 7 // = math.floor(SAVGOLLENGTH/2.0)
#define BUFFERSIZE 256 // must be bigger than PUSHBATCH + SAVGOLLENGTH
unsigned int head;
unsigned int tail;
unsigned int available;
float angleBuffer[BUFFERSIZE];
float rateBuffer[BUFFERSIZE];

// savgol_coeffs(31,1,deriv=1,use="dot")
//const float savGolCoefficients[] = {
//    -6.04838710e-03, -5.64516129e-03, -5.24193548e-03, -4.83870968e-03,
//    -4.43548387e-03, -4.03225806e-03, -3.62903226e-03, -3.22580645e-03,
//    -2.82258065e-03, -2.41935484e-03, -2.01612903e-03, -1.61290323e-03,
//    -1.20967742e-03, -8.06451613e-04, -4.03225806e-04, -1.02877021e-17,
//    4.03225806e-04,   8.06451613e-04,  1.20967742e-03,  1.61290323e-03,
//    2.01612903e-03,   2.41935484e-03,  2.82258065e-03,  3.22580645e-03,
//    3.62903226e-03,   4.03225806e-03,  4.43548387e-03,  4.83870968e-03,
//    5.24193548e-03,   5.64516129e-03,  6.04838710e-03};

// savgol_coeffs(15,2,deriv=1,use="dot")
const float savGolCoefficients[] = {
    -2.50000000e-02, -2.14285714e-02, -1.78571429e-02, -1.42857143e-02,
    -1.07142857e-02, -7.14285714e-03, -3.57142857e-03,  2.95770353e-16,
    3.57142857e-03,  7.14285714e-03,  1.07142857e-02,  1.42857143e-02,
    1.78571429e-02,  2.14285714e-02,  2.50000000e-02};

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
 
    FILE *fdTare;
    fdTare = fopen("/tmp/bb_tare","r");
    if(fdTare != NULL) {
        if (fgets(linein, sizeof(linein), fdTare) != NULL) {
            tareValue = atof(linein);
        }
        fclose(fdTare);
    }

    FILE *fdFlagRunning;
    fdFlagRunning = fopen("/tmp/BBlive","w"); // signal to powermonitor that we have taken over power monitoring
    unlink("/tmp/BBlive");
    usleep(1000); // need to ensure that powermonitor is not doing anything before we try to access i2C

    setup();

    while(!sig_exit){
        usleep(LOOPSLEEP);
		LOOPCOUNT += 1;
		if(LOOPCOUNT == 1 || (LOOPCOUNT % 2000) == 0){ // check every 100 secs but also the first time through
			I2C_BUFFER[0]=2;
			bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 2 (power)
			bcm2835_i2c_read(I2C_BUFFER,2); usleep(100);
			unsigned int result = (I2C_BUFFER[0]<<8)+I2C_BUFFER[1];
			if(result & 0x8000) system("/usr/bin/sync && /usr/bin/shutdown -P now");
			if((result & 0x7FFF) > 519){
				printf("BATT:0\n");
			} else if((result & 0x7FFF) > 509){
				printf("BATT:10\n");
			} else if((result & 0x7FFF) > 503){
				printf("BATT:20\n");
			} else if((result & 0x7FFF) > 500){
				printf("BATT:30\n");
			} else if((result & 0x7FFF) > 495){
				printf("BATT:40\n");
			} else if((result & 0x7FFF) > 490){
				printf("BATT:50\n");
			} else if((result & 0x7FFF) > 482){
				printf("BATT:60\n");
			} else if((result & 0x7FFF) > 475){
				printf("BATT:70\n");
			} else if((result & 0x7FFF) > 466){
				printf("BATT:80\n");
			} else if((result & 0x7FFF) > 456){
				printf("BATT:90\n");
			} else {
				printf("BATT:100\n");
			}
		}
      
        pushData();

        if(EYECANDY) printf("EYEC:%+07.1f, %+07.1f, %+07.1f\n", roll, pitch, yaw);

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
                printf("SAMP:%f\n",calibrationData.samplePeriod);
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
                }
                continue;
            }
            if(strcmp("TEST:", command) == 0) {
				printf("TEST: R:%f P:%f Y:%f\n",roll,pitch,yaw);
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
            if(strcmp("TARE:", command) == 0) {
                tareValue = roll;
                fdTare = fopen("/tmp/bb_tare","w");
                if(fdTare == NULL) {
                    printf("EIMU: Could not open tare file for writing\n");
                    continue;
                } else {
                    fprintf(fdTare,"%+07.4f\n", tareValue);
                    fclose(fdTare);
                }
                continue;
            }
            if(strcmp("CLTA:", command) == 0) {
                tareValue = 0.0;
                continue;
            }
            if(strcmp("STRT:", command) == 0) {
                if(entries == 2  && strlen(details) < 34){
                    sprintf(FILENAME, "/data/samples/%s", details);
                } else {
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
                OUT_COUNT = 0;
                startRun();
                continue;
            }
            if(strcmp("STOP:", command) == 0) {
                if(fd_write_out != NULL){
                    fflush(fd_write_out);
                    fclose(fd_write_out);
                    fd_write_out = NULL;
                }
                RUNNING = 0;
                printf("STPD:%d\n", OUT_COUNT);
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
                    }
                    if(READ_OUTBUF_COUNT != 0) printf("%s\n",READ_OUTBUF);
                    printf("LFIN:%d\n", counter);
                }
                fclose(fd_read_in);
                continue;
            }
            if(strcmp("SHDN:", command) == 0) {
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

void startRun(void){
    if(abs(roll-tareValue) < 160 || abs(roll-tareValue) > 178){
        printf("ESTD:%+07.1f\n",roll-tareValue); // bell not at stand
        return;
    }

    if(roll < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        direction = -1;
    } else {
        direction = +1;
    }

    for(int i = 0; i< BUFFERSIZE; ++i){ // initialise with dummy data for savgol alignment
        rateBuffer[i] = 0;
        angleBuffer[i] = direction*(roll-tareValue)-180.0;
    }
    head = SAVGOLHALF;
    tail = 0;
    available = SAVGOLHALF;

    RUNNING = 1;
	EYECANDY=0;
    return;
}

void pushData(void){
    static char local_outbuf[4000];
    static char remote_outbuf[4000];
    static char local_outbuf_line[150];
    static char remote_outbuf_line[150];
    static int local_count = 0;
    int remote_count = 0;
    static float accn = 0;
    static float nudgeAngle=0;
    static int nudgeCount =0;
    float taccn;

    pullData();

    pullData();

    if(!RUNNING) return;
    if(available < PUSHBATCH + SAVGOLLENGTH) return;

    for(int counter = available; counter > SAVGOLLENGTH; --counter){
        taccn = savGol(tail);
        if(taccn * accn < 0.0 && angleBuffer[tail] > 170.0 && angleBuffer[tail] < 190.0  && nudgeCount == 0) {
            unsigned int lastTail = (tail == 0) ? BUFFERSIZE-1 : tail -1;
            float nudgeFactor = accn /(accn - taccn);
            float angleDiffFrom180 = (angleBuffer[lastTail] + nudgeFactor*(angleBuffer[tail]-angleBuffer[lastTail])) - 180.0;
            if(nudgeAngle != 0.0){
                nudgeAngle = 0.9*nudgeAngle + 0.1*angleDiffFrom180; // apply a bit of smoothing
            } else {
                nudgeAngle = angleDiffFrom180;
            }
            nudgeCount = 5;
        } 
        if(nudgeCount != 0) nudgeCount -= 1; // don't do this twice in one half stroke
        accn = taccn;

        sprintf(remote_outbuf_line, "LIVE:A:%+07.1f,R:%+07.1f,C:%+07.1f,N:%+07.1f\n", angleBuffer[tail]-nudgeAngle, rateBuffer[tail], accn, nudgeAngle);
        if (remote_count + strlen(remote_outbuf_line) > (sizeof remote_outbuf -2)) {
            printf("%s",remote_outbuf);
            remote_count = 0;
        }
        remote_count += sprintf(&remote_outbuf[remote_count],remote_outbuf_line);
 
        sprintf(local_outbuf_line,"A:%+07.1f,R:%+07.1f,C:%+07.1f\n", angleBuffer[tail], rateBuffer[tail], accn);
        if (local_count + strlen(local_outbuf_line) > (sizeof local_outbuf -2)) {
            fputs(local_outbuf, fd_write_out);
            fflush(fd_write_out);
            local_count = 0;
        }
        local_count += sprintf(&local_outbuf[local_count],local_outbuf_line);
        tail = (tail + 1) % BUFFERSIZE;
        available -= 1;
//        fprintf(fd_write_out,"A:%+07.1f,R:%+07.1f,C:%+07.1f,P:%+07.1f,Y:%+07.1f,AX:%+06.3f,AY:%+06.3f,AZ:%+06.3f,GX:%+07.1f,GY:%+07.1f,GZ:%+07.1f,X:%+06.3f,Y:%+06.3f,Z:%+06.3f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang, pitch, yaw, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2],a[0],a[1],a[2]);
    }
    if(remote_count != 0) {printf("%s",remote_outbuf); remote_count = 0;}
    if(local_count != 0) {fputs(local_outbuf, fd_write_out); local_count = 0;}  // perhaps make sd card writes less frequent but this'll do for the moment, dont' forget to increase the size of the buffer to do this.
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
    
    if(RUNNING) OUT_COUNT += (count_1 > count_2) ? (count_2 / 48) : (count_1 / 48);
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
		
        combined_data[0] = (combined_data[0]-calibrationData.accBiasX)*calibrationData.accScaleX;
        combined_data[1] = (combined_data[1]-calibrationData.accBiasY)*calibrationData.accScaleY;
        combined_data[2] = (combined_data[2]-calibrationData.accBiasZ)*calibrationData.accScaleZ;
        combined_data[3] = (combined_data[3]-calibrationData.gyroBiasX)*calibrationData.gyroScaleX;
        combined_data[4] = (combined_data[4]-calibrationData.gyroBiasY)*calibrationData.gyroScaleY;
        combined_data[5] = (combined_data[5]-calibrationData.gyroBiasZ)*calibrationData.gyroScaleZ;
		
        calculate(combined_data[3],combined_data[4],combined_data[5], combined_data[0],combined_data[1],combined_data[2],calibrationData.samplePeriod);

        if(!RUNNING) continue;

    // need to correct reported angles to match
    // the system we are using 0 degrees = bell at
    // balance at handstroke, 360 degrees = bell at
    // balance at backstroke. 180 degrees = BDC. 

        unsigned int lastHead = (head == 0) ? BUFFERSIZE-1 : head -1;
        float angle = direction*(roll-tareValue)-180.0;
        angle += angleCorrection;
        if((angleBuffer[lastHead] - angle) > 270){
            if(angleCorrection < 720 ) { // should always be true
                angleCorrection += 360; 
                angle += 360;
            }
        } else if ((angleBuffer[lastHead] - angle) < -270){
            if(angleCorrection > 0){ // should always be true
            angleCorrection -= 360; 
            angle -= 360;
            }
        }
        
        rateBuffer[head] = (combined_data[3] - xGyroBias) * direction;
        angleBuffer[head] = angle;
        head = (head + 1) % BUFFERSIZE;
        available += 1;
        if(available >= BUFFERSIZE) {printf("EOVF:\n"); available -= 1;}
    }
}

float savGol(unsigned int startPosition){
    unsigned int windowStartPosition = (startPosition < SAVGOLHALF) ? (startPosition + BUFFERSIZE) - SAVGOLHALF: startPosition - SAVGOLHALF;
    float savGolResult=0;
    for(int i = 0; i < SAVGOLLENGTH; ++i){
        savGolResult += ODR*(savGolCoefficients[i] * rateBuffer[(windowStartPosition +i) % BUFFERSIZE]);
    }
    return(savGolResult);
}

/* 
This function does some magic with an Extended Kalman Filter to produce roll
pitch and yaw measurements from accelerometer/gyro data.  Of the filters tested,
it produced the best results.

Github is here: https://github.com/hhyyti/dcm-imu 

MIT licence but the readme includes this line:
    If you use the algorithm in any scientific context, please cite: Heikki Hyyti and Arto Visala, 
    "A DCM Based Attitude Estimation Algorithm for Low-Cost MEMS IMUs," International Journal 
    of Navigation and Observation, vol. 2015, Article ID 503814, 18 pages, 2015. http://dx.doi.org/10.1155/2015/503814

function inputs are 
Xgyro (u0 - in degrees/sec), Ygyro (u1 - in degrees/sec), Zgyro (u2 - in degrees/sec), 
Xaccel (z0 - in g), Yaccel (z1 - in g), Zaccel (z2 - in g),
interval (h - in seconds from the last time the function was called)
*/

void calculate(float u0, float u1, float u2, float z0, float z1,float z2, float h){

    static float x[] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    static float P[6][6] = {    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

// convert units
    u0 = u0 * DEGREES_TO_RADIANS_MULTIPLIER;
    u1 = u1 * DEGREES_TO_RADIANS_MULTIPLIER;
    u2 = u2 * DEGREES_TO_RADIANS_MULTIPLIER;
    
    z0 = z0 * g0;
    z1 = z1 * g0;
    z2 = z2 * g0;
    
    float x_last[] = {x[0], x[1], x[2]};
    float x_0 = x[0]-h*(u1*x[2]-u2*x[1]+x[1]*x[5]-x[2]*x[4]);
    float x_1 = x[1]+h*(u0*x[2]-u2*x[0]+x[0]*x[5]-x[2]*x[3]);
    float x_2 = x[2]-h*(u0*x[1]-u1*x[0]+x[0]*x[4]-x[1]*x[3]);
    float x_3 = x[3];
    float x_4 = x[4];
    float x_5 = x[5];

    float hh = h*h;
    float P_00 = P[0][0]-h*(P[0][5]*x[1]-P[0][4]*x[2]-P[4][0]*x[2]+P[5][0]*x[1]+P[0][2]*(u1-x[4])+P[2][0]*(u1-x[4])-P[0][1]*(u2-x[5])-P[1][0]*(u2-x[5]))+hh*(q_dcm2-x[1]*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]))+x[2]*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]))-(u1-x[4])*(P[4][2]*x[2]-P[5][2]*x[1]-P[2][2]*(u1-x[4])+P[1][2]*(u2-x[5]))+(u2-x[5])*(P[4][1]*x[2]-P[5][1]*x[1]-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5])));
    float P_01 = P[0][1]+h*(P[0][5]*x[0]-P[0][3]*x[2]+P[4][1]*x[2]-P[5][1]*x[1]+P[0][2]*(u0-x[3])-P[0][0]*(u2-x[5])-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5]))+hh*(x[0]*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]))-x[2]*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]))+(u0-x[3])*(P[4][2]*x[2]-P[5][2]*x[1]-P[2][2]*(u1-x[4])+P[1][2]*(u2-x[5]))-(u2-x[5])*(P[4][0]*x[2]-P[5][0]*x[1]-P[2][0]*(u1-x[4])+P[1][0]*(u2-x[5])));
    float P_02 = P[0][2]-h*(P[0][4]*x[0]-P[0][3]*x[1]-P[4][2]*x[2]+P[5][2]*x[1]+P[0][1]*(u0-x[3])-P[0][0]*(u1-x[4])+P[2][2]*(u1-x[4])-P[1][2]*(u2-x[5]))-hh*(x[0]*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]))-x[1]*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]))+(u0-x[3])*(P[4][1]*x[2]-P[5][1]*x[1]-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5]))-(u1-x[4])*(P[4][0]*x[2]-P[5][0]*x[1]-P[2][0]*(u1-x[4])+P[1][0]*(u2-x[5])));
    float P_03 = P[0][3]+h*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]));
    float P_04 = P[0][4]+h*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]));
    float P_05 = P[0][5]+h*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]));
    float P_10 = P[1][0]-h*(P[1][5]*x[1]-P[1][4]*x[2]+P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[1][2]*(u1-x[4])+P[0][0]*(u2-x[5])-P[1][1]*(u2-x[5]))+hh*(x[1]*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]))-x[2]*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]))+(u1-x[4])*(P[3][2]*x[2]-P[5][2]*x[0]-P[2][2]*(u0-x[3])+P[0][2]*(u2-x[5]))-(u2-x[5])*(P[3][1]*x[2]-P[5][1]*x[0]-P[2][1]*(u0-x[3])+P[0][1]*(u2-x[5])));
    float P_11 = P[1][1]+h*(P[1][5]*x[0]-P[1][3]*x[2]-P[3][1]*x[2]+P[5][1]*x[0]+P[1][2]*(u0-x[3])+P[2][1]*(u0-x[3])-P[0][1]*(u2-x[5])-P[1][0]*(u2-x[5]))+hh*(q_dcm2-x[0]*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]))+x[2]*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]))-(u0-x[3])*(P[3][2]*x[2]-P[5][2]*x[0]-P[2][2]*(u0-x[3])+P[0][2]*(u2-x[5]))+(u2-x[5])*(P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[0][0]*(u2-x[5])));
    float P_12 = P[1][2]-h*(P[1][4]*x[0]-P[1][3]*x[1]+P[3][2]*x[2]-P[5][2]*x[0]+P[1][1]*(u0-x[3])-P[2][2]*(u0-x[3])-P[1][0]*(u1-x[4])+P[0][2]*(u2-x[5]))+hh*(x[0]*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]))-x[1]*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]))+(u0-x[3])*(P[3][1]*x[2]-P[5][1]*x[0]-P[2][1]*(u0-x[3])+P[0][1]*(u2-x[5]))-(u1-x[4])*(P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[0][0]*(u2-x[5])));
    float P_13 = P[1][3]-h*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]));
    float P_14 = P[1][4]-h*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]));
    float P_15 = P[1][5]-h*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]));
    float P_20 = P[2][0]-h*(P[2][5]*x[1]-P[3][0]*x[1]+P[4][0]*x[0]-P[2][4]*x[2]+P[1][0]*(u0-x[3])-P[0][0]*(u1-x[4])+P[2][2]*(u1-x[4])-P[2][1]*(u2-x[5]))-hh*(x[1]*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]))-x[2]*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]))+(u1-x[4])*(P[3][2]*x[1]-P[4][2]*x[0]-P[1][2]*(u0-x[3])+P[0][2]*(u1-x[4]))-(u2-x[5])*(P[3][1]*x[1]-P[4][1]*x[0]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4])));
    float P_21 = P[2][1]+h*(P[2][5]*x[0]+P[3][1]*x[1]-P[4][1]*x[0]-P[2][3]*x[2]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4])+P[2][2]*(u0-x[3])-P[2][0]*(u2-x[5]))+hh*(x[0]*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]))-x[2]*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]))+(u0-x[3])*(P[3][2]*x[1]-P[4][2]*x[0]-P[1][2]*(u0-x[3])+P[0][2]*(u1-x[4]))-(u2-x[5])*(P[3][0]*x[1]-P[4][0]*x[0]-P[1][0]*(u0-x[3])+P[0][0]*(u1-x[4])));
    float P_22 = P[2][2]-h*(P[2][4]*x[0]-P[2][3]*x[1]-P[3][2]*x[1]+P[4][2]*x[0]+P[1][2]*(u0-x[3])+P[2][1]*(u0-x[3])-P[0][2]*(u1-x[4])-P[2][0]*(u1-x[4]))+hh*(q_dcm2-x[0]*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]))+x[1]*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]))-(u0-x[3])*(P[3][1]*x[1]-P[4][1]*x[0]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4]))+(u1-x[4])*(P[3][0]*x[1]-P[4][0]*x[0]-P[1][0]*(u0-x[3])+P[0][0]*(u1-x[4])));
    float P_23 = P[2][3]+h*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]));
    float P_24 = P[2][4]+h*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]));
    float P_25 = P[2][5]+h*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]));
    float P_30 = P[3][0]-h*(P[3][5]*x[1]-P[3][4]*x[2]+P[3][2]*(u1-x[4])-P[3][1]*(u2-x[5]));
    float P_31 = P[3][1]+h*(P[3][5]*x[0]-P[3][3]*x[2]+P[3][2]*(u0-x[3])-P[3][0]*(u2-x[5]));
    float P_32 = P[3][2]-h*(P[3][4]*x[0]-P[3][3]*x[1]+P[3][1]*(u0-x[3])-P[3][0]*(u1-x[4]));
    float P_33 = P[3][3]+hh*q_gyro_bias2;
    float P_34 = P[3][4];
    float P_35 = P[3][5];
    float P_40 = P[4][0]-h*(P[4][5]*x[1]-P[4][4]*x[2]+P[4][2]*(u1-x[4])-P[4][1]*(u2-x[5]));
    float P_41 = P[4][1]+h*(P[4][5]*x[0]-P[4][3]*x[2]+P[4][2]*(u0-x[3])-P[4][0]*(u2-x[5]));
    float P_42 = P[4][2]-h*(P[4][4]*x[0]-P[4][3]*x[1]+P[4][1]*(u0-x[3])-P[4][0]*(u1-x[4]));
    float P_43 = P[4][3];
    float P_44 = P[4][4]+hh*q_gyro_bias2;
    float P_45 = P[4][5];
    float P_50 = P[5][0]-h*(P[5][5]*x[1]-P[5][4]*x[2]+P[5][2]*(u1-x[4])-P[5][1]*(u2-x[5]));
    float P_51 = P[5][1]+h*(P[5][5]*x[0]-P[5][3]*x[2]+P[5][2]*(u0-x[3])-P[5][0]*(u2-x[5]));
    float P_52 = P[5][2]-h*(P[5][4]*x[0]-P[5][3]*x[1]+P[5][1]*(u0-x[3])-P[5][0]*(u1-x[4]));
    float P_53 = P[5][3];
    float P_54 = P[5][4];
    float P_55 = P[5][5]+hh*q_gyro_bias2;

    // kalman innovation
    float y0 = z0-g0*x_0;
    float y1 = z1-g0*x_1;
    float y2 = z2-g0*x_2;

    float a_len = sqrt(y0*y0+y1*y1+y2*y2);

    float S00 = r_acc2+a_len*r_a2+P_00*g0_2;
    float S01 = P_01*g0_2;
    float S02 = P_02*g0_2;
    float S10 = P_10*g0_2;
    float S11 = r_acc2+a_len*r_a2+P_11*g0_2;
    float S12 = P_12*g0_2;
    float S20 = P_20*g0_2;
    float S21 = P_21*g0_2;
    float S22 = r_acc2+a_len*r_a2+P_22*g0_2;

    // Kalman gain
    float invPart = 1.0 / (S00*S11*S22-S00*S12*S21-S01*S10*S22+S01*S12*S20+S02*S10*S21-S02*S11*S20);
    float K00 = (g0*(P_02*S10*S21-P_02*S11*S20-P_01*S10*S22+P_01*S12*S20+P_00*S11*S22-P_00*S12*S21))*invPart;
    float K01 = -(g0*(P_02*S00*S21-P_02*S01*S20-P_01*S00*S22+P_01*S02*S20+P_00*S01*S22-P_00*S02*S21))*invPart;
    float K02 = (g0*(P_02*S00*S11-P_02*S01*S10-P_01*S00*S12+P_01*S02*S10+P_00*S01*S12-P_00*S02*S11))*invPart;
    float K10 = (g0*(P_12*S10*S21-P_12*S11*S20-P_11*S10*S22+P_11*S12*S20+P_10*S11*S22-P_10*S12*S21))*invPart;
    float K11 = -(g0*(P_12*S00*S21-P_12*S01*S20-P_11*S00*S22+P_11*S02*S20+P_10*S01*S22-P_10*S02*S21))*invPart;
    float K12 = (g0*(P_12*S00*S11-P_12*S01*S10-P_11*S00*S12+P_11*S02*S10+P_10*S01*S12-P_10*S02*S11))*invPart;
    float K20 = (g0*(P_22*S10*S21-P_22*S11*S20-P_21*S10*S22+P_21*S12*S20+P_20*S11*S22-P_20*S12*S21))*invPart;
    float K21 = -(g0*(P_22*S00*S21-P_22*S01*S20-P_21*S00*S22+P_21*S02*S20+P_20*S01*S22-P_20*S02*S21))*invPart;
    float K22 = (g0*(P_22*S00*S11-P_22*S01*S10-P_21*S00*S12+P_21*S02*S10+P_20*S01*S12-P_20*S02*S11))*invPart;
    float K30 = (g0*(P_32*S10*S21-P_32*S11*S20-P_31*S10*S22+P_31*S12*S20+P_30*S11*S22-P_30*S12*S21))*invPart;
    float K31 = -(g0*(P_32*S00*S21-P_32*S01*S20-P_31*S00*S22+P_31*S02*S20+P_30*S01*S22-P_30*S02*S21))*invPart;
    float K32 = (g0*(P_32*S00*S11-P_32*S01*S10-P_31*S00*S12+P_31*S02*S10+P_30*S01*S12-P_30*S02*S11))*invPart;
    float K40 = (g0*(P_42*S10*S21-P_42*S11*S20-P_41*S10*S22+P_41*S12*S20+P_40*S11*S22-P_40*S12*S21))*invPart;
    float K41 = -(g0*(P_42*S00*S21-P_42*S01*S20-P_41*S00*S22+P_41*S02*S20+P_40*S01*S22-P_40*S02*S21))*invPart;
    float K42 = (g0*(P_42*S00*S11-P_42*S01*S10-P_41*S00*S12+P_41*S02*S10+P_40*S01*S12-P_40*S02*S11))*invPart;
    float K50 = (g0*(P_52*S10*S21-P_52*S11*S20-P_51*S10*S22+P_51*S12*S20+P_50*S11*S22-P_50*S12*S21))*invPart;
    float K51 = -(g0*(P_52*S00*S21-P_52*S01*S20-P_51*S00*S22+P_51*S02*S20+P_50*S01*S22-P_50*S02*S21))*invPart;
    float K52 = (g0*(P_52*S00*S11-P_52*S01*S10-P_51*S00*S12+P_51*S02*S10+P_50*S01*S12-P_50*S02*S11))*invPart;

    // update a posteriori
    x[0] = x_0+K00*y0+K01*y1+K02*y2;
    x[1] = x_1+K10*y0+K11*y1+K12*y2;
    x[2] = x_2+K20*y0+K21*y1+K22*y2;
    x[3] = x_3+K30*y0+K31*y1+K32*y2;
    x[4] = x_4+K40*y0+K41*y1+K42*y2;
    x[5] = x_5+K50*y0+K51*y1+K52*y2;

    //  update a posteriori covariance
    float r_adab = (r_acc2+a_len*r_a2);
    float P__00 = P_00-g0*(K00*P_00*2.0+K01*P_01+K01*P_10+K02*P_02+K02*P_20)+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+g0_2*(K00*(K00*P_00+K01*P_10+K02*P_20)+K01*(K00*P_01+K01*P_11+K02*P_21)+K02*(K00*P_02+K01*P_12+K02*P_22));
    float P__01 = P_01-g0*(K00*P_01+K01*P_11+K02*P_21+K10*P_00+K11*P_01+K12*P_02)+g0_2*(K10*(K00*P_00+K01*P_10+K02*P_20)+K11*(K00*P_01+K01*P_11+K02*P_21)+K12*(K00*P_02+K01*P_12+K02*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
    float P__02 = P_02-g0*(K00*P_02+K01*P_12+K02*P_22+K20*P_00+K21*P_01+K22*P_02)+g0_2*(K20*(K00*P_00+K01*P_10+K02*P_20)+K21*(K00*P_01+K01*P_11+K02*P_21)+K22*(K00*P_02+K01*P_12+K02*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
    float P__03 = P_03-g0*(K00*P_03+K01*P_13+K02*P_23+K30*P_00+K31*P_01+K32*P_02)+g0_2*(K30*(K00*P_00+K01*P_10+K02*P_20)+K31*(K00*P_01+K01*P_11+K02*P_21)+K32*(K00*P_02+K01*P_12+K02*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
    float P__04 = P_04-g0*(K00*P_04+K01*P_14+K02*P_24+K40*P_00+K41*P_01+K42*P_02)+g0_2*(K40*(K00*P_00+K01*P_10+K02*P_20)+K41*(K00*P_01+K01*P_11+K02*P_21)+K42*(K00*P_02+K01*P_12+K02*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
    float P__05 = P_05-g0*(K00*P_05+K01*P_15+K02*P_25+K50*P_00+K51*P_01+K52*P_02)+g0_2*(K50*(K00*P_00+K01*P_10+K02*P_20)+K51*(K00*P_01+K01*P_11+K02*P_21)+K52*(K00*P_02+K01*P_12+K02*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
    float P__10 = P_10-g0*(K00*P_10+K01*P_11+K02*P_12+K10*P_00+K11*P_10+K12*P_20)+g0_2*(K00*(K10*P_00+K11*P_10+K12*P_20)+K01*(K10*P_01+K11*P_11+K12*P_21)+K02*(K10*P_02+K11*P_12+K12*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
    float P__11 = P_11-g0*(K10*P_01+K10*P_10+K11*P_11*2.0+K12*P_12+K12*P_21)+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+g0_2*(K10*(K10*P_00+K11*P_10+K12*P_20)+K11*(K10*P_01+K11*P_11+K12*P_21)+K12*(K10*P_02+K11*P_12+K12*P_22));
    float P__12 = P_12-g0*(K10*P_02+K11*P_12+K12*P_22+K20*P_10+K21*P_11+K22*P_12)+g0_2*(K20*(K10*P_00+K11*P_10+K12*P_20)+K21*(K10*P_01+K11*P_11+K12*P_21)+K22*(K10*P_02+K11*P_12+K12*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
    float P__13 = P_13-g0*(K10*P_03+K11*P_13+K12*P_23+K30*P_10+K31*P_11+K32*P_12)+g0_2*(K30*(K10*P_00+K11*P_10+K12*P_20)+K31*(K10*P_01+K11*P_11+K12*P_21)+K32*(K10*P_02+K11*P_12+K12*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
    float P__14 = P_14-g0*(K10*P_04+K11*P_14+K12*P_24+K40*P_10+K41*P_11+K42*P_12)+g0_2*(K40*(K10*P_00+K11*P_10+K12*P_20)+K41*(K10*P_01+K11*P_11+K12*P_21)+K42*(K10*P_02+K11*P_12+K12*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
    float P__15 = P_15-g0*(K10*P_05+K11*P_15+K12*P_25+K50*P_10+K51*P_11+K52*P_12)+g0_2*(K50*(K10*P_00+K11*P_10+K12*P_20)+K51*(K10*P_01+K11*P_11+K12*P_21)+K52*(K10*P_02+K11*P_12+K12*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
    float P__20 = P_20-g0*(K00*P_20+K01*P_21+K02*P_22+K20*P_00+K21*P_10+K22*P_20)+g0_2*(K00*(K20*P_00+K21*P_10+K22*P_20)+K01*(K20*P_01+K21*P_11+K22*P_21)+K02*(K20*P_02+K21*P_12+K22*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
    float P__21 = P_21-g0*(K10*P_20+K11*P_21+K12*P_22+K20*P_01+K21*P_11+K22*P_21)+g0_2*(K10*(K20*P_00+K21*P_10+K22*P_20)+K11*(K20*P_01+K21*P_11+K22*P_21)+K12*(K20*P_02+K21*P_12+K22*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
    float P__22 = P_22-g0*(K20*P_02+K20*P_20+K21*P_12+K21*P_21+K22*P_22*2.0)+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+g0_2*(K20*(K20*P_00+K21*P_10+K22*P_20)+K21*(K20*P_01+K21*P_11+K22*P_21)+K22*(K20*P_02+K21*P_12+K22*P_22));
    float P__23 = P_23-g0*(K20*P_03+K21*P_13+K22*P_23+K30*P_20+K31*P_21+K32*P_22)+g0_2*(K30*(K20*P_00+K21*P_10+K22*P_20)+K31*(K20*P_01+K21*P_11+K22*P_21)+K32*(K20*P_02+K21*P_12+K22*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
    float P__24 = P_24-g0*(K20*P_04+K21*P_14+K22*P_24+K40*P_20+K41*P_21+K42*P_22)+g0_2*(K40*(K20*P_00+K21*P_10+K22*P_20)+K41*(K20*P_01+K21*P_11+K22*P_21)+K42*(K20*P_02+K21*P_12+K22*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
    float P__25 = P_25-g0*(K20*P_05+K21*P_15+K22*P_25+K50*P_20+K51*P_21+K52*P_22)+g0_2*(K50*(K20*P_00+K21*P_10+K22*P_20)+K51*(K20*P_01+K21*P_11+K22*P_21)+K52*(K20*P_02+K21*P_12+K22*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
    float P__30 = P_30-g0*(K00*P_30+K01*P_31+K02*P_32+K30*P_00+K31*P_10+K32*P_20)+g0_2*(K00*(K30*P_00+K31*P_10+K32*P_20)+K01*(K30*P_01+K31*P_11+K32*P_21)+K02*(K30*P_02+K31*P_12+K32*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
    float P__31 = P_31-g0*(K10*P_30+K11*P_31+K12*P_32+K30*P_01+K31*P_11+K32*P_21)+g0_2*(K10*(K30*P_00+K31*P_10+K32*P_20)+K11*(K30*P_01+K31*P_11+K32*P_21)+K12*(K30*P_02+K31*P_12+K32*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
    float P__32 = P_32-g0*(K20*P_30+K21*P_31+K22*P_32+K30*P_02+K31*P_12+K32*P_22)+g0_2*(K20*(K30*P_00+K31*P_10+K32*P_20)+K21*(K30*P_01+K31*P_11+K32*P_21)+K22*(K30*P_02+K31*P_12+K32*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
    float P__33 = P_33-g0*(K30*P_03+K31*P_13+K30*P_30+K31*P_31+K32*P_23+K32*P_32)+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+g0_2*(K30*(K30*P_00+K31*P_10+K32*P_20)+K31*(K30*P_01+K31*P_11+K32*P_21)+K32*(K30*P_02+K31*P_12+K32*P_22));
    float P__34 = P_34-g0*(K30*P_04+K31*P_14+K32*P_24+K40*P_30+K41*P_31+K42*P_32)+g0_2*(K40*(K30*P_00+K31*P_10+K32*P_20)+K41*(K30*P_01+K31*P_11+K32*P_21)+K42*(K30*P_02+K31*P_12+K32*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
    float P__35 = P_35-g0*(K30*P_05+K31*P_15+K32*P_25+K50*P_30+K51*P_31+K52*P_32)+g0_2*(K50*(K30*P_00+K31*P_10+K32*P_20)+K51*(K30*P_01+K31*P_11+K32*P_21)+K52*(K30*P_02+K31*P_12+K32*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
    float P__40 = P_40-g0*(K00*P_40+K01*P_41+K02*P_42+K40*P_00+K41*P_10+K42*P_20)+g0_2*(K00*(K40*P_00+K41*P_10+K42*P_20)+K01*(K40*P_01+K41*P_11+K42*P_21)+K02*(K40*P_02+K41*P_12+K42*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
    float P__41 = P_41-g0*(K10*P_40+K11*P_41+K12*P_42+K40*P_01+K41*P_11+K42*P_21)+g0_2*(K10*(K40*P_00+K41*P_10+K42*P_20)+K11*(K40*P_01+K41*P_11+K42*P_21)+K12*(K40*P_02+K41*P_12+K42*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
    float P__42 = P_42-g0*(K20*P_40+K21*P_41+K22*P_42+K40*P_02+K41*P_12+K42*P_22)+g0_2*(K20*(K40*P_00+K41*P_10+K42*P_20)+K21*(K40*P_01+K41*P_11+K42*P_21)+K22*(K40*P_02+K41*P_12+K42*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
    float P__43 = P_43-g0*(K30*P_40+K31*P_41+K32*P_42+K40*P_03+K41*P_13+K42*P_23)+g0_2*(K30*(K40*P_00+K41*P_10+K42*P_20)+K31*(K40*P_01+K41*P_11+K42*P_21)+K32*(K40*P_02+K41*P_12+K42*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
    float P__44 = P_44-g0*(K40*P_04+K41*P_14+K40*P_40+K42*P_24+K41*P_41+K42*P_42)+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+g0_2*(K40*(K40*P_00+K41*P_10+K42*P_20)+K41*(K40*P_01+K41*P_11+K42*P_21)+K42*(K40*P_02+K41*P_12+K42*P_22));
    float P__45 = P_45-g0*(K40*P_05+K41*P_15+K42*P_25+K50*P_40+K51*P_41+K52*P_42)+g0_2*(K50*(K40*P_00+K41*P_10+K42*P_20)+K51*(K40*P_01+K41*P_11+K42*P_21)+K52*(K40*P_02+K41*P_12+K42*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
    float P__50 = P_50-g0*(K00*P_50+K01*P_51+K02*P_52+K50*P_00+K51*P_10+K52*P_20)+g0_2*(K00*(K50*P_00+K51*P_10+K52*P_20)+K01*(K50*P_01+K51*P_11+K52*P_21)+K02*(K50*P_02+K51*P_12+K52*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
    float P__51 = P_51-g0*(K10*P_50+K11*P_51+K12*P_52+K50*P_01+K51*P_11+K52*P_21)+g0_2*(K10*(K50*P_00+K51*P_10+K52*P_20)+K11*(K50*P_01+K51*P_11+K52*P_21)+K12*(K50*P_02+K51*P_12+K52*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
    float P__52 = P_52-g0*(K20*P_50+K21*P_51+K22*P_52+K50*P_02+K51*P_12+K52*P_22)+g0_2*(K20*(K50*P_00+K51*P_10+K52*P_20)+K21*(K50*P_01+K51*P_11+K52*P_21)+K22*(K50*P_02+K51*P_12+K52*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
    float P__53 = P_53-g0*(K30*P_50+K31*P_51+K32*P_52+K50*P_03+K51*P_13+K52*P_23)+g0_2*(K30*(K50*P_00+K51*P_10+K52*P_20)+K31*(K50*P_01+K51*P_11+K52*P_21)+K32*(K50*P_02+K51*P_12+K52*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
    float P__54 = P_54-g0*(K40*P_50+K41*P_51+K42*P_52+K50*P_04+K51*P_14+K52*P_24)+g0_2*(K40*(K50*P_00+K51*P_10+K52*P_20)+K41*(K50*P_01+K51*P_11+K52*P_21)+K42*(K50*P_02+K51*P_12+K52*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
    float P__55 = P_55-g0*(K50*P_05+K51*P_15+K52*P_25+K50*P_50+K51*P_51+K52*P_52)+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+g0_2*(K50*(K50*P_00+K51*P_10+K52*P_20)+K51*(K50*P_01+K51*P_11+K52*P_21)+K52*(K50*P_02+K51*P_12+K52*P_22));

    float xlen = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    float invlen3 = 1.0/(xlen*xlen*xlen);
    float invlen32 = (invlen3*invlen3);

    float x1_x2 = (x[1]*x[1]+x[2]*x[2]);
    float x0_x2 = (x[0]*x[0]+x[2]*x[2]);
    float x0_x1 = (x[0]*x[0]+x[1]*x[1]);

    // normalized a posteriori covariance
    P[0][0] = invlen32*(-x1_x2*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[0]*x[1]*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2])+x[0]*x[2]*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2]));
    P[0][1] = invlen32*(-x0_x2*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2])+x[0]*x[1]*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[1]*x[2]*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2]));
    P[0][2] = invlen32*(-x0_x1*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2])+x[0]*x[2]*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[1]*x[2]*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2]));
    P[0][3] = -invlen3*(-P__03*x1_x2+P__13*x[0]*x[1]+P__23*x[0]*x[2]);
    P[0][4] = -invlen3*(-P__04*x1_x2+P__14*x[0]*x[1]+P__24*x[0]*x[2]);
    P[0][5] = -invlen3*(-P__05*x1_x2+P__15*x[0]*x[1]+P__25*x[0]*x[2]);
    P[1][0] = invlen32*(-x1_x2*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[0]*x[1]*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2])+x[0]*x[2]*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2]));
    P[1][1] = invlen32*(-x0_x2*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2])+x[0]*x[1]*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[1]*x[2]*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2]));
    P[1][2] = invlen32*(-x0_x1*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2])+x[0]*x[2]*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[1]*x[2]*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2]));
    P[1][3] = -invlen3*(-P__13*x0_x2+P__03*x[0]*x[1]+P__23*x[1]*x[2]);
    P[1][4] = -invlen3*(-P__14*x0_x2+P__04*x[0]*x[1]+P__24*x[1]*x[2]);
    P[1][5] = -invlen3*(-P__15*x0_x2+P__05*x[0]*x[1]+P__25*x[1]*x[2]);
    P[2][0] = invlen32*(-x1_x2*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[0]*x[1]*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2])+x[0]*x[2]*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2]));
    P[2][1] = invlen32*(-x0_x2*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2])+x[0]*x[1]*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[1]*x[2]*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2]));
    P[2][2] = invlen32*(-x0_x1*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2])+x[0]*x[2]*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[1]*x[2]*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2]));
    P[2][3] = -invlen3*(-P__23*x0_x1+P__03*x[0]*x[2]+P__13*x[1]*x[2]);
    P[2][4] = -invlen3*(-P__24*x0_x1+P__04*x[0]*x[2]+P__14*x[1]*x[2]);
    P[2][5] = -invlen3*(-P__25*x0_x1+P__05*x[0]*x[2]+P__15*x[1]*x[2]);
    P[3][0] = -invlen3*(-P__30*x1_x2+P__31*x[0]*x[1]+P__32*x[0]*x[2]);
    P[3][1] = -invlen3*(-P__31*x0_x2+P__30*x[0]*x[1]+P__32*x[1]*x[2]);
    P[3][2] = -invlen3*(-P__32*x0_x1+P__30*x[0]*x[2]+P__31*x[1]*x[2]);
    P[3][3] = P__33;
    P[3][4] = P__34;
    P[3][5] = P__35;
    P[4][0] = -invlen3*(-P__40*x1_x2+P__41*x[0]*x[1]+P__42*x[0]*x[2]);
    P[4][1] = -invlen3*(-P__41*x0_x2+P__40*x[0]*x[1]+P__42*x[1]*x[2]);
    P[4][2] = -invlen3*(-P__42*x0_x1+P__40*x[0]*x[2]+P__41*x[1]*x[2]);
    P[4][3] = P__43;
    P[4][4] = P__44;
    P[4][5] = P__45;
    P[5][0] = -invlen3*(-P__50*x1_x2+P__51*x[0]*x[1]+P__52*x[0]*x[2]);
    P[5][1] = -invlen3*(-P__51*x0_x2+P__50*x[0]*x[1]+P__52*x[1]*x[2]);
    P[5][2] = -invlen3*(-P__52*x0_x1+P__50*x[0]*x[2]+P__51*x[1]*x[2]);
    P[5][3] = P__53;
    P[5][4] = P__54;
    P[5][5] = P__55;

    // normalized a posteriori state
    x[0] = x[0]/xlen;
    x[1] = x[1]/xlen;
    x[2] = x[2]/xlen;

    xGyroBias=x[3]*RADIANS_TO_DEGREES_MULTIPLIER; // get this for free!

    // compute Euler angles (not exactly a part of the extended Kalman filter)
    // yaw integration through full rotation matrix
    float u_nb1 = u1 - x[4];
    float u_nb2 = u2 - x[5];

    float cy = cos(yaw * DEGREES_TO_RADIANS_MULTIPLIER); //old angles (last state before integration)
    float sy = sin(yaw * DEGREES_TO_RADIANS_MULTIPLIER);
    float d = sqrt(x_last[1]*x_last[1] + x_last[2]*x_last[2]);
    float d_inv = 1.0 / d;

    // compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
    float R11 = cy * d;
    float R12 = -(x_last[2]*sy + x_last[0]*x_last[1]*cy) * d_inv;
    float R13 = (x_last[1]*sy - x_last[0]*x_last[2]*cy) * d_inv;
    float R21 = sy * d;
    float R22 = (x_last[2]*cy - x_last[0]*x_last[1]*sy) * d_inv;
    float R23 = -(x_last[1]*cy + x_last[0]*x_last[2]*sy) * d_inv;

    // update needed parts of R for yaw computation
    float R11_new = R11 + h*(u_nb2*R12 - u_nb1*R13);
    float R21_new = R21 + h*(u_nb2*R22 - u_nb1*R23);
    yaw = atan2(R21_new,R11_new) * RADIANS_TO_DEGREES_MULTIPLIER;

    // compute new pitch and roll angles from a posteriori states
    pitch = asin(-x[0]) * RADIANS_TO_DEGREES_MULTIPLIER;
    roll = atan2(x[1],x[2]) * RADIANS_TO_DEGREES_MULTIPLIER;
    
    // save the estimated non-gravitational acceleration
    a[0] = z0-x[0]*g0;
    a[1] = z1-x[1]*g0;
    a[2] = z2-x[2]*g0;
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
    
    //select registers for FIFO
    writeRegister(BCM2835_SPI_CS0,ICM20689_FIFO_EN,0x78);
    writeRegister(BCM2835_SPI_CS1,ICM20689_FIFO_EN,0x78);

    //clear data paths and reset FIFO (keep i2c disabled)
    writeRegister(BCM2835_SPI_CS0,ICM20689_USER_CTRL,0x15);
    writeRegister(BCM2835_SPI_CS1,ICM20689_USER_CTRL,0x15);

	// load up calibration data from Arduino
    // (registers 	10=samplePeriod, 11=accBiasX,
    // 12=accBiasY, 13=accBiasZ, 14=accScaleX, 15=accScaleY, 16=accScaleZ,
    // 17=gyroBiasX, 18=gyroBiasY, 19=gyroBiasZ, 20=gyroScaleX, 21=gyroScaleY,
    // 22=gyroScaleZ, 5=accBiasXYZ(all in one go 12 bytes), 6=accScaleXYZ(all in one go 12 bytes)
    // 7=gyroBiasXYZ(all in one go 12 bytes), 8=gyroScaleXYZ(all in one go 12 bytes))

    I2C_BUFFER[0]=10;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 10 (samplePeriod)
    bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
    float result;
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.samplePeriod = result;
	
    I2C_BUFFER[0]=5;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 5 (accBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.accBiasX = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.accBiasY = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.accBiasZ = result;
	
    I2C_BUFFER[0]=6;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 6 (accScaleXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.accScaleX = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.accScaleY = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.accScaleZ = result;

    I2C_BUFFER[0]=7;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 7 (gyroBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.gyroBiasX = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.gyroBiasY = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.gyroBiasZ = result;

    I2C_BUFFER[0]=8;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 8 (gyroScaleXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.gyroScaleX = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.gyroScaleY = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.gyroScaleZ = result;

    //start FIFO - keep i2c disabled
    writeRegister(BCM2835_SPI_CS0,ICM20689_USER_CTRL,0x50);
    writeRegister(BCM2835_SPI_CS1,ICM20689_USER_CTRL,0x50);
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
