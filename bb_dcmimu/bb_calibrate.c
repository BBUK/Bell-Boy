//gcc bb_calibrate.c -o bb_calibrate

/*
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
* This program is a calibration routine for IMU accelerometer sensors.  
* The program only needs to be run once after the Bell Boy device has been
* constructed.  The calibration "should" remain valid for the life of the device.
* There is, however, no harm in rerunning the calibration.
*
* This program works by using gravity measurements in the positive and negative
* directions of the Y and Z axes.  The X axis is not used as its accelerometer
* (and its calibration) is irrelevant for the Bell Boy application.
* 
* The result is an offset (which bb_dcmimu adds to the raw readings) 
* and a scale (which bb_dcmimu multiplies the raw readings by).
* 
* To use this get a completely flat surface ready.  Test the "flatness" of the 
* surface using a bulls eye spirit level.  Place one large flat side of the Bell Boy
* device on the surface, switch it on, connect to its wifi and run the bb_calibrate
* program over ssh. Note that you will be turning the device so that it is on
* the other flat side and on each of the "long" edges.  I have called these four
* orientations North, South, East and West.
* The program starts by taking some gyroscope measurements (overall biasses).  These are 
* not used in the calibration itself but are used to work out whether the devioe
* is still enough to make an accurate accelerometer reading.
* The program then takes the readings for the orientation it is in.  You will 
* probably see something like "insufficiently still" a few times before a reading
* is taken and reported.  Once this happens turn the device to one of the other
* orientations and wait for a reading to be taken (and so on).
* If you get a report "Not in the right position" this simply says that the device is still
* but probably not oriented correctly.  Wait to see if the error clears and also check
* the flatness of the surface you have placed the device on.
* The program exits once it has taken all its measurements and saves the Y and Z offset
* and scale values (four values) to /boot/bb_calibrations.
* The saved file is read and used by bb_dcmimu (the main grabber program).  If, however,
* bb_dcmimu does not see the file, it just uses uncalibrated values - you may
* not notice any difference!
*
* Finally note that the offset and scale values are temperature dependent.  If you
* wanted to do some scientific analysis then the values should be measured at different
* temperatures and some form of linear regression analysis should take place so that
* you would know what scale and offset values to apply.  That's why the IMUs have
* a user accessible temperature reading on them.  Testing showed that the effect of
* temperature was really small (to small to worry about really).  
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
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <dirent.h>
#include "bb_calibrate.h"

#ifndef NULL
#define NULL 0
#endif

#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif

#define I2CDEV "/dev/i2c-1"

int NXP_fd_gyro = -1;
int NXP_fd_accel = -1;

float NXP_gyro_scale_factor = 0;
float NXP_accel_scale_factor = 0;
float threshold = 2.0;

float GYRO_BIAS[3] = { 0.0, 0.0, 0.0 };

FILE *fd_write_out;

int SWAPXY = 1; // swap XY for different mounting.  Swap is applied before rotation.

volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

int main(int argc, char const *argv[]){
    float return_values[6];
    int temperature;
    float north,south,east,west;
    int count = 0, nsew = 0;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
    
    
    if(NXP_setup() < 0) return -1;
    printf("Getting gyro biasses\n");

    while(sig_exit == 0){
        for(count = 0; count < 4; ++count){ // must be still for about 2 seconds
            if(NXP_get_calibration_values(return_values, &temperature) < 0) break;
        }
        if(count != 4) {
            printf("Insufficiently still ...\n");
            usleep(500000);
            continue;
        }

        GYRO_BIAS[0] = return_values[0];
        GYRO_BIAS[1] = return_values[1];
        GYRO_BIAS[2] = return_values[2];
        printf("Biasses read OK\n");
        threshold = 0.2;
        break;
    }

    while(sig_exit == 0){
        for(count = 0; count < 4; ++count){ // must be still for about 2 seconds
            if(NXP_get_calibration_values(return_values, &temperature) < 0) break;
        }
        if(count != 4) {
            printf("Insufficiently still ...\n"); 
            usleep(500000);
            continue;
        }
        if(return_values[4] > 0.90 && abs(return_values[5]) < 0.10  && abs(return_values[3]) < 0.10){
            if(nsew & 1){
                printf("North already read, please flip to another face.\n");
            } else {
                nsew |= 1;
                north = return_values[4];
                printf("North read\n");
            }
        } else if(return_values[4] < -0.90 && abs(return_values[5]) < 0.10  && abs(return_values[3]) < 0.10) {
            if(nsew & 2){
                printf("South already read, please flip to another face.\n");
            } else {
                nsew |= 2;
                south = return_values[4];
                printf("South read\n");
            }
        } else if(return_values[5] > 0.90 && abs(return_values[4]) < 0.10  && abs(return_values[3]) < 0.10) {
            if(nsew & 4){
                printf("East already read, please flip to another face.\n");
            } else {
                nsew |= 4;
                east = return_values[5];
                printf("East read\n");
            }
        } else if(return_values[5] < -0.90 && abs(return_values[4]) < 0.10  && abs(return_values[3]) < 0.10) {
            if(nsew & 8){
                printf("West already read, please flip to another face.\n");
            } else {
                nsew |= 8;
                west = return_values[5];
                printf("West read\n");
            }
        } else {
            printf("Not in the right position..\n");
        }
        if(nsew == 15){
            printf("Success!\n");
            printf("North= %f, South= %f, East= %f and West= %f\n", north,south,east,west);
            float nsoffset = -((north+south)/2.0);
//               float nsoffset = -((north+south)/(north-south));
            float nsscale = 2.0/(north-south);
            north += nsoffset;
            south += nsoffset;
            north *= nsscale;
            south *= nsscale;
            printf("Y offset = %f, Y scale = %f\n",nsoffset, nsscale);
            float ewoffset = -((east+west)/2.0);
//                float ewoffset = -((east+west)/(east-west));
            float ewscale = 2.0/(east-west);
            east += ewoffset;
            west += ewoffset;
            east *= ewscale;
            west *= ewscale;
            printf("Z offset = %f, Z scale = %f\n",ewoffset, ewscale);
            printf("Temperature: %d\n",temperature);
            printf("North= %f, South= %f, East= %f and West= %f\n", north,south,east,west);
            close(NXP_fd_accel);
            NXP_fd_accel = -1;
            close(NXP_fd_gyro);
            NXP_fd_gyro = -1;
            fd_write_out = fopen("/boot/bb_calibrations","w");  // "w"
            if(fd_write_out == NULL) {
                fprintf(stderr, "Could not open file for writing\n");
                return 1;
            } else {
                fprintf(fd_write_out,"YO:%+06.3f,YS:%+06.3f,ZO:%+06.3f,ZS:%+06.3f,TM:%d\n", nsoffset,nsscale,ewoffset,ewscale,temperature);
                fclose(fd_write_out);
                fd_write_out = NULL;
                return 0;
            }
        }
    }
    if (NXP_fd_gyro != -1) close(NXP_fd_gyro);
    if (NXP_fd_accel != -1) close(NXP_fd_accel);
    if (fd_write_out != NULL) fclose(fd_write_out);
    return 0;
}
  
int NXP_setup(void){

	if ((NXP_fd_gyro = open(I2CDEV, O_RDWR)) < 0) {
        printf("Could not open i2c device.   Is i2c_dev loaded?");
        return -1;
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
        printf("Could not open IMU.   Is IMU connected?");
        return -1;
	}

	if ((NXP_fd_accel = open(I2CDEV, O_RDWR)) < 0) {
        printf("Could not open i2c device.   Is i2c_dev loaded?");
        return -1;
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
        printf("Could not open IMU.   Is IMU connected?");
        return -1;
	}

    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x02); 
    NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_500DPS;
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x80);  // enable fifo 
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x0A);


    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG2, 0x02);  // High resolution
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_MCTRL_REG1, 0x03); // magnetometer must be active
    i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x00); // 2g 
    NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_2G;
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x15); // 200 samples/sec
    return 0;
}

int NXP_get_calibration_values(float *results, int *temperature){
    int count;
    float gyro_data[3];
    float accel_data[3];
    float u0=0.0,u1=0.0,u2=0.0,z0=0.0,z1=0.0,z2=0.0;

    usleep(6000);
    NXP_read_accel_data(accel_data); // ditch a sample or two
    NXP_read_gyro_data(gyro_data);
    usleep(6000);
    NXP_read_accel_data(accel_data);
    NXP_read_gyro_data(gyro_data);
	
    for(count = 0; count < 100; ++count){
        usleep(6000);
        NXP_read_accel_data(accel_data);
        NXP_read_gyro_data(gyro_data);

        u0 += gyro_data[0];
        u1 += gyro_data[1];
        u2 += gyro_data[2];
        z0 += accel_data[0];
        z1 += accel_data[1];
        z2 += accel_data[2];
        if(abs(gyro_data[0]) > threshold || abs(gyro_data[1]) > threshold || abs(gyro_data[2]) > threshold){
            return -1;
        }
    }
    results[0] = u0/100.0;
    results[1] = u1/100.0;
    results[2] = u2/100.0;
    results[3] = z0/100.0;
    results[4] = z1/100.0;
    results[5] = z2/100.0;
 
    *temperature = i2c_smbus_read_byte_data(NXP_fd_accel, NXP_ACCEL_REGISTER_TEMP);
    if(*temperature >= 0x80) *temperature -= 0x100;
    return 0;
}


void NXP_read_gyro_data(float *values){
    __u8 returns[6];
    i2cReadBlockData(NXP_fd_gyro, NXP_GYRO_REGISTER_OUT_X_MSB, 6, returns);
    values[0]=(float)((returns[0] << 8) + returns[1]); 
    values[1]=(float)((returns[2] << 8) + returns[3]);
    values[2]=(float)((returns[4] << 8) + returns[5]);
    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] *= NXP_gyro_scale_factor;
    }
    if(SWAPXY){
        float temp;
        temp = values[0];
        values[0] = values[1];
        values[1] = temp;
    }
    
    values[0] -= GYRO_BIAS[0]; // adjust for gyro bias
    values[1] -= GYRO_BIAS[1];
    values[2] -= GYRO_BIAS[2];

}

void NXP_read_accel_data(float *values){
    __u8 returns[6];
    i2cReadBlockData(NXP_fd_accel, NXP_ACCEL_REGISTER_OUT_X_MSB, 6, returns);
    values[0]=(float)((returns[0] << 8) + returns[1]); 
    values[1]=(float)((returns[2] << 8) + returns[3]);
    values[2]=(float)((returns[4] << 8) + returns[5]);
    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] /= 4; // 14 bit data
        values[i] *= NXP_accel_scale_factor;
    }
    if(SWAPXY){
        float temp;
        temp = values[0];
        values[0] = values[1];
        values[1] = temp;
    }
}


/*
  i2c functions
*/


__s32 i2cReadInt(int fd, __u8 address) {
	__s32 res = i2c_smbus_read_word_data(fd, address);
	if (0 > res) {
		close(fd);
		exit(1);
	}
	res = ((res<<8) & 0xFF00) | ((res>>8) & 0xFF);
	return res;
}

//Write a byte
void i2cWriteByteData(int fd, __u8 address, __u8 value) {
	if (0 > i2c_smbus_write_byte_data(fd, address, value)) {
		close(fd);
		exit(1);
	}
}

// Read a block of data
void i2cReadBlockData(int fd, __u8 address, __u8 length, __u8 *values) {
	if (0 > i2c_smbus_read_i2c_block_data(fd, address,length,values)) {
		close(fd);
		exit(1);
	}
}
    
__s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data) {
	struct i2c_smbus_ioctl_data args;
	__s32 err;
	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	err = ioctl(file, I2C_SMBUS, &args);
	if (err == -1)
		err = -errno;
	return err;
}

__s32 i2c_smbus_write_quick(int file, __u8 value) {
	return i2c_smbus_access(file, value, 0, I2C_SMBUS_QUICK, NULL);
}

__s32 i2c_smbus_read_byte(int file) {
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	if (err < 0)
		return err;
	return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte(int file, __u8 value) {
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

__s32 i2c_smbus_read_byte_data(int file, __u8 command) {
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0)
		return err;
	return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value) {
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

__s32 i2c_smbus_read_word_data(int file, __u8 command) {
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data);
	if (err < 0)
		return err;
	return 0x0FFFF & data.word;
}

__s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value){
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}

__s32 i2c_smbus_process_call(int file, __u8 command, __u16 value) {
	union i2c_smbus_data data;
	data.word = value;
	if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_PROC_CALL, &data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

/* Returns the number of read bytes */
__s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values){
	union i2c_smbus_data data;
	int i, err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BLOCK_DATA, &data);
	if (err < 0)
		return err;

	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

__s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length, const __u8 *values) {
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BLOCK_DATA, &data);
}

/* Returns the number of read bytes */
__s32 i2c_smbus_read_i2c_block_data(int file, __u8 command, __u8 length, __u8 *values) {
	union i2c_smbus_data data;
	int i, err;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       length == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN :
				I2C_SMBUS_I2C_BLOCK_DATA, &data);
	if (err < 0)
		return err;
	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

__s32 i2c_smbus_write_i2c_block_data(int file, __u8 command, __u8 length, const __u8 *values) {
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_I2C_BLOCK_BROKEN, &data);
}

/* Returns the number of read bytes */
__s32 i2c_smbus_block_process_call(int file, __u8 command, __u8 length, __u8 *values) {
	union i2c_smbus_data data;
	int i, err;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	err = i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
			       I2C_SMBUS_BLOCK_PROC_CALL, &data);
	if (err < 0)
		return err;
	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

