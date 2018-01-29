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

#include "Python.h"
#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "bb_dcmimu.h"


#ifndef NULL
#define NULL 0
#endif

#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif

#define g0 9.8189
#define g0_2 (g0*g0)
#define q_dcm2 (0.1*0.1)
#define q_gyro_bias2 (0.0001*0.0001)
#define r_acc2 (0.5*0.5)
#define r_a2 (10*10)
#define q_dcm2_init (1*1)
#define q_gyro_bias2_init (0.1*0.1)

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 

int NXP_fd_gyro = -1;
int NXP_fd_accel = -1;
//int MPU6050_fd = -1;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float a[3];
float sample_period = 0.0;
float gyro_bias[3] = { 0.0, 0.0, 0.0 };
int DEBUG = 0;

float NXP_gyro_scale_factor = 0;
float NXP_accel_scale_factor = 0;
int rotations[6] = { 1,1,1 }; //set rotation values for mounting IMU device gx gy gx ax ay az
int SWAPXY = 0; // swap XY for different mounting.  Swap is applied before rotation.

static PyObject *set_gyro_bias(PyObject *self, PyObject *args){
    float bx, by, bz;
    if (!PyArg_ParseTuple(args, "fff;set_gyro_bias expects three floats", &bx, &by, &bz)) return NULL;
    gyro_bias[0] = bx;
    gyro_bias[1] = by;
    gyro_bias[2] = bz;

    return Py_BuildValue("i", 0); 
}


static PyObject *set_rotations(PyObject *self, PyObject *args){
    int r0, r1, r2;
    if (!PyArg_ParseTuple(args, "iii", &r0, &r1, &r2)) return NULL;
    if ( abs(r0) != 1 || abs(r1) != 1 || abs(r2) != 1 ) return Py_BuildValue("i", -1);
    rotations[0] = r0;
    rotations[1] = r1;
    rotations[2] = r2;

    return Py_BuildValue("i", 0); 
}

static PyObject *set_swap_XY(PyObject *self){
    SWAPXY = 1;
    return Py_BuildValue("i", 0); 
}

static PyObject *clear_swap_XY(PyObject *self){
    SWAPXY = 0;
    return Py_BuildValue("i", 0); 
}

static PyObject *set_debug(PyObject *self){
    DEBUG = 1;
    return Py_BuildValue("i", 0); 
}

static PyObject *clear_debug(PyObject *self){
    DEBUG = 0;
    return Py_BuildValue("i", 0); 
}

static PyObject *get_sample_period(PyObject *self){
    return Py_BuildValue("f", sample_period); 
}

/*
Wrapper for the calculate function
*/

static PyObject *w_calculate(PyObject *self, PyObject *args){
    float u0, u1, u2, z0, z1, z2, h;
    if (!PyArg_ParseTuple(args, "fffffff", &u0, &u1, &u2, &z0, &z1, &z2, &h)) return NULL;
    calculate(u0, u1, u2, z0, z1, z2, h);
    return Py_BuildValue("fff", roll, pitch, yaw);
}

static PyObject *w_NXP_start_fifos(PyObject *self, PyObject *args){
    int ODR, FS_gyro, FS_accel;
    if (!PyArg_ParseTuple(args, "iii;NXP_start_fifos expects 3 integers, ODR, gyro fs and accelerometer fs.", &ODR, &FS_gyro, &FS_accel)) return NULL;
    if (ODR != 50 && ODR != 100 && ODR != 200 && ODR != 400 && ODR != 800) return NULL;
    if (FS_gyro != 500 && FS_gyro != 1000) return NULL;
    if (FS_accel != 2 && FS_accel != 4) return NULL;
    if(sample_period == 0.0) sample_period = 1.0/ODR;
    return Py_BuildValue("i", NXP_start_fifos(ODR, FS_gyro, FS_accel));
}

static PyObject *w_NXP_stop_fifos(PyObject *self){
    NXP_stop_fifos();
    return Py_BuildValue("i", 0);
}

/*
Tests that the IMU is accessible over i2c.  Returns negative number if tests fails, zero otherwise
*/
static PyObject *NXP_test(PyObject *self){
    if (NXP_fd_gyro != -1) return Py_BuildValue("i", -10);
	if ((NXP_fd_gyro = open("/dev/i2c-1", O_RDWR)) < 0) {
        NXP_fd_gyro = -1;
        return Py_BuildValue("i", -1);
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
	    close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return Py_BuildValue("i", -2);
	}
    if (i2c_smbus_read_byte_data(NXP_fd_gyro, NXP_GYRO_REGISTER_WHO_AM_I) != NXP_GYRO_ID) {
        close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return Py_BuildValue("i", -3);
    }
    close(NXP_fd_gyro);
    NXP_fd_gyro = -1;

    if (NXP_fd_accel != -1) return Py_BuildValue("i", -11);
	if ((NXP_fd_accel = open("/dev/i2c-1", O_RDWR)) < 0) {
        NXP_fd_accel = -1;
        return Py_BuildValue("i", -4);
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
	    close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return Py_BuildValue("i", -5);
	}
    if (i2c_smbus_read_byte_data(NXP_fd_accel, NXP_ACCEL_REGISTER_WHO_AM_I) != NXP_ACCEL_ID){
        close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return Py_BuildValue("i", -6);
    }
    close(NXP_fd_accel);
    NXP_fd_accel = -1;

    return Py_BuildValue("i", 0);
}

static PyObject *NXP_fifo_timer(PyObject *self, PyObject *args){
    struct timeval start, stop;
    int cco, ODR, loops;
    float dummy[3], gyro_result = 0, accel_result = 0;
    if (!PyArg_ParseTuple(args, "i;NXP_fifo_timer expects an integer ODR rate", &ODR)) return NULL;
    if (ODR != 50 && ODR != 100 && ODR != 200 && ODR != 400 && ODR != 800) return NULL;
    if (NXP_start_fifos(ODR,500,2) < 0){
        NXP_stop_fifos();
        return NULL;
    }
    loops = (int)(ODR/25);
    for(int i = 0; i < loops; ++i){
        while (NXP_read_gyro_fifo_count() != 0) NXP_read_gyro_data(dummy);
        while (NXP_read_gyro_fifo_count() == 0);
        gettimeofday(&start, NULL);
        NXP_read_gyro_data(dummy);
        Py_BEGIN_ALLOW_THREADS
        // Py_BLOCK_THREADS // Py_UNBLOCK_THREADS
        while (NXP_read_gyro_fifo_count() < 26) usleep((int)(2/ODR)*1000000);  // sleep for about 2 periods
        Py_END_ALLOW_THREADS
        while(1){
            cco = NXP_read_gyro_fifo_count();
            if(cco == 30) {
                gettimeofday(&stop, NULL);
                gyro_result += (float)((stop.tv_sec-start.tv_sec)+(float)(stop.tv_usec-start.tv_usec)/1000000.0);
                break;
            }
            if(cco > 30) {
                i -= 1;
                break;
            }
        }
    }
    for(int i = 0; i < loops; ++i){
        while (NXP_read_accel_fifo_count() != 0) NXP_read_accel_data(dummy);
        while (NXP_read_accel_fifo_count() == 0);
        gettimeofday(&start, NULL);
        NXP_read_accel_data(dummy);
        Py_BEGIN_ALLOW_THREADS
        // Py_BLOCK_THREADS // Py_UNBLOCK_THREADS
        while (NXP_read_accel_fifo_count() < 26) usleep((int)(2/ODR)*1000000);
        Py_END_ALLOW_THREADS
        while(1){
            cco = NXP_read_accel_fifo_count();
            if(cco == 30) {
                gettimeofday(&stop, NULL);
                accel_result += (float)((stop.tv_sec-start.tv_sec)+(float)(stop.tv_usec-start.tv_usec)/1000000.0);
                break;
            }
            if(cco > 30) {
                i -= 1;
                break;
            }
        }
    }
    NXP_stop_fifos();
    sample_period = gyro_result/(30*loops);
    if(DEBUG) return Py_BuildValue("ff", accel_result/(30*loops), gyro_result/(30*loops));
    return Py_BuildValue("f", sample_period); 
}

static PyObject *NXP_pull_data(PyObject *self){
    float gyro_data[3], accel_data[3], accTang;
    static float last_x_gyro = 0.0, last_angle = 0.0, angle_correction = 0.0;    
    int accel_count, number_to_pull, i, max_string, duplicate;
    Py_ssize_t string_count;
    char * output;
    PyObject * result;
    string_count = 0;
//    if (sample_period == 0.0) return Py_BuildValue("s", "ESAM:\n"); // sample period needs to be measured first
    if (NXP_fd_gyro == -1 || NXP_fd_accel == -1) return Py_BuildValue("s", "EFIF:\n"); // fifos need to be started
    
    accel_count = NXP_read_accel_fifo_count();
    number_to_pull = NXP_read_gyro_fifo_count();
    if(number_to_pull == 0 || accel_count == 0) return Py_BuildValue("s", "");
    Py_BEGIN_ALLOW_THREADS
    if((accel_count - number_to_pull) >= 2) NXP_read_accel_data(accel_data);   //acceleration ahead, ditch a sample from the acceleration fifo
    if(DEBUG){
        max_string = 98 + number_to_pull * 146;
    } else {
        max_string = 32 + number_to_pull * 30;  // 30 characters per output plus 32 for safety!
    }
    output = malloc(max_string);
    if (accel_count >= 31 || number_to_pull >= 31){ // overflow (or nearly so)
        string_count += sprintf(&output[string_count],"EOVF:\n");
    }

    for(i=0; i<number_to_pull; ++i){
        if (accel_count != 0) {
            NXP_read_accel_data(accel_data);
            accel_count -= 1;
        }
        NXP_read_gyro_data(gyro_data);

        calculate(gyro_data[0],gyro_data[1],gyro_data[2], accel_data[0],accel_data[1],accel_data[2],sample_period);
        accTang = (gyro_data[0] - last_x_gyro)/sample_period; // assumes rotation of bell is around y axis
        roll += angle_correction;                                   // need to correct reported angles to match
        if ((last_angle - roll) > 250 && angle_correction != 360){    // the system we are using 0 degrees = bell at
            angle_correction = 360;                                 // balance at handstroke, 360 degrees = bell at
            roll += 360.0;                                          // balance at backstroke. 180 degrees = BDC. 
        } else if ((last_angle - roll) < -250 && angle_correction != 0) {                                                 
            angle_correction = 0;
            roll -= 360.0;
        }
        if(DEBUG){
            string_count += sprintf(&output[string_count],"A:%+07.1f,R:%+07.1f,C:%+07.1f,P:%+07.1f,Y:%+07.1f,AX:%+06.3f,AY:%+06.3f,AZ:%+06.3f,GX:%+07.1f,GY:%+07.1f,GZ:%+07.1f,X:%+06.3f,Y:%+06.3f,Z:%+06.3f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang, pitch, yaw, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2],a[0],a[1],a[2]);
        } else {
            string_count += sprintf(&output[string_count],"A:%+07.1f,R:%+07.1f,C:%+07.1f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang);
        }
        if((string_count + 30) >= max_string ) break; // shouldn't happen!
        last_x_gyro = gyro_data[0];
        last_angle = roll;
    }
    Py_END_ALLOW_THREADS
    if(DEBUG && (string_count + 30) >= max_string ) {
        free(output);
        return Py_BuildValue("s", "ESTR:\n");
    }
    result = Py_BuildValue("s#", output, string_count - 1); // -1 to ditch the last /n
    free(output);
    return result;
}

/*
Gets an initial set of values for accelerometer and gyro (smoothed)
This is used when a ringing session is started to ensure that the bell is up
and stationary.  It is also used to deal with two main mounting positions of
the device (so the user does not have to worry about which way the bell
is to rotate when initially mounting the device (the python method calling this
makes the measurement and adjusts by changing the rotation array/tuple).
I will probably use this also to make an initial angle correction when the device
is first switched on - the device may not be quite horizontally mounted or 
the headstock itself may not be quite parallel with the ground! 
*/

static PyObject *NXP_get_orientation(PyObject *self){
    int count;
    float u0=0, u1=0, u2=0, z0=0, z1=0, z2=0, gyro_data[3], accel_data[3];
    
    if (NXP_fd_gyro != -1) return Py_BuildValue("i", -10);
	if ((NXP_fd_gyro = open("/dev/i2c-1", O_RDWR)) < 0) {
        NXP_fd_gyro = -1;
        return Py_BuildValue("i", -1);
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
	    close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return Py_BuildValue("i", -2);
	}
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x02);  // 500 degrees/sec
    NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_500DPS;
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x06); // 400 samples/sec

    if (NXP_fd_accel != -1) return Py_BuildValue("i", -11);
	if ((NXP_fd_accel = open("/dev/i2c-1", O_RDWR)) < 0) {
        NXP_fd_accel = -1;
        return Py_BuildValue("i", -4);
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
	    close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return Py_BuildValue("i", -5);
	}
    Py_BEGIN_ALLOW_THREADS
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG2, 0x02);  // High resolution
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_MCTRL_REG1, 0x00); // disable magnetometer
    i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x00); // 2g full scale
    NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_2G;
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x0D); // 400 samples/sec

    NXP_read_accel_data(accel_data);
    NXP_read_gyro_data(gyro_data);
 
    for(count = 0; count < 50; ++count){
        usleep(5000);
        NXP_read_accel_data(accel_data);
        NXP_read_gyro_data(gyro_data);

        u0 += gyro_data[0];
        u1 += gyro_data[1];
        u2 += gyro_data[2];
        z0 += accel_data[0];
        z1 += accel_data[1];
        z2 += accel_data[2];
    }
    u0 = u0/50.0;
    u1 = u1/50.0;
    u2 = u2/50.0;
    z0 = z0/50.0;
    z1 = z1/50.0;
    z2 = z2/50.0;
    
    close(NXP_fd_accel);
    NXP_fd_accel = -1;
    close(NXP_fd_gyro);
    NXP_fd_gyro = -1;
    Py_END_ALLOW_THREADS
    return Py_BuildValue("[ffffff]", u0, u1, u2, z0, z1, z2);
}
/*
Method definitions
*/

static PyMethodDef dcmimu_methods[] = {
    {"calculate", w_calculate, METH_VARARGS, "Calculate roll angle.  Can but shouldn't be called directly."},
    {"NXP_start_fifos", w_NXP_start_fifos, METH_VARARGS, "Starts the two NXP fifos."},
    {"set_gyro_bias", set_gyro_bias, METH_VARARGS, "Sets gyro bias.  3 floats, x, y and z representing the number to be subtracted from the gyros.  Applied after swaps and rotations"},
    {"set_rotations", set_rotations, METH_VARARGS, "Sets the rotation of the sensor."},
    {"NXP_stop_fifos", (PyCFunction)w_NXP_stop_fifos, METH_NOARGS, "Stops the two NXP fifos and clears file descriptors etc."},
    {"NXP_test", (PyCFunction)NXP_test, METH_NOARGS, "Tests that the IMU is accessible over i2c."},
    {"NXP_pull_data", (PyCFunction)NXP_pull_data, METH_NOARGS, "Pulls data from the NXP devices and returns a string in a format suitable for the front end 'A:+XXXX.X,R:+YYYY.Y,C:+ZZZZ.Z'"},
    {"set_debug", (PyCFunction)set_debug, METH_NOARGS, "Sets the debug flag - pushes more data out"},
    {"clear_debug", (PyCFunction)clear_debug, METH_NOARGS, "Clears the debug flag"},
    {"set_swap_XY", (PyCFunction)set_swap_XY, METH_NOARGS, "Sets the swapXY flag - swaps the readings for the X and Y axes - eases making changes for mounting.  Swap is applied before rotations!"},
    {"clear_swap_XY", (PyCFunction)clear_swap_XY, METH_NOARGS, "Clears the swapXY flag"},
    {"get_sample_period", (PyCFunction)get_sample_period, METH_NOARGS, "Gets the current sample period."},
    {"NXP_get_orientation", (PyCFunction)NXP_get_orientation, METH_NOARGS, "Reports initial accelerometer and gyro readings.  Smoothed over 50 samples."},
    {"NXP_fifo_timer", NXP_fifo_timer, METH_VARARGS, "Times the rates of the two fifos and returns the result (seconds)."},
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC
initdcmimu(void) {
    Py_InitModule("dcmimu", dcmimu_methods);
}

/* 
Internal functions 
*/

int NXP_start_fifos(int ODR, int gyro_fs, int accel_fs){
    if (NXP_fd_gyro != -1) return -1;
	if ((NXP_fd_gyro = open("/dev/i2c-1", O_RDWR)) < 0) {
        NXP_fd_gyro = -1;
        return -1;
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
	    close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return -1;
	}

    if (NXP_fd_accel != -1) return -1;
	if ((NXP_fd_accel = open("/dev/i2c-1", O_RDWR)) < 0) {
        NXP_fd_accel = -1;
        return -1;
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
	    close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return -1;
	}

    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x00);  // device to standby
    switch(gyro_fs){
        case 1000:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x01);  
            NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_1000DPS;
            break;
        case 500:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x02); 
            NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_500DPS;
            break;
    }
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x80);  // enable fifo 
    switch(ODR){
        case 800:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x02);
            break;
        case 400:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x06);
            break;
        case 200:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x0A);
            break;
        case 100:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x0E);
            break;
        case 50:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x12);
            break;
    }

    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG2, 0x02);  // High resolution
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_MCTRL_REG1, 0x00); // disable magnetometer
    switch(accel_fs){
        case 4:
            i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x01); 
            NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_4G;
            break;
        case 2:
            i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x00);
            NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_2G;
    }

    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x80);  // enable fifo

    switch(ODR){
        case 800:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x05);
            break;
        case 400:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x0D);
            break;
        case 200:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x15);
            break;
        case 100:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x1D);
            break;
        case 50:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x25);
            break;
    }
    return 0;
}

void NXP_stop_fifos(){
    if (NXP_fd_gyro != -1){
        i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00); 
        close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
    }

    if (NXP_fd_accel != -1){
        i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_F_SETUP, 0x00); 
        close(NXP_fd_accel);
        NXP_fd_accel = -1;
    }
}

void NXP_clear_fifos(){
    if (NXP_fd_gyro != -1){
        i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00);
        i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x80); 
    }

    if (NXP_fd_accel != -1){
        i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_F_SETUP, 0x00); 
        i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_F_SETUP, 0x80); 
    }
}

int NXP_read_gyro_fifo_count(){
  return i2c_smbus_read_byte_data(NXP_fd_gyro, NXP_GYRO_REGISTER_F_STATUS) & 0x3F;
}

int NXP_read_accel_fifo_count(){
  return i2c_smbus_read_byte_data(NXP_fd_accel, NXP_GYRO_REGISTER_STATUS) & 0x3F;
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
    values[0] *= rotations[0]; // deal with package being mounted differently
    values[1] *= rotations[1];
    values[2] *= rotations[2];

    values[0] -= gyro_bias[0]; // adjust for gyro bias
    values[1] -= gyro_bias[1];
    values[2] -= gyro_bias[2];
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
    values[0] *= rotations[0]; // deal with package being mounted differently
    values[1] *= rotations[1];
    values[2] *= rotations[2];
}


/* 
This function does some magic with an Extended Kalman Filter to produce roll
pitch and yaw measurements from accelerometer/gyro data.  Of the filters tested,
it produced the best results.
Github is here: https://github.com/hhyyti/dcm-imu
inputs are 
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

    // compute Euler angles (not exactly a part of the extended Kalman filter)
    // yaw integration through full rotation matrix
    float u_nb1 = u1 - x[4];
    float u_nb2 = u2 - x[5];

    float cy = cos(yaw); //old angles (last state before integration)
    float sy = sin(yaw);
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

