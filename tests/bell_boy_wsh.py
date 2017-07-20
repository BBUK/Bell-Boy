# Copyright (c) 2017 Peter Budd. All rights reserved
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
# associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
# subject to the following conditions:
#     * The above copyright notice and this permission notice shall be included in all copies or substantial 
#       portions of the Software.
#     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#       BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
#       IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
#       WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
#       SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER, 
#       ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY 
#       FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW

# This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
# bell rope.  The Bell-Boy uses tangential acceleration of the bell as a proxy for force applied.  The
# hardware is currently a Pi Zero running Arch Linux, a MPU6050 breakout board and the official Pi Wifi dongle

# This version of the pywebsocket file is used to test a single accelerometer

# I pulled snippets of code from loads of places so if any of you recognise anything you wrote - thanks! 

# The script below is called by pywebsocket https://github.com/google/pywebsocket.  Websockets are needed 
# by the front-end Javascript which pulls data from the Bell-Boy device.

# The MPU6050 device is connected to the I2C-1 GPIOs on the Raspberry Pi (GPIOs 2 and 3, header pins 3 and 5)

from multiprocessing import Process, Queue, Event
from time import time,sleep,strftime
from os import listdir,system
from os.path import isfile, join
from math import atan, atan2, sqrt
from subprocess import call
import sys
import os.path
import math
from MPUConstants import MPUConstants as C
import smbus

DEV_ADDR_1=0x68
i2c=smbus.SMBus(1)
sample_period = 1/50.0 # this is the current sample period (50 samples/sec].  This needs to reflect 
                       # MPU6050_RA_SMPLRT_DIV in enable_FIFO in the MPU6050 class

dataQueue=Queue()

worker = None
workerevent = None

def web_socket_do_extra_handshake(request):
    pass

def web_socket_transfer_data(request):
    global worker,workerevent
    while True:
        instruction = request.ws_stream.receive_message()
        if instruction is None:
            return
        if instruction[:5] == "STRT:":
##TODO: check whether there is already a thread running and end it
            while (not dataQueue.empty()):
                dataQueue.get(False)
            workerevent = Event()
            fileName = instruction[5:]
            if fileName == "":
                fileName = "CurrentRecording" + strftime("%Y%m%d-%H%M%S")
            worker = process_sample(fileName, sample_period, dataQueue,workerevent)
            worker.start()
        elif instruction[:5] == "STOP:":
            if worker is not None:
                workerevent.set()
                sleep(1) # allow existing server threads to stop but must dump any further LDATS: recieved (done in JS)
                request.ws_stream.send_message("STPD:quite a few", binary=False)
                while (not dataQueue.empty()):
                    dataQueue.get(False)
            else:
                request.ws_stream.send_message("ESTP:", binary=False)
                while (not dataQueue.empty()):
                    dataQueue.get(False)

        elif instruction[:5] == "LDAT:":
            if not dataQueue.empty():
                data_out=[]
                while (not dataQueue.empty()):
                    temp=dataQueue.get()
                    if temp[:1] == "E": # error flagged
                        workerevent.set() # doesn't appear to need a sleep here
                        data_out=[]
                        data_out.append(temp)
                        while (not dataQueue.empty()):
                            temp = dataQueue.get(False)
                    else:
                        data_out.append("LIVE:" + temp)
                request.ws_stream.send_message(''.join(data_out), binary=False)
            else:
                request.ws_stream.send_message("NDAT:", binary=False)

        elif instruction[:5] == "FILE:":
            for line in [f for f in listdir("/data/samples/") if isfile(join("/data/samples/", f))]:
                request.ws_stream.send_message("FILE:" + line, binary=False)
        elif instruction[:5] == "LOAD:":
            with open('/data/samples/' + instruction[5:]) as f:
                chunk = 0
                data_out = []
                for line in f:
                    chunk += 1
                    if (line == "" or len(line.split(",")) < 3): #ignore bad line
                        chunk -= 1
                        continue
                    data_out.append("DATA:" + line)
                    if (chunk % 60) == 0:
                        request.ws_stream.send_message(''.join(data_out), binary=False)
                        data_out = []
                if (chunk % 60) != 0:
                    request.ws_stream.send_message(''.join(data_out), binary=False)
                request.ws_stream.send_message("LFIN: " + str(chunk), binary=False)
        elif instruction[:5] == "SHDN:":
            system("/usr/bin/sync && /usr/bin/shutdown -P now")
            #pass

            
# Using multiprocessing class here.  At some point the Pi 3A will become available
# and we can then really ramp up the sample rate as this could run in its own core.
# At the moment with 50 samples/sec and everything else going on here, we are
# burning about 50% CPU.
class process_sample(Process):
    def __init__ (self, filename, interval, q, workerevent):
        self.filename = filename
        self.interval = interval
        self.q = q
        self.initcount = 0
        self.datastore = []
        self.negateReadings = None
        self.stopped = workerevent
        self.imu=MPU6050(DEV_ADDR_1)
        self.kalfilter = Kalman()
        self.timestamp = 0.0
        self.lastGyro = 0.0
        if self.imu.who_am_i() != 104:  # can't detect one of the IMUs
            self.q.put("EIMU:")
        else:
# these values are determined through a separate calibration process (one time only)
            self.imu.set_x_accel_offset(-5886)
            self.imu.set_y_accel_offset(-1269)
            self.imu.set_z_accel_offset(1331)
            self.imu.set_x_gyro_offset(38)
            self.imu.set_y_gyro_offset(-35)
            self.imu.set_z_gyro_offset(15)
            
            self.imu.enable_FIFO()
            self.imu.reset_FIFO()
            
            Process.__init__ (self)
            
    def run(self):
        while not self.stopped.wait(self.interval):
            fifo = self.imu.get_FIFO_count()
            if fifo == 1024: # should never happen
                self.imu.reset_FIFO()
                print "Overflow"
                continue
            while fifo >= 12:
                data = self.imu.read_FIFO_block()
                fifo -= 12
                self.kalfilter.calculate(data[1],data[2], data[3]) 
                self.timestamp += (sample_period * 1000000) # 50 samples/sec in microseconds

                if self.initcount != 100: # ditch first two seconds to allow for stability (doesn't need that much)
                    self.initcount +=1
# To make mounting easier we don't want then user to think too much about which way the bell is going to rotate
# so we allow for two mounting positions, the difference being the measurement of the angle when the bell is at
# stand.  Tests have shown that the reported angle at stand is about 9-10 degrees from balance so if the bell
# is in that range we define it as being at stand.  The two possibilities are angle at stand between -20 and -3 degrees
# we define this as the "right" way i.e. angles as per the Kalman filter and pull at handstroke producing
# positive acceleration and postive velocity.  If at stand the bell is reporting an angle between 3 and 20
# we define this as the wrong way and have to negate angle, velocity and acceleration readings.
                    if self.initcount == 100:
                        if self.kalfilter.KalAngle < 20 and self.kalfilter.KalAngle >= 3: #angle is "wrong" way round
                            self.negateReadings=True
                            print "wrong %f" % (self.kalfilter.KalAngle)
                        elif self.kalfilter.KalAngle > -20 and self.kalfilter.KalAngle <= -3: #angle is "right" way round
                            self.negateReadings=False
                            print "right %f" % (self.kalfilter.KalAngle)
                        else:
                            print "out of range %f" % (self.kalfilter.KalAngle)
                            self.q.put("ESTD:")
                        if abs(data[3] > 0.1): # if moving more than 0.1 degrees.sec
                            self.q.put("EMOV:")
                        self.lastGyro=data[3]
                    continue
                accel = data[3]-self.lastGyro # quick and dirty diff, arbitary units
                angle = self.kalfilter.KalAngle # degrees
                rate = data[3] # degrees/sec
                self.lastGyro = rate
                if self.negateReadings:
                    angle = -1.0*angle
                    rate = -1.0*rate
                    accn = -1.0*accel
                entry = "A:{0:.3f},R:{1:.3f},C:{2:.3f}".format(angle,rate,accel)
                self.q.put(entry + "\n")
#                self.datastore.append("A:{0:.3f},R:{1:.3f},C:{2:.3f}, X:{3:.3f}, Z:{4:.3f}, TS:{5:d}, GY:{6:.3f}, GZ:{7:.3f}".format(angle,rate,accn,accel[0],accel[2],tstamp,gyro[1],gyro[2]))
                self.datastore.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},TS :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(angle,rate,accel,0.0,data[0],data[1],data[2],0.0,0.0,0.0,data[3],data[4],data[5],0.0,0.0,0.0))
                if self.q.qsize() > 500: # if nowt being pulled for 10 secs assume broken link and save off what we have
                    self.filename += "(aborted)"
                    break
        with open('/data/samples/' + self.filename, "w") as f:
            f.writelines( "%s\n" % item for item in self.datastore )

            
# Thanks to TKJ Electronics https://github.com/TKJElectronics/KalmanFilter
# and to Berry IMU https://github.com/mwilliams03/BerryIMU
class Kalman:
    def __init__(self):
        self.timeDelta= sample_period #time between samples
        self.Q_angle  =  0.01 # process noise for accelerometer
        self.Q_gyro   =  0.0003 # process noise for gyro
        self.R_angle  =  0.01  # measurement noise the bigger this value is the slower the filter will respond to changes
        self.bias = 0.0
        self.P_00 = 0.0
        self.P_01 = 0.0
        self.P_10 = 0.0
        self.P_11 = 0.0
        self.KalAngle = 0.0
        self.lastAccAngle = None
        
    def calculate(self, accGravY, accGravZ, gyroRate):
# what we want is report of 0 degrees at balance at start of handstroke then increasing to 360 degrees at balance 
# at backstroke.  Stand at handstroke should report -ve angle, stand at backstroke should report angle of > 360
# overall range is about -20 to +380.  The use of atan2 means that there is a discontinuity when that angle
# flips from +180 to -180 or vice versa.  The bell is a simple system so rather than using quaternions throughout
# we can just detect the flip and adjust accordingly
        if self.lastAccAngle is None:
            accAngle = atan2(accGravY,accGravZ)*180/3.142 # assumes accx is zero?? : roll=atan2(accy,sqrt(accx^2+accz^2))
            if accAngle < -40: # in case kalman stuff is started when the bell is down - shouldn't happen
                accAngle += 360.0
            self.KalAngle = accAngle
        else:
            accAngleDiff = atan2(accGravY,accGravZ)*180/3.142 - self.lastAccAngle
            if accAngleDiff > 300.0:
                accAngleDiff -= 360.0
            elif accAngleDiff < -300.0:
                accAngleDiff += 360.0
            accAngle = self.lastAccAngle + accAngleDiff
        self.lastAccAngle = accAngle
        self.KalAngle += self.timeDelta * (gyroRate - self.bias)

        self.P_00 +=  - self.timeDelta * (self.P_10 + self.P_01) + self.Q_angle * self.timeDelta
        self.P_01 +=  - self.timeDelta * self.P_11
        self.P_10 +=  - self.timeDelta * self.P_11
        self.P_11 +=  self.Q_gyro * self.timeDelta

        y = accAngle - self.KalAngle
        S = self.P_00 + self.R_angle
        K_0 = self.P_00 / S
        K_1 = self.P_10 / S

        self.KalAngle +=  K_0 * y;
        self.bias  +=  K_1 * y;
        self.P_00 -= K_0 * self.P_00;
        self.P_01 -= K_0 * self.P_01;
        self.P_10 -= K_1 * self.P_00;
        self.P_11 -= K_1 * self.P_01;
        
    def setAngle(self, accGravY, accGravZ):
        self.KalAngle = atan2(accGravY,accGraxZ)*180/3.142



class MPU6050:
    def __init__(self,address):
        self.dev_addr = address
        self.accn_scale = None
        self.gyro_scale = None
        self.FIFO_block_length = 12

    # reset device
        self.write_bit(C.MPU6050_RA_PWR_MGMT_1,C.MPU6050_PWR1_DEVICE_RESET_BIT, 1)
        sleep(0.5)
    #wakeup
    #possibly some undocumented behaviour here - gyro config register does
    #not accept write of FS factor unless device woken up first
        self.write_bit(C.MPU6050_RA_PWR_MGMT_1, C.MPU6050_PWR1_SLEEP_BIT, 0)
        
    #set clock source
    #because we have two accelerometers with slightly different clocks (and no way of syncing
    #them with the GY-521 breakouts we are using), we have to select which pll gives the closest
    #sync on a case by case basis.  Should only be a one time calibration.  The FIFO read code
    #above takes account of a lack of sync by ditching fifo packets from the faster running device
    #but wise to keep things as far in sync as we can.
        t1 = i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_PWR_MGMT_1) & 0xFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_PWR_MGMT_1, (t1 & 0xF8) | C.MPU6050_CLOCK_PLL_ZGYRO)

    # set full scale acceleration range
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ACCEL_CONFIG, C.MPU6050_ACCEL_FS_2 << 3)
        self.accn_scale = C.MPU6050_ACCEL_SCALE_MODIFIER_2G

    # set full scale gyro range
        i2c.write_byte_data(self.dev_addr,C.MPU6050_RA_GYRO_CONFIG, C.MPU6050_GYRO_FS_500 << 3)
        self.gyro_scale = C.MPU6050_GYRO_SCALE_MODIFIER_500DEG
                
    def who_am_i(self):
        return i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_WHO_AM_I) & 0xFF
 
    def write_bit(self,a_reg_add, a_bit_num, a_bit):
        byte = i2c.read_byte_data(self.dev_addr, a_reg_add)
        if a_bit:
            byte |= 1 << a_bit_num
        else:
            byte &= ~(1 << a_bit_num)
        i2c.write_byte_data(self.dev_addr, a_reg_add, byte)

    def set_x_accel_offset(self,a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XA_OFFS_H,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XA_OFFS_L_TC,(a_offset & 0xFF))

    def set_y_accel_offset(self,a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YA_OFFS_H,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YA_OFFS_L_TC,(a_offset & 0xFF))

    def set_z_accel_offset(self,a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZA_OFFS_H,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZA_OFFS_L_TC,(a_offset & 0xFF))

    def set_x_gyro_offset(self, a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XG_OFFS_USRH,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XG_OFFS_USRL,(a_offset & 0xFF))

    def set_y_gyro_offset(self, a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YG_OFFS_USRH,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YG_OFFS_USRL,(a_offset & 0xFF))

    def set_z_gyro_offset(self, a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZG_OFFS_USRH,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZG_OFFS_USRL,(a_offset & 0xFF))

    def read_word_signed(self, register):
        high = i2c.read_byte_data(self.dev_addr, register)
        res = (high << 8) + i2c.read_byte_data(self.dev_addr, register + 1)
        if (res >= 0x8000):
            return res - 0x10000
        return res

    def write_word_signed(self,register,value):
        value = value & 0xFFFF
        i2c.write_byte_data(self.dev_addr, register,(value >> 8))
        i2c.write_byte_data(self.dev_addr, register + 1,(value & 0xFF))
        
    def get_acceleration(self):
        result = []
        readbytes = i2c.read_i2c_block_data(self.dev_addr, C.MPU6050_RA_ACCEL_XOUT_H, 6)
        result.append(readbytes[0] << 8 | readbytes[1])
        result.append(readbytes[2] << 8 | readbytes[3])
        result.append(readbytes[4] << 8 | readbytes[5])
        for index in range(3):
            if (result[index] >= 0x8000):
                result[index] -= 0x10000
        return result

    def get_rotation(self):
        result = []
        readbytes = i2c.read_i2c_block_data(self.dev_addr, C.MPU6050_RA_GYRO_XOUT_H, 6)
        result.append(readbytes[0] << 8 | readbytes[1])
        result.append(readbytes[2] << 8 | readbytes[3])
        result.append(readbytes[4] << 8 | readbytes[5])
        for index in range(3):
            if (result[index] >= 0x8000):
                result[index] -= 0x10000
        return result
        
    def reset_FIFO(self):
        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_EN_BIT, 0)
        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_RESET_BIT, 1)
        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_EN_BIT, 1)

    def get_int_status(self):
        return i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_INT_STATUS)
        
    def get_FIFO_count(self):
        high = i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_COUNTH)
        return (high << 8) + i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_COUNTL)

    def read_FIFO_block(self):
        result = [] # accn x,y,z then gyro x,y,z
        readbytes=[]
        for index in range(self.FIFO_block_length):
            readbytes.append(i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_R_W))
        for index in range(0, self.FIFO_block_length,2):
            result.append((readbytes[index] << 8) + readbytes[index+1])
            if (result[len(result)-1] >= 0x8000):
                result[len(result)-1] -= 0x10000
        result[0] /= float(self.accn_scale) # results in g
        result[1] /= float(self.accn_scale)
        result[2] /= float(self.accn_scale)
        result[3] /= float(self.gyro_scale) # result degrees/sec
        result[4] /= float(self.gyro_scale)
        result[5] /= float(self.gyro_scale)
        return result
        
    def enable_FIFO(self):
        # set LPF to 188Hz
        
        t1 = i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_CONFIG) & 0xFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_CONFIG, (t1 & 0xF8) | C.MPU6050_DLPF_BW_188) # 5 Hz LPF

        #set gyroscope output rate divider
        # * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
        # * Motion detection, and Free Fall detection are all based on the Sample Rate.
        # * The Sample Rate is generated by dividing the gyroscope output rate by
        # * SMPLRT_DIV:
        # *
        # * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        # *
        # * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
        # * 7), and 1kHz when the DLPF is enabled (see Register 26).
        # *
        # * Note: The accelerometer output rate is 1kHz. This means that for a Sample
        # * Rate greater than 1kHz, the same accelerometer sample may be output to the
        # * FIFO, DMP, and sensor registers more than once.
        # *
        # * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
        # * of the MPU-6000/MPU-6050 Product Specification document

        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_SMPLRT_DIV, 19) # takes us to 50hz
        
        # enable fifo, select accel and gyro to go in
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_EN, 0x78) # enable temp = 0xF8

        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_EN_BIT, 1)            
            
