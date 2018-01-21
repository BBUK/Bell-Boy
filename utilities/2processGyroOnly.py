#!/usr/bin/python2

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
# hardware is currently a Pi Zero running Arch Linux, two MPU6050 breakout boards and the official Pi Wifi dongle
# The MPU6050 breakout boards are mounted on opposite sides of the bell axle such that gravity pulls
# on the MPU6050s in the same direction but so that the tangential acceleration applied by the ringer 
# works in opposite directions on each sensor.  This makes it easy to separate out tangential acceleration 
# due to gravity and tangential acceleration due to force applied.

# This script is used to test calibration methods.  The two accelerometers may not be placed exactly equidistant
# from the axle of the bell.

from time import time,sleep,strftime
from math import atan, atan2, sqrt, sin
import sys
import os.path
import time
import math
import csv
import numpy
import peakutils
import matplotlib.pyplot as plt

import dcmimu

sample_period = 0.002507 # this is the current sample period (50 samples/sec].
#sample_period = 0.005 # this is the current sample period (50 samples/sec].
NL = 0.035

def main():
    kalfilter = EKF()
    angles=[]
    rates=[]
    accns=[]
    corraccns=[]
    AX1=[]
    AY1=[]
    AZ1=[]
    GX1=[]
    GY1=[]
    GZ1=[]
    output=[]
    lastcalcAngle = -10
    flipAngleCorrection = 0.0
    calcAngle = None
    with open('input', 'rb') as f:
        reader = csv.reader(f)
        raw_data = list(reader)
    count = 0
    avangle = 0
    avrate = 0
    avaccn = 0
    kalaccn = 0.0
    ax1 = 0
    az1= 0
    gy1 =0
    for row in raw_data:
#        ts = float(row[3][4:])
        ax1 = -float(row[3][3:])
#        ax1 = 0.80* ax1 - 0.20* float(row[3][3:])

        ay1 = float(row[4][3:])
        az1 = -float(row[5][3:])
#        az1 = 0.80* az1 - 0.20* float(row[3][3:])

        gx1 = float(row[6][3:])
        gy1 = - (float(row[7][3:]))
#        gy1 = 0.80* gy1 - 0.20*(float(row[7][3:]) + 0.6)

        gz1 = float(row[8][3:])

#        if gx1 > 0:
#            gx1 -= gx1 * NL
#        else:
#            gx1 -= gx1 * NL
       
        if count < 50: # get a rough angle estimate for the first 50 samples (bell should be stationary)
            lastcalcAngle = 0.9*lastcalcAngle + 0.1*(atan2(ax1,az1)*180/3.142)
#            kalfilter.calculate(lastAccAngle,0.0)
#                calcAngle = 0.95* calcAngle + 0.05 * lastAccAngle
            count +=1
            lastgx1 = gy1
            continue
        accTang = (gy1-lastgx1)/sample_period
        adjaz1 = az1 - ((gy1*gy1*0.152)/32203)
        adjax1 = ax1 - (accTang * 0.152)/562
        accAngle = (atan2(adjax1,adjaz1)*180/3.142)
#        accAngle += flipAngleCorrection
#        if calcAngle > 180.0:
#            accAngle += 360.0
#        if (lastAccAngle - accAngle) > 250 and flipAngleCorrection != 360 and kalfilter.KalAngle > 120 and kalfilter.KalAngle < 240: #positive to negative flip
#            accAngle -= flipAngleCorrection
#            flipAngleCorrection = 360
 #           accAngle += 360
#            output.append("flipUp")
#        elif (lastAccAngle - accAngle) < -250 and flipAngleCorrection != 0 and kalfilter.KalAngle > 120 and kalfilter.KalAngle < 240: # negative to positive flip
#            accAngle -= 360
#            flipAngleCorrection = 0
#            accAngle += flipAngleCorrection
#            output.append("flipDown")


#        if calcAngle > 180.0:
#            accAngle += 360.0
 #       if (lastAccAngle - accAngle) > 120 : #positive to negative flip
#            accAngle -= flipAngleCorrection
#            flipAngleCorrection += 360
#            accAngle += flipAngleCorrection
#            output.append("flipUp")
#        elif (lastAccAngle - accAngle) < -120: # negative to positive flip
#            accAngle -= flipAngleCorrection
#            flipAngleCorrection -= 360
#            accAngle += flipAngleCorrection
#            output.append("flipDown")

        agyro = lastgx1 + (gy1-lastgx1)*0.5
        kalfilter.calculate(math.radians(gy1),math.radians(gx1),math.radians(gz1),-ay1*9.81,ax1*9.81,az1*9.81,sample_period)
        calcAngle = math.degrees(kalfilter.roll) + flipAngleCorrection
#        if calcAngle > 180.0:
#            accAngle += 360.0
        if (lastcalcAngle - calcAngle) > 250 and flipAngleCorrection != 360: #positive to negative flip
            flipAngleCorrection = 360
            calcAngle += 360
        elif (lastcalcAngle - calcAngle) < -250 and flipAngleCorrection != 0: # negative to positive flip
            calcAngle -= 360
            flipAngleCorrection = 0
#            accAngle += flipAngleCorrection
#            output.append("flipDown")
        lastcalcAngle = calcAngle

        lastgx1 = gy1

#        if abs(gy1) < 90.0:
#            calcAngle = 0.98*(calcAngle + (sample_period * agyro)) + 0.02 * accAngle
#        else:
#            calcAngle = calcAngle + (sample_period * agyro)
#        if abs(kalfilter.KalAngle - 180.0) < 20.0:
#            if gy1 > 0.0:
#                if kalfilter.KalAngle > 180.0 and accTang > 0.0:
#                    kalfilter.KalAngle = 180.0
#                elif kalfilter.KalAngle < 180.0 and accTang < 0.0:
#                    kalfilter.KalAngle = 180.0
#            else:
#                if kalfilter.KalAngle < 180.0 and accTang < 0.0:
#                    kalfilter.KalAngle = 180.0
#                elif kalfilter.KalAngle > 180.0 and accTang > 0.0:
#                    kalfilter.KalAngle = 180.0
        angles.append(calcAngle)
        rates.append(agyro)
        avaccn = 0.90*avaccn + 0.10*(accTang - 930*sin(math.radians(calcAngle)))
#        avaccn = accTang #0.90*avaccn + 0.10*accTang # math.degrees(kalfilter.a[0]) #accTang -1000*sin(kalfilter.KalAngle*3.1416/180)
        kalaccn = 0.9 * kalaccn + 0.10* kalfilter.a[0] #0.90* kalaccn + 0.10*(kalfilter.a[0] * 57) 
        accns.append(avaccn)
        corraccns.append(avaccn-(kalaccn/0.1))

        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}, ATN:{9:.3f}".format(calcAngle,agyro,avaccn,ax1,ay1,az1,gx1,gy1,gz1, accAngle))
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}, ATN:{9:.3f}".format(calcAngle,agyro,accTang-1000*sin(calcAngle*3.1416/180),ax1,ay1,az1,gx1,gy1,gz1, accAngle))

#        accGravY = (ay1 + ay2*radius_factor)/2.0 # for the Y axis gravity is pulling in the same direction on the sensor, centripetal acceleration in opposite direction so add to get gravity
#        accGravZ = (az1 + az2*radius_factor)/2.0 # for the Z axis gravity is pulling in the same direction on the sensor, tangential acceleration in opposite direction so add two readings to get gravity only.
#        accTang = (az1 - az2*radius_factor)/2.0 # this is the tangental acceleration signal we want to measure
#        avgGyro = (gx1 + gx2)/2.0 # may as well average the two X gyro readings
#        kalfilter.calculate(accGravY,accGravZ, avgGyro) 
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},OA :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(kalfilter.KalAngle,avgGyro,accTang*8192.0,0,ax1,ay1,az1,0,0,0,gx1,gy1,gz1,0,0,0))
#    INPOL=numpy.interp(range(-10,371),angles,accns,period=380)
#    peaks = peakutils.peak.indexes(INPOL,thres=0.90,min_dist=300)
#    peak = INPOL[peaks[0]]
#    adjacn1=[]
#    adjang1=[]
#    adjacn2=[]
#    adjang2=[]
#    for av in range(len(accns)):
#        if rates[av] > 0.0 and angles[av] < 180.0:
#            adjacn1.append(accns[av]-peak*sin(angles[av]*3.1416/180.0))
#            adjang1.append(angles[av])
#            adjacn1.append(-accns[av]-peak*sin((360.0-angles[av])*3.1416/180.0))
#            adjang1.append(360.0-angles[av])
#        if rates[av] < 0.0 and angles[av] > 180.0:
#            adjacn2.append(accns[av]-peak*sin(angles[av]*3.1416/180.0))
#            adjang2.append(angles[av])
#            adjacn2.append(-accns[av]-peak*sin((360.0-angles[av])*3.1416/180.0))
#            adjang2.append(360.0-angles[av])
##    INPOL1=numpy.interp(range(-10,371),adjang1,adjacn1,period=380)
##    BASEL1=peakutils.baseline(INPOL1,deg=7)
 ##   INPOL2=numpy.interp(range(-10,371),adjang2,adjacn2,period=380)
##    BASEL2=peakutils.baseline(INPOL2,deg=7)
#    adjacn=[]
#    for av in range(len(accns)):
#        gravminus = accns[av]-peak*sin(angles[av]*3.1416/180.0)
#        adjacn.append(gravminus)
#        if rates[av] >= 0.0:
#            adjacn.append(gravminus-BASEL1[int(angles[av]+10)])
#        else:
#            adjacn.append(gravminus-BASEL2[int(angles[av]+10)])
#    plt.plot(range(-10,371),INPOL,"ro")
    plt.plot(angles,corraccns,"r")
    plt.plot(angles,accns,"y")

#    plt.plot(angles,adjacn,"b")
#    plt.plot(range(-10,371),BASEL1,"g")
#    plt.plot(range(-10,371),BASEL2,"y")
    plt.plot(range(-10,371),[0]*381,"r")

    with open('outputKal', 'wb') as f:
        writer = csv.writer(f)
        for row in range(len(output)):
            writer.writerow(output[row].split(','))

    plt.show()     
        
# Thanks to TKJ Electronics https://github.com/TKJElectronics/KalmanFilter
# and to Berry IMU https://github.com/mwilliams03/BerryIMU
class Kalman:
    def __init__(self):
        self.timeDelta= sample_period #time between samples
        self.Q_angle  =  0.001 # process noise for accelerometer
        self.Q_gyro   =  0.003 # process noise for gyro
        self.R_angle  =  0.1  # measurement noise the bigger this value is the slower the filter will respond to changes
        self.bias = 0.0
        self.P_00 = 0.0
        self.P_01 = 0.0
        self.P_10 = 0.0
        self.P_11 = 0.0
        self.KalAngle = 0.0
        self.lastAccAngle = None
        
    def calculate(self, accAngle, gyroRate):
# what we want is report of 0 degrees at balance at start of handstroke then increasing to 360 degrees at balance 
# at backstroke.  Stand at handstroke should report -ve angle, stand at backstroke should report angle of > 360
# overall range is about -20 to +380.  The use of atan2 means that there is a discontinuity when that angle
# flips from +180 to -180 or vice versa.  The bell is a simple system so rather than using quaternions throughout
# we can just detect the flip and adjust accordingly
        self.KalAngle += self.timeDelta * (gyroRate - self.bias)

        self.P_00 +=  - self.timeDelta * (self.P_10 + self.P_01) + self.Q_angle * self.timeDelta
        self.P_01 +=  - self.timeDelta * self.P_11
        self.P_10 +=  - self.timeDelta * self.P_11
        self.P_11 +=  self.Q_gyro * self.timeDelta

# this is an attempt to make the KF more responsive to accelerometer readings when the bell is moving more
# slowly.  The idea being that gyro readings should be relied upon instead.
#        if abs(gyroRate) > 200.0:
#            self.R_angle = 100
#        elif abs(gyroRate) > 100.0:
#            self.R_angle = 50
#        elif abs(gyroRate) > 50.0:
#            self.R_angle = 25
#        else:
#            self.R_angle = 0.01
#        self.R_angle = abs(gyroRate)/2
#        if self.R_angle < 0.01:
#            self.R_angle = 0.01
        
        
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


class Kalman2:
    def __init__(self):
        self.timeDelta= sample_period #time between samples
        self.Q_angle  =  0.01 # process noise for accelerometer
        self.Q_gyro   =  0.0003 # process noise for gyro
        self.R_angle  =  0.001  # measurement noise the bigger this value is the slower the filter will respond to changes

        self.qbias = 0.0
        self.P_00 = 1.0
        self.P_01 = 0.0
        self.P_10 = 0.0
        self.P_11 = 1.0
        self.KalAngle = 0.0
        
    def calculate(self, accAngle, gyroRate):
        self.KalAngle += self.timeDelta * (gyroRate - self.qbias)
        self.P_00 +=  - self.timeDelta * (self.P_10 + self.P_01) + self.Q_angle * self.timeDelta
        self.P_01 +=  - self.timeDelta * self.P_11
        self.P_10 +=  - self.timeDelta * self.P_11
        self.P_11 +=  self.Q_gyro * self.timeDelta

        y = accAngle - self.KalAngle
        S = self.R_angle + self.P_00
        K_0 = self.P_00 / S
        K_1 = self.P_10 / S	 	

        self.P_00 -= K_0 * self.P_00
        self.P_01 -= K_0 * self.P_01
        self.P_10 -= K_1 * self.P_00
        self.P_11 -= K_1 * self.P_01
        self.KalAngle	+= K_0 * y
        self.qbias	+= K_1 * y
#https://github.com/hhyyti/dcm-imu
class EKF:
    def __init__(self):
    
        self.x = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.P = [  [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.a = [0.0, 0.0, 0.0] # non-gravitational accn
                    
        #define DEFAULT_g0 9.8189
#define DEFAULT_g0 9.8189
#define DEFAULT_state {0,0,1,0,0,0}
#define DEFAULT_q_dcm2 (0.1*0.1)
#define DEFAULT_q_gyro_bias2 (0.0001*0.0001)
#define DEFAULT_r_acc2 (0.5*0.5)
#define DEFAULT_r_a2 (10*10)
#define DEFAULT_q_dcm2_init (1*1)
#define DEFAULT_q_gyro_bias2_init (0.1*0.1)

#DCM_IMU_uC::DCM_IMU_uC(
#const float Gravity, g0(Gravity) const float Gravity = DEFAULT_g0
#const float *State, const float *State = NULL
#const float *Covariance, const float *Covariance = NULL
#const float DCMVariance, q_dcm2(DCMVariance) DCMVariance = DEFAULT_q_dcm2
#const float BiasVariance, q_gyro_bias2(BiasVariance) const float BiasVariance = DEFAULT_q_gyro_bias2
#const float InitialDCMVariance, InitialDCMVariance = DEFAULT_q_dcm2_init
#const float InitialBiasVariance, const float InitialBiasVariance = DEFAULT_q_gyro_bias2_init
#const float MeasurementVariance, r_acc2(MeasurementVariance) MeasurementVariance = DEFAULT_r_acc2
#const float MeasurementVarianceVariableGain) r_a2(MeasurementVarianceVariableGain): const float MeasurementVarianceVariableGain = DEFAULT_r_a2

    def calculate(self,u0,u1,u2,z0,z1,z2,h): #gyro x,y,z accel x,y,z, timestep
        g0 = 9.8189
        g0_2 = g0*g0
        q_dcm2 = 0.1*0.1
        q_gyro_bias2 = 0.0001*0.0001
        r_acc2 = 0.5*0.5
        r_a2 = 10*10
        q_dcm2_init = 1*1
        q_gyro_bias2_init = 0.1*0.1

        x_last = [self.x[0], self.x[1], self.x[2]]
        x_0 = self.x[0]-h*(u1*self.x[2]-u2*self.x[1]+self.x[1]*self.x[5]-self.x[2]*self.x[4])
        x_1 = self.x[1]+h*(u0*self.x[2]-u2*self.x[0]+self.x[0]*self.x[5]-self.x[2]*self.x[3])
        x_2 = self.x[2]-h*(u0*self.x[1]-u1*self.x[0]+self.x[0]*self.x[4]-self.x[1]*self.x[3])
        x_3 = self.x[3]
        x_4 = self.x[4]
        x_5 = self.x[5]

	# covariance prediction
        hh = h*h
        P_00 = self.P[0][0]-h*(self.P[0][5]*self.x[1]-self.P[0][4]*self.x[2]-self.P[4][0]*self.x[2]+self.P[5][0]*self.x[1]+self.P[0][2]*(u1-self.x[4])+self.P[2][0]*(u1-self.x[4])-self.P[0][1]*(u2-self.x[5])-self.P[1][0]*(u2-self.x[5]))+hh*(q_dcm2-self.x[1]*(self.P[4][5]*self.x[2]-self.P[5][5]*self.x[1]-self.P[2][5]*(u1-self.x[4])+self.P[1][5]*(u2-self.x[5]))+self.x[2]*(self.P[4][4]*self.x[2]-self.P[5][4]*self.x[1]-self.P[2][4]*(u1-self.x[4])+self.P[1][4]*(u2-self.x[5]))-(u1-self.x[4])*(self.P[4][2]*self.x[2]-self.P[5][2]*self.x[1]-self.P[2][2]*(u1-self.x[4])+self.P[1][2]*(u2-self.x[5]))+(u2-self.x[5])*(self.P[4][1]*self.x[2]-self.P[5][1]*self.x[1]-self.P[2][1]*(u1-self.x[4])+self.P[1][1]*(u2-self.x[5])))
        P_01 = self.P[0][1]+h*(self.P[0][5]*self.x[0]-self.P[0][3]*self.x[2]+self.P[4][1]*self.x[2]-self.P[5][1]*self.x[1]+self.P[0][2]*(u0-self.x[3])-self.P[0][0]*(u2-self.x[5])-self.P[2][1]*(u1-self.x[4])+self.P[1][1]*(u2-self.x[5]))+hh*(self.x[0]*(self.P[4][5]*self.x[2]-self.P[5][5]*self.x[1]-self.P[2][5]*(u1-self.x[4])+self.P[1][5]*(u2-self.x[5]))-self.x[2]*(self.P[4][3]*self.x[2]-self.P[5][3]*self.x[1]-self.P[2][3]*(u1-self.x[4])+self.P[1][3]*(u2-self.x[5]))+(u0-self.x[3])*(self.P[4][2]*self.x[2]-self.P[5][2]*self.x[1]-self.P[2][2]*(u1-self.x[4])+self.P[1][2]*(u2-self.x[5]))-(u2-self.x[5])*(self.P[4][0]*self.x[2]-self.P[5][0]*self.x[1]-self.P[2][0]*(u1-self.x[4])+self.P[1][0]*(u2-self.x[5])))
        P_02 = self.P[0][2]-h*(self.P[0][4]*self.x[0]-self.P[0][3]*self.x[1]-self.P[4][2]*self.x[2]+self.P[5][2]*self.x[1]+self.P[0][1]*(u0-self.x[3])-self.P[0][0]*(u1-self.x[4])+self.P[2][2]*(u1-self.x[4])-self.P[1][2]*(u2-self.x[5]))-hh*(self.x[0]*(self.P[4][4]*self.x[2]-self.P[5][4]*self.x[1]-self.P[2][4]*(u1-self.x[4])+self.P[1][4]*(u2-self.x[5]))-self.x[1]*(self.P[4][3]*self.x[2]-self.P[5][3]*self.x[1]-self.P[2][3]*(u1-self.x[4])+self.P[1][3]*(u2-self.x[5]))+(u0-self.x[3])*(self.P[4][1]*self.x[2]-self.P[5][1]*self.x[1]-self.P[2][1]*(u1-self.x[4])+self.P[1][1]*(u2-self.x[5]))-(u1-self.x[4])*(self.P[4][0]*self.x[2]-self.P[5][0]*self.x[1]-self.P[2][0]*(u1-self.x[4])+self.P[1][0]*(u2-self.x[5])))
        P_03 = self.P[0][3]+h*(self.P[4][3]*self.x[2]-self.P[5][3]*self.x[1]-self.P[2][3]*(u1-self.x[4])+self.P[1][3]*(u2-self.x[5]))
        P_04 = self.P[0][4]+h*(self.P[4][4]*self.x[2]-self.P[5][4]*self.x[1]-self.P[2][4]*(u1-self.x[4])+self.P[1][4]*(u2-self.x[5]))
        P_05 = self.P[0][5]+h*(self.P[4][5]*self.x[2]-self.P[5][5]*self.x[1]-self.P[2][5]*(u1-self.x[4])+self.P[1][5]*(u2-self.x[5]))
        P_10 = self.P[1][0]-h*(self.P[1][5]*self.x[1]-self.P[1][4]*self.x[2]+self.P[3][0]*self.x[2]-self.P[5][0]*self.x[0]-self.P[2][0]*(u0-self.x[3])+self.P[1][2]*(u1-self.x[4])+self.P[0][0]*(u2-self.x[5])-self.P[1][1]*(u2-self.x[5]))+hh*(self.x[1]*(self.P[3][5]*self.x[2]-self.P[5][5]*self.x[0]-self.P[2][5]*(u0-self.x[3])+self.P[0][5]*(u2-self.x[5]))-self.x[2]*(self.P[3][4]*self.x[2]-self.P[5][4]*self.x[0]-self.P[2][4]*(u0-self.x[3])+self.P[0][4]*(u2-self.x[5]))+(u1-self.x[4])*(self.P[3][2]*self.x[2]-self.P[5][2]*self.x[0]-self.P[2][2]*(u0-self.x[3])+self.P[0][2]*(u2-self.x[5]))-(u2-self.x[5])*(self.P[3][1]*self.x[2]-self.P[5][1]*self.x[0]-self.P[2][1]*(u0-self.x[3])+self.P[0][1]*(u2-self.x[5])))
        P_11 = self.P[1][1]+h*(self.P[1][5]*self.x[0]-self.P[1][3]*self.x[2]-self.P[3][1]*self.x[2]+self.P[5][1]*self.x[0]+self.P[1][2]*(u0-self.x[3])+self.P[2][1]*(u0-self.x[3])-self.P[0][1]*(u2-self.x[5])-self.P[1][0]*(u2-self.x[5]))+hh*(q_dcm2-self.x[0]*(self.P[3][5]*self.x[2]-self.P[5][5]*self.x[0]-self.P[2][5]*(u0-self.x[3])+self.P[0][5]*(u2-self.x[5]))+self.x[2]*(self.P[3][3]*self.x[2]-self.P[5][3]*self.x[0]-self.P[2][3]*(u0-self.x[3])+self.P[0][3]*(u2-self.x[5]))-(u0-self.x[3])*(self.P[3][2]*self.x[2]-self.P[5][2]*self.x[0]-self.P[2][2]*(u0-self.x[3])+self.P[0][2]*(u2-self.x[5]))+(u2-self.x[5])*(self.P[3][0]*self.x[2]-self.P[5][0]*self.x[0]-self.P[2][0]*(u0-self.x[3])+self.P[0][0]*(u2-self.x[5])))
        P_12 = self.P[1][2]-h*(self.P[1][4]*self.x[0]-self.P[1][3]*self.x[1]+self.P[3][2]*self.x[2]-self.P[5][2]*self.x[0]+self.P[1][1]*(u0-self.x[3])-self.P[2][2]*(u0-self.x[3])-self.P[1][0]*(u1-self.x[4])+self.P[0][2]*(u2-self.x[5]))+hh*(self.x[0]*(self.P[3][4]*self.x[2]-self.P[5][4]*self.x[0]-self.P[2][4]*(u0-self.x[3])+self.P[0][4]*(u2-self.x[5]))-self.x[1]*(self.P[3][3]*self.x[2]-self.P[5][3]*self.x[0]-self.P[2][3]*(u0-self.x[3])+self.P[0][3]*(u2-self.x[5]))+(u0-self.x[3])*(self.P[3][1]*self.x[2]-self.P[5][1]*self.x[0]-self.P[2][1]*(u0-self.x[3])+self.P[0][1]*(u2-self.x[5]))-(u1-self.x[4])*(self.P[3][0]*self.x[2]-self.P[5][0]*self.x[0]-self.P[2][0]*(u0-self.x[3])+self.P[0][0]*(u2-self.x[5])))
        P_13 = self.P[1][3]-h*(self.P[3][3]*self.x[2]-self.P[5][3]*self.x[0]-self.P[2][3]*(u0-self.x[3])+self.P[0][3]*(u2-self.x[5]))
        P_14 = self.P[1][4]-h*(self.P[3][4]*self.x[2]-self.P[5][4]*self.x[0]-self.P[2][4]*(u0-self.x[3])+self.P[0][4]*(u2-self.x[5]))
        P_15 = self.P[1][5]-h*(self.P[3][5]*self.x[2]-self.P[5][5]*self.x[0]-self.P[2][5]*(u0-self.x[3])+self.P[0][5]*(u2-self.x[5]))
        P_20 = self.P[2][0]-h*(self.P[2][5]*self.x[1]-self.P[3][0]*self.x[1]+self.P[4][0]*self.x[0]-self.P[2][4]*self.x[2]+self.P[1][0]*(u0-self.x[3])-self.P[0][0]*(u1-self.x[4])+self.P[2][2]*(u1-self.x[4])-self.P[2][1]*(u2-self.x[5]))-hh*(self.x[1]*(self.P[3][5]*self.x[1]-self.P[4][5]*self.x[0]-self.P[1][5]*(u0-self.x[3])+self.P[0][5]*(u1-self.x[4]))-self.x[2]*(self.P[3][4]*self.x[1]-self.P[4][4]*self.x[0]-self.P[1][4]*(u0-self.x[3])+self.P[0][4]*(u1-self.x[4]))+(u1-self.x[4])*(self.P[3][2]*self.x[1]-self.P[4][2]*self.x[0]-self.P[1][2]*(u0-self.x[3])+self.P[0][2]*(u1-self.x[4]))-(u2-self.x[5])*(self.P[3][1]*self.x[1]-self.P[4][1]*self.x[0]-self.P[1][1]*(u0-self.x[3])+self.P[0][1]*(u1-self.x[4])))
        P_21 = self.P[2][1]+h*(self.P[2][5]*self.x[0]+self.P[3][1]*self.x[1]-self.P[4][1]*self.x[0]-self.P[2][3]*self.x[2]-self.P[1][1]*(u0-self.x[3])+self.P[0][1]*(u1-self.x[4])+self.P[2][2]*(u0-self.x[3])-self.P[2][0]*(u2-self.x[5]))+hh*(self.x[0]*(self.P[3][5]*self.x[1]-self.P[4][5]*self.x[0]-self.P[1][5]*(u0-self.x[3])+self.P[0][5]*(u1-self.x[4]))-self.x[2]*(self.P[3][3]*self.x[1]-self.P[4][3]*self.x[0]-self.P[1][3]*(u0-self.x[3])+self.P[0][3]*(u1-self.x[4]))+(u0-self.x[3])*(self.P[3][2]*self.x[1]-self.P[4][2]*self.x[0]-self.P[1][2]*(u0-self.x[3])+self.P[0][2]*(u1-self.x[4]))-(u2-self.x[5])*(self.P[3][0]*self.x[1]-self.P[4][0]*self.x[0]-self.P[1][0]*(u0-self.x[3])+self.P[0][0]*(u1-self.x[4])))
        P_22 = self.P[2][2]-h*(self.P[2][4]*self.x[0]-self.P[2][3]*self.x[1]-self.P[3][2]*self.x[1]+self.P[4][2]*self.x[0]+self.P[1][2]*(u0-self.x[3])+self.P[2][1]*(u0-self.x[3])-self.P[0][2]*(u1-self.x[4])-self.P[2][0]*(u1-self.x[4]))+hh*(q_dcm2-self.x[0]*(self.P[3][4]*self.x[1]-self.P[4][4]*self.x[0]-self.P[1][4]*(u0-self.x[3])+self.P[0][4]*(u1-self.x[4]))+self.x[1]*(self.P[3][3]*self.x[1]-self.P[4][3]*self.x[0]-self.P[1][3]*(u0-self.x[3])+self.P[0][3]*(u1-self.x[4]))-(u0-self.x[3])*(self.P[3][1]*self.x[1]-self.P[4][1]*self.x[0]-self.P[1][1]*(u0-self.x[3])+self.P[0][1]*(u1-self.x[4]))+(u1-self.x[4])*(self.P[3][0]*self.x[1]-self.P[4][0]*self.x[0]-self.P[1][0]*(u0-self.x[3])+self.P[0][0]*(u1-self.x[4])))
        P_23 = self.P[2][3]+h*(self.P[3][3]*self.x[1]-self.P[4][3]*self.x[0]-self.P[1][3]*(u0-self.x[3])+self.P[0][3]*(u1-self.x[4]))
        P_24 = self.P[2][4]+h*(self.P[3][4]*self.x[1]-self.P[4][4]*self.x[0]-self.P[1][4]*(u0-self.x[3])+self.P[0][4]*(u1-self.x[4]))
        P_25 = self.P[2][5]+h*(self.P[3][5]*self.x[1]-self.P[4][5]*self.x[0]-self.P[1][5]*(u0-self.x[3])+self.P[0][5]*(u1-self.x[4]))
        P_30 = self.P[3][0]-h*(self.P[3][5]*self.x[1]-self.P[3][4]*self.x[2]+self.P[3][2]*(u1-self.x[4])-self.P[3][1]*(u2-self.x[5]))
        P_31 = self.P[3][1]+h*(self.P[3][5]*self.x[0]-self.P[3][3]*self.x[2]+self.P[3][2]*(u0-self.x[3])-self.P[3][0]*(u2-self.x[5]))
        P_32 = self.P[3][2]-h*(self.P[3][4]*self.x[0]-self.P[3][3]*self.x[1]+self.P[3][1]*(u0-self.x[3])-self.P[3][0]*(u1-self.x[4]))
        P_33 = self.P[3][3]+hh*q_gyro_bias2
        P_34 = self.P[3][4]
        P_35 = self.P[3][5]
        P_40 = self.P[4][0]-h*(self.P[4][5]*self.x[1]-self.P[4][4]*self.x[2]+self.P[4][2]*(u1-self.x[4])-self.P[4][1]*(u2-self.x[5]))
        P_41 = self.P[4][1]+h*(self.P[4][5]*self.x[0]-self.P[4][3]*self.x[2]+self.P[4][2]*(u0-self.x[3])-self.P[4][0]*(u2-self.x[5]))
        P_42 = self.P[4][2]-h*(self.P[4][4]*self.x[0]-self.P[4][3]*self.x[1]+self.P[4][1]*(u0-self.x[3])-self.P[4][0]*(u1-self.x[4]))
        P_43 = self.P[4][3]
        P_44 = self.P[4][4]+hh*q_gyro_bias2
        P_45 = self.P[4][5]
        P_50 = self.P[5][0]-h*(self.P[5][5]*self.x[1]-self.P[5][4]*self.x[2]+self.P[5][2]*(u1-self.x[4])-self.P[5][1]*(u2-self.x[5]))
        P_51 = self.P[5][1]+h*(self.P[5][5]*self.x[0]-self.P[5][3]*self.x[2]+self.P[5][2]*(u0-self.x[3])-self.P[5][0]*(u2-self.x[5]))
        P_52 = self.P[5][2]-h*(self.P[5][4]*self.x[0]-self.P[5][3]*self.x[1]+self.P[5][1]*(u0-self.x[3])-self.P[5][0]*(u1-self.x[4]))
        P_53 = self.P[5][3]
        P_54 = self.P[5][4]
        P_55 = self.P[5][5]+hh*q_gyro_bias2

        #kalman innovation
        y0 = z0-g0*x_0
        y1 = z1-g0*x_1
        y2 = z2-g0*x_2

        a_len = math.sqrt(y0*y0+y1*y1+y2*y2)

        S00 = r_acc2+a_len*r_a2+P_00*g0_2
        S01 = P_01*g0_2
        S02 = P_02*g0_2
        S10 = P_10*g0_2
        S11 = r_acc2+a_len*r_a2+P_11*g0_2
        S12 = P_12*g0_2
        S20 = P_20*g0_2
        S21 = P_21*g0_2
        S22 = r_acc2+a_len*r_a2+P_22*g0_2

	# Kalman gain
        invPart = 1.0 / (S00*S11*S22-S00*S12*S21-S01*S10*S22+S01*S12*S20+S02*S10*S21-S02*S11*S20)
        K00 = (g0*(P_02*S10*S21-P_02*S11*S20-P_01*S10*S22+P_01*S12*S20+P_00*S11*S22-P_00*S12*S21))*invPart
        K01 = -(g0*(P_02*S00*S21-P_02*S01*S20-P_01*S00*S22+P_01*S02*S20+P_00*S01*S22-P_00*S02*S21))*invPart
        K02 = (g0*(P_02*S00*S11-P_02*S01*S10-P_01*S00*S12+P_01*S02*S10+P_00*S01*S12-P_00*S02*S11))*invPart
        K10 = (g0*(P_12*S10*S21-P_12*S11*S20-P_11*S10*S22+P_11*S12*S20+P_10*S11*S22-P_10*S12*S21))*invPart
        K11 = -(g0*(P_12*S00*S21-P_12*S01*S20-P_11*S00*S22+P_11*S02*S20+P_10*S01*S22-P_10*S02*S21))*invPart
        K12 = (g0*(P_12*S00*S11-P_12*S01*S10-P_11*S00*S12+P_11*S02*S10+P_10*S01*S12-P_10*S02*S11))*invPart
        K20 = (g0*(P_22*S10*S21-P_22*S11*S20-P_21*S10*S22+P_21*S12*S20+P_20*S11*S22-P_20*S12*S21))*invPart
        K21 = -(g0*(P_22*S00*S21-P_22*S01*S20-P_21*S00*S22+P_21*S02*S20+P_20*S01*S22-P_20*S02*S21))*invPart
        K22 = (g0*(P_22*S00*S11-P_22*S01*S10-P_21*S00*S12+P_21*S02*S10+P_20*S01*S12-P_20*S02*S11))*invPart
        K30 = (g0*(P_32*S10*S21-P_32*S11*S20-P_31*S10*S22+P_31*S12*S20+P_30*S11*S22-P_30*S12*S21))*invPart
        K31 = -(g0*(P_32*S00*S21-P_32*S01*S20-P_31*S00*S22+P_31*S02*S20+P_30*S01*S22-P_30*S02*S21))*invPart
        K32 = (g0*(P_32*S00*S11-P_32*S01*S10-P_31*S00*S12+P_31*S02*S10+P_30*S01*S12-P_30*S02*S11))*invPart
        K40 = (g0*(P_42*S10*S21-P_42*S11*S20-P_41*S10*S22+P_41*S12*S20+P_40*S11*S22-P_40*S12*S21))*invPart
        K41 = -(g0*(P_42*S00*S21-P_42*S01*S20-P_41*S00*S22+P_41*S02*S20+P_40*S01*S22-P_40*S02*S21))*invPart
        K42 = (g0*(P_42*S00*S11-P_42*S01*S10-P_41*S00*S12+P_41*S02*S10+P_40*S01*S12-P_40*S02*S11))*invPart
        K50 = (g0*(P_52*S10*S21-P_52*S11*S20-P_51*S10*S22+P_51*S12*S20+P_50*S11*S22-P_50*S12*S21))*invPart
        K51 = -(g0*(P_52*S00*S21-P_52*S01*S20-P_51*S00*S22+P_51*S02*S20+P_50*S01*S22-P_50*S02*S21))*invPart
        K52 = (g0*(P_52*S00*S11-P_52*S01*S10-P_51*S00*S12+P_51*S02*S10+P_50*S01*S12-P_50*S02*S11))*invPart

# update a posteriori
        self.x[0] = x_0+K00*y0+K01*y1+K02*y2
        self.x[1] = x_1+K10*y0+K11*y1+K12*y2
        self.x[2] = x_2+K20*y0+K21*y1+K22*y2
        self.x[3] = x_3+K30*y0+K31*y1+K32*y2
        self.x[4] = x_4+K40*y0+K41*y1+K42*y2
        self.x[5] = x_5+K50*y0+K51*y1+K52*y2

#update a posteriori covariance
        r_adab = (r_acc2+a_len*r_a2)
        P__00 = P_00-g0*(K00*P_00*2.0+K01*P_01+K01*P_10+K02*P_02+K02*P_20)+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+g0_2*(K00*(K00*P_00+K01*P_10+K02*P_20)+K01*(K00*P_01+K01*P_11+K02*P_21)+K02*(K00*P_02+K01*P_12+K02*P_22))
        P__01 = P_01-g0*(K00*P_01+K01*P_11+K02*P_21+K10*P_00+K11*P_01+K12*P_02)+g0_2*(K10*(K00*P_00+K01*P_10+K02*P_20)+K11*(K00*P_01+K01*P_11+K02*P_21)+K12*(K00*P_02+K01*P_12+K02*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab
        P__02 = P_02-g0*(K00*P_02+K01*P_12+K02*P_22+K20*P_00+K21*P_01+K22*P_02)+g0_2*(K20*(K00*P_00+K01*P_10+K02*P_20)+K21*(K00*P_01+K01*P_11+K02*P_21)+K22*(K00*P_02+K01*P_12+K02*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab
        P__03 = P_03-g0*(K00*P_03+K01*P_13+K02*P_23+K30*P_00+K31*P_01+K32*P_02)+g0_2*(K30*(K00*P_00+K01*P_10+K02*P_20)+K31*(K00*P_01+K01*P_11+K02*P_21)+K32*(K00*P_02+K01*P_12+K02*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab
        P__04 = P_04-g0*(K00*P_04+K01*P_14+K02*P_24+K40*P_00+K41*P_01+K42*P_02)+g0_2*(K40*(K00*P_00+K01*P_10+K02*P_20)+K41*(K00*P_01+K01*P_11+K02*P_21)+K42*(K00*P_02+K01*P_12+K02*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab
        P__05 = P_05-g0*(K00*P_05+K01*P_15+K02*P_25+K50*P_00+K51*P_01+K52*P_02)+g0_2*(K50*(K00*P_00+K01*P_10+K02*P_20)+K51*(K00*P_01+K01*P_11+K02*P_21)+K52*(K00*P_02+K01*P_12+K02*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab
        P__10 = P_10-g0*(K00*P_10+K01*P_11+K02*P_12+K10*P_00+K11*P_10+K12*P_20)+g0_2*(K00*(K10*P_00+K11*P_10+K12*P_20)+K01*(K10*P_01+K11*P_11+K12*P_21)+K02*(K10*P_02+K11*P_12+K12*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab
        P__11 = P_11-g0*(K10*P_01+K10*P_10+K11*P_11*2.0+K12*P_12+K12*P_21)+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+g0_2*(K10*(K10*P_00+K11*P_10+K12*P_20)+K11*(K10*P_01+K11*P_11+K12*P_21)+K12*(K10*P_02+K11*P_12+K12*P_22))
        P__12 = P_12-g0*(K10*P_02+K11*P_12+K12*P_22+K20*P_10+K21*P_11+K22*P_12)+g0_2*(K20*(K10*P_00+K11*P_10+K12*P_20)+K21*(K10*P_01+K11*P_11+K12*P_21)+K22*(K10*P_02+K11*P_12+K12*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab
        P__13 = P_13-g0*(K10*P_03+K11*P_13+K12*P_23+K30*P_10+K31*P_11+K32*P_12)+g0_2*(K30*(K10*P_00+K11*P_10+K12*P_20)+K31*(K10*P_01+K11*P_11+K12*P_21)+K32*(K10*P_02+K11*P_12+K12*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab
        P__14 = P_14-g0*(K10*P_04+K11*P_14+K12*P_24+K40*P_10+K41*P_11+K42*P_12)+g0_2*(K40*(K10*P_00+K11*P_10+K12*P_20)+K41*(K10*P_01+K11*P_11+K12*P_21)+K42*(K10*P_02+K11*P_12+K12*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab
        P__15 = P_15-g0*(K10*P_05+K11*P_15+K12*P_25+K50*P_10+K51*P_11+K52*P_12)+g0_2*(K50*(K10*P_00+K11*P_10+K12*P_20)+K51*(K10*P_01+K11*P_11+K12*P_21)+K52*(K10*P_02+K11*P_12+K12*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab
        P__20 = P_20-g0*(K00*P_20+K01*P_21+K02*P_22+K20*P_00+K21*P_10+K22*P_20)+g0_2*(K00*(K20*P_00+K21*P_10+K22*P_20)+K01*(K20*P_01+K21*P_11+K22*P_21)+K02*(K20*P_02+K21*P_12+K22*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab
        P__21 = P_21-g0*(K10*P_20+K11*P_21+K12*P_22+K20*P_01+K21*P_11+K22*P_21)+g0_2*(K10*(K20*P_00+K21*P_10+K22*P_20)+K11*(K20*P_01+K21*P_11+K22*P_21)+K12*(K20*P_02+K21*P_12+K22*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab
        P__22 = P_22-g0*(K20*P_02+K20*P_20+K21*P_12+K21*P_21+K22*P_22*2.0)+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+g0_2*(K20*(K20*P_00+K21*P_10+K22*P_20)+K21*(K20*P_01+K21*P_11+K22*P_21)+K22*(K20*P_02+K21*P_12+K22*P_22))
        P__23 = P_23-g0*(K20*P_03+K21*P_13+K22*P_23+K30*P_20+K31*P_21+K32*P_22)+g0_2*(K30*(K20*P_00+K21*P_10+K22*P_20)+K31*(K20*P_01+K21*P_11+K22*P_21)+K32*(K20*P_02+K21*P_12+K22*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab
        P__24 = P_24-g0*(K20*P_04+K21*P_14+K22*P_24+K40*P_20+K41*P_21+K42*P_22)+g0_2*(K40*(K20*P_00+K21*P_10+K22*P_20)+K41*(K20*P_01+K21*P_11+K22*P_21)+K42*(K20*P_02+K21*P_12+K22*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab
        P__25 = P_25-g0*(K20*P_05+K21*P_15+K22*P_25+K50*P_20+K51*P_21+K52*P_22)+g0_2*(K50*(K20*P_00+K21*P_10+K22*P_20)+K51*(K20*P_01+K21*P_11+K22*P_21)+K52*(K20*P_02+K21*P_12+K22*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab
        P__30 = P_30-g0*(K00*P_30+K01*P_31+K02*P_32+K30*P_00+K31*P_10+K32*P_20)+g0_2*(K00*(K30*P_00+K31*P_10+K32*P_20)+K01*(K30*P_01+K31*P_11+K32*P_21)+K02*(K30*P_02+K31*P_12+K32*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab
        P__31 = P_31-g0*(K10*P_30+K11*P_31+K12*P_32+K30*P_01+K31*P_11+K32*P_21)+g0_2*(K10*(K30*P_00+K31*P_10+K32*P_20)+K11*(K30*P_01+K31*P_11+K32*P_21)+K12*(K30*P_02+K31*P_12+K32*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab
        P__32 = P_32-g0*(K20*P_30+K21*P_31+K22*P_32+K30*P_02+K31*P_12+K32*P_22)+g0_2*(K20*(K30*P_00+K31*P_10+K32*P_20)+K21*(K30*P_01+K31*P_11+K32*P_21)+K22*(K30*P_02+K31*P_12+K32*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab
        P__33 = P_33-g0*(K30*P_03+K31*P_13+K30*P_30+K31*P_31+K32*P_23+K32*P_32)+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+g0_2*(K30*(K30*P_00+K31*P_10+K32*P_20)+K31*(K30*P_01+K31*P_11+K32*P_21)+K32*(K30*P_02+K31*P_12+K32*P_22))
        P__34 = P_34-g0*(K30*P_04+K31*P_14+K32*P_24+K40*P_30+K41*P_31+K42*P_32)+g0_2*(K40*(K30*P_00+K31*P_10+K32*P_20)+K41*(K30*P_01+K31*P_11+K32*P_21)+K42*(K30*P_02+K31*P_12+K32*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab
        P__35 = P_35-g0*(K30*P_05+K31*P_15+K32*P_25+K50*P_30+K51*P_31+K52*P_32)+g0_2*(K50*(K30*P_00+K31*P_10+K32*P_20)+K51*(K30*P_01+K31*P_11+K32*P_21)+K52*(K30*P_02+K31*P_12+K32*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab
        P__40 = P_40-g0*(K00*P_40+K01*P_41+K02*P_42+K40*P_00+K41*P_10+K42*P_20)+g0_2*(K00*(K40*P_00+K41*P_10+K42*P_20)+K01*(K40*P_01+K41*P_11+K42*P_21)+K02*(K40*P_02+K41*P_12+K42*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab
        P__41 = P_41-g0*(K10*P_40+K11*P_41+K12*P_42+K40*P_01+K41*P_11+K42*P_21)+g0_2*(K10*(K40*P_00+K41*P_10+K42*P_20)+K11*(K40*P_01+K41*P_11+K42*P_21)+K12*(K40*P_02+K41*P_12+K42*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab
        P__42 = P_42-g0*(K20*P_40+K21*P_41+K22*P_42+K40*P_02+K41*P_12+K42*P_22)+g0_2*(K20*(K40*P_00+K41*P_10+K42*P_20)+K21*(K40*P_01+K41*P_11+K42*P_21)+K22*(K40*P_02+K41*P_12+K42*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab
        P__43 = P_43-g0*(K30*P_40+K31*P_41+K32*P_42+K40*P_03+K41*P_13+K42*P_23)+g0_2*(K30*(K40*P_00+K41*P_10+K42*P_20)+K31*(K40*P_01+K41*P_11+K42*P_21)+K32*(K40*P_02+K41*P_12+K42*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab
        P__44 = P_44-g0*(K40*P_04+K41*P_14+K40*P_40+K42*P_24+K41*P_41+K42*P_42)+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+g0_2*(K40*(K40*P_00+K41*P_10+K42*P_20)+K41*(K40*P_01+K41*P_11+K42*P_21)+K42*(K40*P_02+K41*P_12+K42*P_22))
        P__45 = P_45-g0*(K40*P_05+K41*P_15+K42*P_25+K50*P_40+K51*P_41+K52*P_42)+g0_2*(K50*(K40*P_00+K41*P_10+K42*P_20)+K51*(K40*P_01+K41*P_11+K42*P_21)+K52*(K40*P_02+K41*P_12+K42*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab
        P__50 = P_50-g0*(K00*P_50+K01*P_51+K02*P_52+K50*P_00+K51*P_10+K52*P_20)+g0_2*(K00*(K50*P_00+K51*P_10+K52*P_20)+K01*(K50*P_01+K51*P_11+K52*P_21)+K02*(K50*P_02+K51*P_12+K52*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab
        P__51 = P_51-g0*(K10*P_50+K11*P_51+K12*P_52+K50*P_01+K51*P_11+K52*P_21)+g0_2*(K10*(K50*P_00+K51*P_10+K52*P_20)+K11*(K50*P_01+K51*P_11+K52*P_21)+K12*(K50*P_02+K51*P_12+K52*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab
        P__52 = P_52-g0*(K20*P_50+K21*P_51+K22*P_52+K50*P_02+K51*P_12+K52*P_22)+g0_2*(K20*(K50*P_00+K51*P_10+K52*P_20)+K21*(K50*P_01+K51*P_11+K52*P_21)+K22*(K50*P_02+K51*P_12+K52*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab
        P__53 = P_53-g0*(K30*P_50+K31*P_51+K32*P_52+K50*P_03+K51*P_13+K52*P_23)+g0_2*(K30*(K50*P_00+K51*P_10+K52*P_20)+K31*(K50*P_01+K51*P_11+K52*P_21)+K32*(K50*P_02+K51*P_12+K52*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab
        P__54 = P_54-g0*(K40*P_50+K41*P_51+K42*P_52+K50*P_04+K51*P_14+K52*P_24)+g0_2*(K40*(K50*P_00+K51*P_10+K52*P_20)+K41*(K50*P_01+K51*P_11+K52*P_21)+K42*(K50*P_02+K51*P_12+K52*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab
        P__55 = P_55-g0*(K50*P_05+K51*P_15+K52*P_25+K50*P_50+K51*P_51+K52*P_52)+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+g0_2*(K50*(K50*P_00+K51*P_10+K52*P_20)+K51*(K50*P_01+K51*P_11+K52*P_21)+K52*(K50*P_02+K51*P_12+K52*P_22))


        xlen = math.sqrt(self.x[0]*self.x[0]+self.x[1]*self.x[1]+self.x[2]*self.x[2])
        invlen3 = 1.0/(xlen*xlen*xlen)
        invlen32 = (invlen3*invlen3)

        x1_x2 = (self.x[1]*self.x[1]+self.x[2]*self.x[2])
        x0_x2 = (self.x[0]*self.x[0]+self.x[2]*self.x[2])
        x0_x1 = (self.x[0]*self.x[0]+self.x[1]*self.x[1])

#normalized a posteriori covariance
        self.P[0][0] = invlen32*(-x1_x2*(-P__00*x1_x2+P__10*self.x[0]*self.x[1]+P__20*self.x[0]*self.x[2])+self.x[0]*self.x[1]*(-P__01*x1_x2+P__11*self.x[0]*self.x[1]+P__21*self.x[0]*self.x[2])+self.x[0]*self.x[2]*(-P__02*x1_x2+P__12*self.x[0]*self.x[1]+P__22*self.x[0]*self.x[2]))
        self.P[0][1] = invlen32*(-x0_x2*(-P__01*x1_x2+P__11*self.x[0]*self.x[1]+P__21*self.x[0]*self.x[2])+self.x[0]*self.x[1]*(-P__00*x1_x2+P__10*self.x[0]*self.x[1]+P__20*self.x[0]*self.x[2])+self.x[1]*self.x[2]*(-P__02*x1_x2+P__12*self.x[0]*self.x[1]+P__22*self.x[0]*self.x[2]))
        self.P[0][2] = invlen32*(-x0_x1*(-P__02*x1_x2+P__12*self.x[0]*self.x[1]+P__22*self.x[0]*self.x[2])+self.x[0]*self.x[2]*(-P__00*x1_x2+P__10*self.x[0]*self.x[1]+P__20*self.x[0]*self.x[2])+self.x[1]*self.x[2]*(-P__01*x1_x2+P__11*self.x[0]*self.x[1]+P__21*self.x[0]*self.x[2]))
        self.P[0][3] = -invlen3*(-P__03*x1_x2+P__13*self.x[0]*self.x[1]+P__23*self.x[0]*self.x[2])
        self.P[0][4] = -invlen3*(-P__04*x1_x2+P__14*self.x[0]*self.x[1]+P__24*self.x[0]*self.x[2])
        self.P[0][5] = -invlen3*(-P__05*x1_x2+P__15*self.x[0]*self.x[1]+P__25*self.x[0]*self.x[2])
        self.P[1][0] = invlen32*(-x1_x2*(-P__10*x0_x2+P__00*self.x[0]*self.x[1]+P__20*self.x[1]*self.x[2])+self.x[0]*self.x[1]*(-P__11*x0_x2+P__01*self.x[0]*self.x[1]+P__21*self.x[1]*self.x[2])+self.x[0]*self.x[2]*(-P__12*x0_x2+P__02*self.x[0]*self.x[1]+P__22*self.x[1]*self.x[2]))
        self.P[1][1] = invlen32*(-x0_x2*(-P__11*x0_x2+P__01*self.x[0]*self.x[1]+P__21*self.x[1]*self.x[2])+self.x[0]*self.x[1]*(-P__10*x0_x2+P__00*self.x[0]*self.x[1]+P__20*self.x[1]*self.x[2])+self.x[1]*self.x[2]*(-P__12*x0_x2+P__02*self.x[0]*self.x[1]+P__22*self.x[1]*self.x[2]))
        self.P[1][2] = invlen32*(-x0_x1*(-P__12*x0_x2+P__02*self.x[0]*self.x[1]+P__22*self.x[1]*self.x[2])+self.x[0]*self.x[2]*(-P__10*x0_x2+P__00*self.x[0]*self.x[1]+P__20*self.x[1]*self.x[2])+self.x[1]*self.x[2]*(-P__11*x0_x2+P__01*self.x[0]*self.x[1]+P__21*self.x[1]*self.x[2]))
        self.P[1][3] = -invlen3*(-P__13*x0_x2+P__03*self.x[0]*self.x[1]+P__23*self.x[1]*self.x[2])
        self.P[1][4] = -invlen3*(-P__14*x0_x2+P__04*self.x[0]*self.x[1]+P__24*self.x[1]*self.x[2])
        self.P[1][5] = -invlen3*(-P__15*x0_x2+P__05*self.x[0]*self.x[1]+P__25*self.x[1]*self.x[2])
        self.P[2][0] = invlen32*(-x1_x2*(-P__20*x0_x1+P__00*self.x[0]*self.x[2]+P__10*self.x[1]*self.x[2])+self.x[0]*self.x[1]*(-P__21*x0_x1+P__01*self.x[0]*self.x[2]+P__11*self.x[1]*self.x[2])+self.x[0]*self.x[2]*(-P__22*x0_x1+P__02*self.x[0]*self.x[2]+P__12*self.x[1]*self.x[2]))
        self.P[2][1] = invlen32*(-x0_x2*(-P__21*x0_x1+P__01*self.x[0]*self.x[2]+P__11*self.x[1]*self.x[2])+self.x[0]*self.x[1]*(-P__20*x0_x1+P__00*self.x[0]*self.x[2]+P__10*self.x[1]*self.x[2])+self.x[1]*self.x[2]*(-P__22*x0_x1+P__02*self.x[0]*self.x[2]+P__12*self.x[1]*self.x[2]))
        self.P[2][2] = invlen32*(-x0_x1*(-P__22*x0_x1+P__02*self.x[0]*self.x[2]+P__12*self.x[1]*self.x[2])+self.x[0]*self.x[2]*(-P__20*x0_x1+P__00*self.x[0]*self.x[2]+P__10*self.x[1]*self.x[2])+self.x[1]*self.x[2]*(-P__21*x0_x1+P__01*self.x[0]*self.x[2]+P__11*self.x[1]*self.x[2]))
        self.P[2][3] = -invlen3*(-P__23*x0_x1+P__03*self.x[0]*self.x[2]+P__13*self.x[1]*self.x[2])
        self.P[2][4] = -invlen3*(-P__24*x0_x1+P__04*self.x[0]*self.x[2]+P__14*self.x[1]*self.x[2])
        self.P[2][5] = -invlen3*(-P__25*x0_x1+P__05*self.x[0]*self.x[2]+P__15*self.x[1]*self.x[2])
        self.P[3][0] = -invlen3*(-P__30*x1_x2+P__31*self.x[0]*self.x[1]+P__32*self.x[0]*self.x[2])
        self.P[3][1] = -invlen3*(-P__31*x0_x2+P__30*self.x[0]*self.x[1]+P__32*self.x[1]*self.x[2])
        self.P[3][2] = -invlen3*(-P__32*x0_x1+P__30*self.x[0]*self.x[2]+P__31*self.x[1]*self.x[2])
        self.P[3][3] = P__33
        self.P[3][4] = P__34
        self.P[3][5] = P__35
        self.P[4][0] = -invlen3*(-P__40*x1_x2+P__41*self.x[0]*self.x[1]+P__42*self.x[0]*self.x[2])
        self.P[4][1] = -invlen3*(-P__41*x0_x2+P__40*self.x[0]*self.x[1]+P__42*self.x[1]*self.x[2])
        self.P[4][2] = -invlen3*(-P__42*x0_x1+P__40*self.x[0]*self.x[2]+P__41*self.x[1]*self.x[2])
        self.P[4][3] = P__43
        self.P[4][4] = P__44
        self.P[4][5] = P__45
        self.P[5][0] = -invlen3*(-P__50*x1_x2+P__51*self.x[0]*self.x[1]+P__52*self.x[0]*self.x[2])
        self.P[5][1] = -invlen3*(-P__51*x0_x2+P__50*self.x[0]*self.x[1]+P__52*self.x[1]*self.x[2])
        self.P[5][2] = -invlen3*(-P__52*x0_x1+P__50*self.x[0]*self.x[2]+P__51*self.x[1]*self.x[2])
        self.P[5][3] = P__53
        self.P[5][4] = P__54
        self.P[5][5] = P__55
# normalized a posteriori state
        self.x[0] = self.x[0]/xlen
        self.x[1] = self.x[1]/xlen
        self.x[2] = self.x[2]/xlen

# compute Euler angles (not exactly a part of the extended Kalman filter)
# yaw integration through full rotation matrix
        u_nb1 = u1 - self.x[4]
        u_nb2 = u2 - self.x[5]

        cy = math.cos(self.yaw) #old angles (last state before integration)
        sy = math.sin(self.yaw)
        d = math.sqrt(x_last[1]*x_last[1] + x_last[2]*x_last[2])
        d_inv = 1.0 / d

#compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
        R11 = cy * d
        R12 = -(x_last[2]*sy + x_last[0]*x_last[1]*cy) * d_inv
        R13 = (x_last[1]*sy - x_last[0]*x_last[2]*cy) * d_inv
        R21 = sy * d
        R22 = (x_last[2]*cy - x_last[0]*x_last[1]*sy) * d_inv
        R23 = -(x_last[1]*cy + x_last[0]*x_last[2]*sy) * d_inv

# update needed parts of R for yaw computation
        R11_new = R11 + h*(u_nb2*R12 - u_nb1*R13)
        R21_new = R21 + h*(u_nb2*R22 - u_nb1*R23)
        self.yaw = math.atan2(R21_new,R11_new)

#compute new pitch and roll angles from a posteriori states
        self.pitch = math.asin(-self.x[0])
        self.roll = math.atan2(self.x[1],self.x[2])

# save the estimated non-gravitational acceleration
        self.a[0] = z0-self.x[0]*g0
        self.a[1] = z1-self.x[1]*g0
        self.a[2] = z2-self.x[2]*g0
     


if __name__ == '__main__':
    main()        
