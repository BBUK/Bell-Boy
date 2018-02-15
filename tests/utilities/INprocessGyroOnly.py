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

sample_period = 0.002507 # this is the current sample period (50 samples/sec].
#sample_period = 0.005 # this is the current sample period (50 samples/sec].
NL = 0.035


def main():
    kalfilter = Kalman()
    angles=[]
    rates=[]
    accns=[]
    AX1=[]
    AY1=[]
    AZ1=[]
    GX1=[]
    GY1=[]
    GZ1=[]
    ax1 = 0
    az1 = 0
    gy1 = 0

    output=[]
    lastAccAngle = None
    flipAngleCorrection = 0.0
    calcAngle = None
    with open('input', 'rb') as f:
        reader = csv.reader(f)
        raw_data = list(reader)
    count = 0
    avangle = 0
    avrate = 0
    avaccn = 0
    for row in raw_data:
#        ts = float(row[3][4:])
#        ax1 = 0.80* ax1 - 0.20* float(row[3][3:])
        ax1 = - float(row[3][3:])

        ay1 = float(row[4][3:])
#        az1 = 0.80* az1 - 0.20* float(row[5][3:])
        az1 = - float(row[5][3:])
        gx1 = float(row[6][3:])
        gy1 = - float(row[7][3:])
 
#        gy1 = 0.80* gy1 - 0.20* (float(row[7][3:]))
        gz1 = float(row[8][3:])

#        if gx1 > 0:
#            gx1 -= gx1 * NL
#        else:
#            gx1 -= gx1 * NL
       
        if count < 50: # get a rough angle estimate for the first 50 samples (bell should be stationary)
            lastAccAngle = (atan2(ax1,az1)*180/3.142)
            if calcAngle is None:
                calcAngle = lastAccAngle
            else:
                calcAngle = 0.97* calcAngle + 0.03 * lastAccAngle
            count +=1
            lastgx1 = gy1
            continue

        accTang = (gy1 - lastgx1)/sample_period
        adjaz1 = az1 - ((gy1*gy1*0.152)/32203)
        adjax1 = ax1 - (accTang * 0.152)/562
        accAngle = (atan2(adjax1,adjaz1)*180/3.142)
        accAngle += flipAngleCorrection
#        if calcAngle > 180.0:
#            accAngle += 360.0
        if (lastAccAngle - accAngle) > 250 and flipAngleCorrection != 360 and calcAngle > 120 and calcAngle < 240: #positive to negative flip
#            accAngle -= flipAngleCorrection
            flipAngleCorrection = 360
            accAngle += 360
#            output.append("flipUp")
        elif (lastAccAngle - accAngle) < -250 and flipAngleCorrection != 0 and calcAngle > 120 and calcAngle < 240: # negative to positive flip
            accAngle -= 360
            flipAngleCorrection = 0
#            accAngle += flipAngleCorrection
#            output.append("flipDown")

        lastAccAngle = accAngle
        agyro = lastgx1 + (gy1-lastgx1)*0.5
        lastgx1 = gy1
        avaccn = 0.95 * avaccn + 0.05* accTang
        avrate = 0.95 * avrate + 0.05* agyro
 
        if abs(gy1) < 200.0:
            calcAngle = 0.90*(calcAngle + (sample_period * agyro)) + 0.10 * accAngle
        else:
            calcAngle = calcAngle + (sample_period * agyro)
            if abs(180.0-calcAngle) < 20.0:
                if avrate > 0.0:
                    if calcAngle >= 180.0 and avaccn > 0.0:
                        calcAngle = 180.0
                    elif calcAngle <= 180.0 and avaccn < 0.0:
                        calcAngle = 180.0
                else:
                    if calcAngle <= 180.0 and avaccn < 0.0:
                        calcAngle = 180.0
                    elif calcAngle >= 180.0 and avaccn > 0.0:
                        calcAngle = 180.0

        avangle = 0.95 * avangle + 0.05* calcAngle
       
        angles.append(avangle)
        rates.append(avrate)
        accns.append(avaccn) # -1000*sin(calcAngle*3.1416/180))

        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}, ATN:{9:.3f}".format(avangle,avrate,avaccn,ax1,ay1,az1,gx1,gy1,gz1, accAngle))
#            avangle = 0
#            avrate = 0
#            avaccn = 0
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}, ATN:{9:.3f}".format(calcAngle,agyro,accTang-1000*sin(calcAngle*3.1416/180),ax1,ay1,az1,gx1,gy1,gz1, accAngle))

#        accGravY = (ay1 + ay2*radius_factor)/2.0 # for the Y axis gravity is pulling in the same direction on the sensor, centripetal acceleration in opposite direction so add to get gravity
#        accGravZ = (az1 + az2*radius_factor)/2.0 # for the Z axis gravity is pulling in the same direction on the sensor, tangential acceleration in opposite direction so add two readings to get gravity only.
#        accTang = (az1 - az2*radius_factor)/2.0 # this is the tangental acceleration signal we want to measure
#        avgGyro = (gx1 + gx2)/2.0 # may as well average the two X gyro readings
#        kalfilter.calculate(accGravY,accGravZ, avgGyro) 
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},OA :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(kalfilter.KalAngle,avgGyro,accTang*8192.0,0,ax1,ay1,az1,0,0,0,gx1,gy1,gz1,0,0,0))
    INPOL=numpy.interp(range(-10,371),angles,accns,period=380)
    peaks = peakutils.peak.indexes(INPOL,thres=0.90,min_dist=300)
    peak = INPOL[peaks[0]]
    adjacn1=[]
    adjang1=[]
    adjacn2=[]
    adjang2=[]
    for av in range(len(accns)):
        if rates[av] > 0.0 and angles[av] < 180.0:
            adjacn1.append(accns[av]-peak*sin(angles[av]*3.1416/180.0))
            adjang1.append(angles[av])
            adjacn1.append(-accns[av]-peak*sin((360.0-angles[av])*3.1416/180.0))
            adjang1.append(360.0-angles[av])
        if rates[av] < 0.0 and angles[av] > 180.0:
            adjacn2.append(accns[av]-peak*sin(angles[av]*3.1416/180.0))
            adjang2.append(angles[av])
            adjacn2.append(-accns[av]-peak*sin((360.0-angles[av])*3.1416/180.0))
            adjang2.append(360.0-angles[av])
#    INPOL1=numpy.interp(range(-10,371),adjang1,adjacn1,period=380)
#    BASEL1=peakutils.baseline(INPOL1,deg=7)
#    INPOL2=numpy.interp(range(-10,371),adjang2,adjacn2,period=380)
#    BASEL2=peakutils.baseline(INPOL2,deg=7)
    adjacn=[]
#    for av in range(len(accns)):
#        gravminus = accns[av]-peak*sin(angles[av]*3.1416/180.0)
#        adjacn.append(gravminus)
#        if rates[av] >= 0.0:
#            adjacn.append(gravminus-BASEL1[int(angles[av]+10)])
#        else:
#            adjacn.append(gravminus-BASEL2[int(angles[av]+10)])
#    plt.plot(range(-10,371),INPOL,"ro")
    plt.plot(angles,accns,"r")
#    plt.plot(angles,adjacn,"b")
#    plt.plot(range(-10,371),BASEL1,"g")
#    plt.plot(range(-10,371),BASEL2,"y")
    plt.plot(range(-10,371),[0]*381,"r")

    with open('outputIN', 'wb') as f:
        writer = csv.writer(f)
        for row in range(len(output)):
            writer.writerow(output[row].split(','))

    plt.show()     
        
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
        accAngle = atan2(accGravY,accGravZ)*180/3.142 # assumes accx is zero?? : roll=atan2(accy,sqrt(accx^2+accz^2))
        if self.KalAngle > 180.0:
            accAngle += 360.0
        accAngle = (180.0 - accAngle) * -1
        if abs(gyroRate) < 75.0:
            self.KalAngle = 0.94*(self.KalAngle + self.timeDelta * gyroRate) + 0.06 * accAngle
        else:
            self.KalAngle = self.KalAngle + self.timeDelta * gyroRate

        return

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
        self.R_angle = abs(gyroRate)/2
        if self.R_angle < 0.01:
            self.R_angle = 0.01
        
        
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

if __name__ == '__main__':
    main()        
