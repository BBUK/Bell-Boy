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
from math import atan, atan2, sqrt
import sys
import os.path
import time
import math
import csv

radius_factor=None

sample_period = 1/50.0 # this is the current sample period (50 samples/sec].

def main():
    kalfilter = Kalman()
    output=[]
    with open('input.csv', 'rb') as f:
        reader = csv.reader(f)
        raw_data = list(reader)
    count = 0
    sum_adj_factor = 0
    sum_radius = 0
    for i in xrange(len(raw_data)-1):
        ay1 = float(raw_data[i][5][4:])
        ay1p1 = float(raw_data[i+1][5][4:])

        az1 = float(raw_data[i][6][4:])
        az1p1 = float(raw_data[i+1][6][4:])

        ay2 = float(raw_data[i][8][4:])
        ay2p1 = float(raw_data[i+1][8][4:])
        
        az2 = float(raw_data[i][9][4:])
        az2p1 = float(raw_data[i+1][9][4:])

        gx1 = float(raw_data[i][10][4:])
        gx2 = float(raw_data[i][13][4:])
        
        tang1 = (az1-az2)/2.0
        tang2 = (az1p1-az2p1)/2.0
        if abs(gx1) > 200.0: # so moving pretty fast
#            print "got here %d %f %f"%(i,tang1*8192,tang2*8192)
            if (tang1 >= 0.0 and tang2 < 0.0) or (tang2 >= 0.0 and tang1 < 0.0): # a sign change
                interpolate = abs(tang1/(tang2-tang1))
                ay1i = ay1 + interpolate*(ay1p1-ay1)
                ay2i = ay2 + interpolate*(ay2p1-ay2)
                radius1 = (ay1i*9.81)/((gx1 * 3.1416/180)**2)
                radius2 = (ay2i*9.81)/((gx2 * 3.1416/180)**2)
                adj_factor = 1-((radius2-abs(radius1))/radius2)
                print "i={0:d} polate={1:.3f} rad1={2:.3f} rad2={3:.3f} fact={4:.3f}".format(i,interpolate,radius1,radius2,adj_factor)
                count += 1
                sum_adj_factor += adj_factor
                sum_radius += radius1
    if count != 0:
        radius_factor = sum_adj_factor/count
        radius = sum_radius/count
    else:
        radius_factor = 1.0
        radius = 0
        print "error"
        return
    count = 0
    for row in raw_data:
        ax1 = float(row[4][4:])
        ay1 = float(row[5][4:])
        az1 = float(row[6][4:])
        ax2 = float(row[7][4:])
        ay2 = float(row[8][4:])
        az2 = float(row[9][4:])

        gx1 = float(row[10][4:])
        gy1 = float(row[11][4:])
        gz1 = float(row[12][4:])
        gx2 = float(row[13][4:])
        gy2 = float(row[14][4:])
        gz2 = float(row[15][4:])
       
        if count < 50: # get a rough angle estimate for the first 50 samples (bell should be stationary)
            kalfilter.calculate(ay1,az1,gx1)
            count += 1
            continue
            
        accGravY = ay1 - radius*((gx1*3.142/180)**2)/9.81 # subtract centripetal acceleration from sensor, what's left should be gravity (ahem!)
        accGravZ = 1.0 - (accGravY ** 2) # assume that rest of gravity is along y sensor (only valid if x axis is properly aligned)
        # now to determine sign of accGravZ, use calculated angle for that
        tempAngle = kalfilter.KalAngle + kalfilter.timeDelta * (gx1 - kalfilter.bias)
        if (tempAngle < 0.0 or tempAngle > 180.0) and tempAngle < 360.0:
            accGravZ = -accGravZ
        kalfilter.calculate(accGravY,accGravZ, gx1)
        accTang = az1 - accGravZ
        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},TS :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(kalfilter.KalAngle,gx1,accTang*8192.0,0,ax1,ay1,az1,ax2,ay2,az2,gx1,gy1,gz1,gx2,gy2,gz2))

#        accGravY = (ay1 + ay2*radius_factor)/2.0 # for the Y axis gravity is pulling in the same direction on the sensor, centripetal acceleration in opposite direction so add to get gravity
#        accGravZ = (az1 + az2*radius_factor)/2.0 # for the Z axis gravity is pulling in the same direction on the sensor, tangential acceleration in opposite direction so add two readings to get gravity only.
#        accTang = (az1 - az2*radius_factor)/2.0 # this is the tangental acceleration signal we want to measure
#        avgGyro = (gx1 + gx2)/2.0 # may as well average the two X gyro readings
#        kalfilter.calculate(accGravY,accGravZ, avgGyro) 
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},TS :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(kalfilter.KalAngle,avgGyro,accTang*8192.0,0,ax1,ay1,az1,ax2,ay2,az2,gx1,gy1,gz1,gx2,gy2,gz2))
    with open('output.csv', 'wb') as f:
        writer = csv.writer(f)
        for row in output:
            writer.writerow(row.split(','))
        
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

if __name__ == '__main__':
    main()        
            
