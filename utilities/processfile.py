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

from time import time,sleep,strftime
from math import atan, atan2, sqrt
import sys
import os.path
import time
import math
import csv

sample_period = 1/50.0 # this is the current sample period (50 samples/sec].

def main():
    kalfilter = Kalman()
    output=[]
    with open('input.csv', 'rb') as f:
        reader = csv.reader(f)
        raw_data = list(reader)
        
        
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
        
        accGravY = (ay1 + ay2*0.83)/2.0 # for the Y axis gravity is pulling in the same direction on the sensor, centripetal acceleration in opposite direction so add to get gravity
        accGravZ = (az1 + az2*0.83)/2.0 # for the Z axis gravity is pulling in the same direction on the sensor, tangential acceleration in opposite direction so add two readings to get gravity only.
        accTang = (az1 - az2)/2.0 # this is the tangental acceleration signal we want to measure
        avgGyro = (gx1 + gx2)/2.0 # may as well average the two X gyro readings
        kalfilter.calculate(accGravY,accGravZ, avgGyro) 
        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},TS :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(kalfilter.KalAngle,avgGyro,accTang,0,data1[0],data1[1],data1[2],data2[0],data2[1],data2[2],data1[3],data1[4],data1[5],data2[3],data2[4],data2[5]))
    with open('output.csv', 'wb') as f:
        writer = csv.writer(f)
        writer.writerows(output)
        
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
            