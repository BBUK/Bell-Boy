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
    coraccn = 0.0
    ax1 = 0
    az1= 0
    gy1 =0
    radius = 0.133
    accnadjust = 940
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
        roll = dcmimu.calculate(math.radians(gy1),math.radians(gx1),math.radians(gz1),-ay1*9.81,ax1*9.81,az1*9.81,sample_period)
        calcAngle = math.degrees(roll) + flipAngleCorrection
#        if calcAngle > 180.0:
#            accAngle += 360.0
        if (lastcalcAngle - calcAngle) > 250 and flipAngleCorrection != 360: #positive to negative flip
            flipAngleCorrection = 360
            calcAngle += 360
            tempor = (1 - az1)/((gy1) ** 2)
            radius = 0.9*radius + 0.1 * tempor * 32203
            print "R: %f " % radius
        elif (lastcalcAngle - calcAngle) < -250 and flipAngleCorrection != 0: # negative to positive flip
            calcAngle -= 360
            flipAngleCorrection = 0
#            accAngle += flipAngleCorrection
#            output.append("flipDown")
            tempor = (1 - az1)/((gy1) ** 2)
            radius = 0.9*radius + 0.1 * tempor  * 32203
            print "R: %f " % radius
        if calcAngle > 90.0 and lastcalcAngle < 90.0:
			accnadjust = 0.9*accnadjust + 0.1*(max(accTang,lastaccTang))
			print "AA: %f T: %f" % (accnadjust, max(accTang,lastaccTang))
        if calcAngle < 270.0 and lastcalcAngle > 270.0:
			accnadjust = 0.9*accnadjust - 0.1*(min(accTang,lastaccTang))
			print "AA: %f T: %f" % (accnadjust, -min(accTang,lastaccTang))

        lastcalcAngle = calcAngle
        lastaccTang = accTang

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
#        avaccn = 0.90* avaccn + 0.10* (accTang - 910*sin(math.radians(calcAngle)))
        tempr = 0.133 
        avaccn = 0.75*avaccn + 0.25*(accTang-930*sin(math.radians(calcAngle)))
#        kalaccn = 0.90* kalaccn + 0.10*math.degrees(kalfilter.a[0]) 
        accns.append(avaccn)
        coraccn = 0.50*coraccn +0.50* ((ax1 - sin(math.radians(calcAngle)) - ((accTang/57)*0.133)/9.81) * 500)
        corraccns.append(accTang)

        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}, ATN:{9:.3f}".format(calcAngle,agyro,avaccn,ax1,ay1,az1,gx1,gy1,gz1, accAngle))
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}, ATN:{9:.3f}".format(calcAngle,agyro,accTang-1000*sin(calcAngle*3.1416/180),ax1,ay1,az1,gx1,gy1,gz1, accAngle))

#        accGravY = (ay1 + ay2*radius_factor)/2.0 # for the Y axis gravity is pulling in the same direction on the sensor, centripetal acceleration in opposite direction so add to get gravity
#        accGravZ = (az1 + az2*radius_factor)/2.0 # for the Z axis gravity is pulling in the same direction on the sensor, tangential acceleration in opposite direction so add two readings to get gravity only.
#        accTang = (az1 - az2*radius_factor)/2.0 # this is the tangental acceleration signal we want to measure
#        avgGyro = (gx1 + gx2)/2.0 # may as well average the two X gyro readings
#        kalfilter.calculate(accGravY,accGravZ, avgGyro) 
#        output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},OA :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},AX2:{7:.3f},AY2:{8:.3f},AZ2:{9:.3f},GX1:{10:.3f},GY1:{11:.3f},GZ1:{12:.3f},GX2:{13:.3f},GY2:{14:.3f},GZ2:{15:.3f}".format(kalfilter.KalAngle,avgGyro,accTang*8192.0,0,ax1,ay1,az1,0,0,0,gx1,gy1,gz1,0,0,0))
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
#    plt.plot(angles,corraccns,"y")
    plt.plot(angles,corraccns,"b")
    plt.plot(angles,accns,"r")
    SIN=[]
    for a in range(-15,375):
		SIN.append(930*math.sin(math.radians(a)))
#    plt.plot(angles,adjacn,"b")
#    plt.plot(range(-10,371),BASEL1,"g")
#    plt.plot(range(-10,371),BASEL2,"y")
    plt.plot(range(-15,375),SIN,"g")

    plt.plot(range(-10,371),[0]*381,"r")

    with open('outputKal', 'wb') as f:
        writer = csv.writer(f)
        for row in range(len(output)):
            writer.writerow(output[row].split(','))

    plt.show()     
        

if __name__ == '__main__':
    main()        
