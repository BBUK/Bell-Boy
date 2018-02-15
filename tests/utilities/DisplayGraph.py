#!/usr/bin/python2

# Copyright (c) 2018 Peter Budd. All rights reserved
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

# This script is used to test calibration methods.

from time import time,sleep,strftime
from math import atan, atan2, sqrt, sin
import sys
import os.path
import time
import math
import csv
import matplotlib.pyplot as plt

smfactor = 0.85

calib = 886

def main():
    angles=[]
    smangles=[]
    smhandangles=[]
    smbackangles=[]
    rates=[]
    accns=[]
    smaccns=[]
    smcalaccns=[]
    smhandaccns=[]
    smbackaccns=[]
    with open('input', 'rb') as f:
        reader = csv.reader(f)
        raw_data = list(reader)
    for row in raw_data:
        angles.append(float(row[0][2:]))
        rates.append(float(row[1][2:]))
        ac = float(row[2][2:])
        if ac > 1100:
            ac=1100
        if ac < -1100:
            ac=-1100
        accns.append(ac)
                
        if len(smaccns) > 1:
            smaccns.append(smfactor*smaccns[len(smaccns)-1] + (1.0-smfactor) * accns[len(accns)-1])
        else:
            smaccns.append(accns[len(accns)-1])

        if len(smangles) > 1:
            smangles.append(smfactor*smangles[len(smangles)-1] + (1.0-smfactor) * angles[len(angles)-1])
        else:
            smangles.append(angles[len(angles)-1])

        smcalaccns.append(smaccns[len(smaccns)-1] - calib*(math.sin(math.radians(smangles[len(smangles)-1]))))

        if rates[len(rates) -1] >= 0:
            smhandangles.append(smangles[len(smangles)-1])
            smhandaccns.append(smaccns[len(smaccns)-1] - calib*(math.sin(math.radians(smangles[len(smangles)-1]))))
#            smhandaccns.append(smaccns[len(smaccns)-1])

        else:
            smbackangles.append(smangles[len(smangles)-1])
            smbackaccns.append(smaccns[len(smaccns)-1] - calib*(math.sin(math.radians(smangles[len(smangles)-1]))))

#    plt.plot(angles,accns,"r")
     
    plt.plot(smangles,smaccns,"r")
    SIN=[]
    for a in range(-15,375):
		SIN.append(calib*math.sin(math.radians(a)))
    plt.plot(range(-15,375),SIN,"m")
#    plt.plot(smhandangles,smhandaccns,"r.")
#    plt.plot(smbackangles,smbackaccns,"b.")
    plt.plot(smangles,smcalaccns,"b")

    plt.plot(range(-10,371),[0]*381,"g")

    plt.show()     
        

if __name__ == '__main__':
    main()        
