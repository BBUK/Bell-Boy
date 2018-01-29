# Bell-Boy
This repository contains the code for the Bell-Boy Project.

THIS IS CURRENTLY IN TESTING AND SHOULD NOT BE USED

The Bell-Boy is a device used to measure how hard a person pulls on a tower bell rope.  

Hardware

The core of the hardware comprises a Raspberry Pi (any version with the 40 pin header), 2 MPU6050 breakout boards and the Pi official wifi dongle.  I have found the dongle is required even for those Pis that have onboard wifi - the signal strength and reliability is greater with the dongle.

Further detail of the hardware and its construction will be detailed on the raspberrypi.org site.

setupBB.sh

This script takes a stock installation of Arch Linux Arm (for the Raspberry Pi) and adds all scripts and installs all required packages to turn the Pi into a Bell-Boy device. General instructions are: 
   (a)    Install Arch Linux to an SD card https://archlinuxarm.org/platforms/armv6/raspberry-pi or 
          https://archlinuxarm.org/platforms/armv7/broadcom/raspberry-pi-2;
   (b)    Copy this script to the FAT partition (for convenience)
   (c)    Pop the SD card into your Raspberry Pi and boot it up with an internet connection (use ethernet
          port or usb/ethernet adapter)
   (d)    SSH into the Pi (username alarm, password alarm), change to root user (su root, password root) 
   (e)    run this script - it will take a while

bb_dcmimu

This folder contains the source for a small executable which interfaces with websocketd (https://github.com/joewalnes/websocketd).  Websockets are used by the front-end Javascript to interface with the Bell-Boy device.  The executable takes data from IMU ICs, applies am extended Kalman filter (from https://github.com/hhyyti/dcm-imu) then makes the data available to the browser running the front end via websocketd. 

bellboy.js

The Javascript that takes the data from the Bell-Boy and presents it.  The browser must support HTML5 and websockets.

index.html

Index file.  Note that this file and the JS reference certain image files (icons used to control the front end).  These were licensed in my me but whilst I am permitted to include them on websites that I operate or in code that I distribute, I am not permitted to make them available in a separately downloadable form.

help.html

Help html file
