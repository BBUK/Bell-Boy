# Bell-Boy
This repository contains the code for the Bell-Boy Project.

THIS IS CURRENTLY IN TESTING AND SHOULD NOT BE USED

The Bell-Boy is a device used to measure how hard a person pulls on a tower bell rope.  I hope it will be a useful training aid.  

## Hardware

The core of the hardware comprises a Raspberry Pi Zero W and an IMU board.  The wiki gives construction information if you want to build your own.

Further detail of the history is detailed on the raspberrypi.org site.

The various files and a description of what they do is set out below

### setupBB.sh

This script takes a stock installation of Arch Linux Arm (for the Raspberry Pi) and adds all scripts and installs all required packages to set up the software environment.  See the wiki for more details 

### bb_dcmimu

This folder contains the source for two small executables.  The first "bb_dcmimu" interfaces with websocketd (https://github.com/joewalnes/websocketd).  Websockets are used by the front-end Javascript to interface with the Bell-Boy device.  The executable takes data from IMU ICs, applies am extended Kalman filter (from https://github.com/hhyyti/dcm-imu) then makes the data available to the browser running the front end via websocketd.

The second executable "bb_calibrate" is use for an optional (but worthwhile) calibration step.

### bellboy.js

The Javascript that takes the data from the Bell-Boy and presents it to the users browser.  The browser must support HTML5 and websockets.  More details for operation are in the wiki.

### index.html

Index file.  Note that this file and the JS reference certain image files (icons used to control the front end).  These were licensed in my me but whilst I am permitted to include them on websites that I operate or in code that I distribute, I am not permitted to make them available in a separately downloadable form.

### help.html

Help html file served by the Bell-Boy device.

# Donations!
[Donations](https://paypal.me/PBUK) to this project are welcome :)
