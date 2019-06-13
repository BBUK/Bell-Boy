# Bell-Boy
NEW VERSION IMMINENT - THIS REPOSITORY CONTAINS A GREAT DEAL OF CODE AND INFORMATION IN FLUX BETWEEN VERSIONS SO SHOULD NOT BE RELIED UPON

This repository contains the code for the Bell-Boy Project.  

The Bell-Boy is a device used to measure how hard a person pulls on a tower bell rope.  I hope it will be a useful training aid.  

## Hardware

The core of the hardware comprises a Raspberry Pi Zero and a PCB mounted on it containing an IMU IC (an ICM20689), a ATMEGA328 with the Arduino bootloader and power management and battery charging circuitry.  The Arduino manages battery charging, controls a LED and also stores device calibration data.  The battery is a LiPo integral to the device and is charged via power sourced from the Rasperry Pi's micro USB power socket.  The wiki gives construction information if you want to build your own.

The various files and a description of what they do is set out below

This is the second version of the hardware.  The first version was designed to be mounted on top of the headstock.  More accurate measurements were found to have been achieved by mounting the device closer to the gudgeon pins of the bell.  That required the device to be smaller, which required a LiPo battery. As I could not find a case that would fit a Pi0 and one of the AA sized LiPo batteries, a "flat" LiPo was used which then needed associated charging paraphernalia.

Despite only using one consumer-grade IMU, the second version is *much* more accurate than the first.  There is virtually zero drift in angular readings for all three axes (less than 0.3 degrees per hour).  Acceleration measurements are similarly improved.

## Software

### setupBB.sh

This script takes a stock installation of Arch Linux Arm (for the Raspberry Pi) and adds all scripts and installs all required packages to set up the software environment.  See the wiki for more details 

### grabber

This folder contains the source for two small executables.  

The first, "grabber" takes data from the IMU, transforms it into angle, rotational velocity and rotational acceleration and pushes the data to the users browser via a websocket.  As the force applied by the ringer on the rope needs to be distinguished from gravity, vibration and sensor noise, the solution:

(a) oversamples data from the IMUs (sensors output at 500Hz but the user sees 125Hz).

(b) uses a straightforward Mayhony-type filter to calculate angles (I previously used the extended Kalman filter (from https://github.com/hhyyti/dcm-imu) to calculate bell angle but it produced no better results in this application).

(c) uses a Savitsky-Golay filter to smooth the measured rotational acceleration

(d) requires an one-time calibration of the IMU system (from https://bitbucket.org/alberto_pretto/imu_tk).  This is a really useful method of scale and bias calibration that does not require the IMU to be exactly positioned.

The grabber communicates with the user's broswer via websocketd (https://github.com/joewalnes/websocketd). 

The second executable "powermonitor" monitors battery level reports from the Arduino and shuts down the Pi if he battery is too low.  It only operates when the grabber is not running (the grabber only runs when there is an active websocket connection to the user's browser - if there is such a connection, the grabber takes over power monitoring).

### bellboy.js

The Javascript that takes the data from the Bell-Boy and presents it to the users browser.  The browser must support HTML5 and websockets.  More details for operation are in the wiki.

### index.html

Index file.  Note that this file and the JS reference certain image files (icons used to control the front end).  These were licensed in my me but whilst I am permitted to include them on websites that I operate or in code that I distribute, I am not permitted to make them available in a separately downloadable form.

### help.html

Help html file served by the Bell-Boy device.

### utility

Some useful utility programs and files.  There is some information here on installing imu_tk (and in particular ceres-solver which is used by imu_tk).

### arduino

Contains the Arduino program running on the "Arduino'd" ATMEGA328 on the Bell-Boy PCB

### old 

Contains some old code and in particular some code for BNO080 interfacing over the Raspberry Pi's SPI which some people may find useful.  I am no longer using the BNO080 so don't expect this code to be updated.

# Donations!
[Donations](https://paypal.me/PBUK) to this project are welcome :)
