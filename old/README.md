## Here is some old code preserved in case someone finds it useful

### kalman.h

This is a pretty good general Kalman filter for use with 6DOF data.  It produces good results but on testing was not any better than the Mahony filter used as part of the grabber.  It is based on https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion.  It's good for me because it is pure C and does not use anything other than the standard libraries.  Instructions for compilation and use are in the file.

### BNO080grabber.c and BNO080-general.c

I experimented for a bit with the BNO080 devices.  These produced good pose quaternions but tended to recalibrate themselves at inopportune moments.  The two files here are the specific driver for the Bell-Boy grabber and a more general one.

### Other files

Will not be of much use to anyone, except, perhaps the dual MPU6050 code (which I used when the Bell-Boy had two IMU ICs working in parallel.  It demonstrates how I aligned the FIFO outputs from the two ICs.

The 32kHz.c file shows how to create a 32768Hz (ish) clock from the Pi's PWM hardware.  This clock was used for the BNO080s (to save the hardware needing a crystal and a couple of caps).
