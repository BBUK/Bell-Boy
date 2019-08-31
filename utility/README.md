## Calibration

Two methods of calibration are provided.  The first uses imu_tk and provides more calibration data (but is difficult to set up) and the second is simpler and can be more easily run on the Bell-Boy itself.

### imu-tk
To calibrate the ICM20689, imu_tk needs to be installed.  imu_tk needs Ceres Solver to be installed.  Ceres Solver only just compiles on the 500MB Pi (better on the larger-memoried models).  Step by step instructions on how to install Ceres Solver and imu_tk are in the NotesOnInstallingimu_tk.txt file.  Other files used are ICM20689.h, BBimu_tk.cpp (a slight variation of the sample file provided as part of imu_tk), calibrate.c and getcalibration.c (the latter is not necessary for doing the calibration but is a handy way of seeing the calibration loaded).

### 6-position
This is in calibrate-6.c.  It is a reasonbly standard 6-position IMU calibration program for 6-DOF IMUs.  This is of the type that uses Gaussian elimination so does not require exact positioning on each of the six faces (within 10 degrees will be fine).  Compile it, run it and as for the imu_tk version, enter SAMP: a few times until you see stable numbers, then enter CALI: to start the calibration process (and follow the on-screen instructions).  Once done, enter SAVE: to save the calibration data to the Bell-Boy.  This program produces good results (pretty much indisguishable from imu_tk in real use) but does not calibrate for gyro scale or axis misalignment errors (which imu_tk does).

## g calculator

gcalculator.xlsx is a little spreadsheet I used to work out what g is in my viscinity.  I am sure the variance in g is not enough to make much difference to results.

## Battery

batteryrecord.txt and BatteryCalibration.xlsx are files file showing how battery voltage is reported over time.  As LiPo batteries discharge on a curve, I could use this data to work out how much battery time is remaining.

## Logo

Bell2LogoSheet.docx is just a Word file with the Bell-Boy logo printed on it (ready to go onto a sticker sheet).



