## Calibration

To calibrate the ICM20689, imu_tk needs to be installed.  imu_tk needs Ceres Solver to be installed.  Ceres Solver only just compiles on the 500MB Pi (better on the larger-memoried models).  Step by step instructions on how to install Ceres Solver and imu_tk are in the NotesOnInstallingimu_tk.txt file.  Other files used are ICM20689.h, BBimu_tk.cpp (a slight variation of the sample file provided as part of imu_tk), calibrate.c and getcalibration.c (the latter is not necessary for doing the calibration but is a handy way of seeing the calibration loaded).

## g calculator

gcalculator.xlsx is a little spreadsheet I used to work out what g is in my viscinity.  I am sure the variance in g is not enough to make much difference to results.

## Battery

batteryrecord.txt and BatteryCalibration.xlsx are files file showing how battery voltage is reported over time.  As LiPo batteries discharge on a curve, I could use this data to work out how much battery time is remaining.

## Logo

Bell2LogoSheet.docx is just a Word file with the Bell-Boy logo printed on it (ready to go onto a sticker sheet).



