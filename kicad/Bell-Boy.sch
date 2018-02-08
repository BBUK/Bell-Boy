EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Bell-Boy-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_02X06 P1
U 1 1 5A7340A6
P 5050 2300
F 0 "P1" H 5050 2650 50  0000 C CNN
F 1 "RPi Header" H 5050 1950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x06" H 5050 1100 50  0001 C CNN
F 3 "" H 5050 1100 50  0000 C CNN
	1    5050 2300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P2
U 1 1 5A734148
P 6450 2300
F 0 "P2" H 6450 2650 50  0000 C CNN
F 1 "Adafruit NXP Header" V 6550 2300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 6450 2300 50  0001 C CNN
F 3 "" H 6450 2300 50  0000 C CNN
	1    6450 2300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P3
U 1 1 5A734195
P 5200 3200
F 0 "P3" H 5200 3400 50  0000 C CNN
F 1 "LiPo Shim Header" V 5300 3200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 5200 3200 50  0001 C CNN
F 3 "" H 5200 3200 50  0000 C CNN
	1    5200 3200
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 5A7341BF
P 6550 2950
F 0 "P4" H 6550 3100 50  0000 C CNN
F 1 "LED Header" V 6650 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6550 2950 50  0001 C CNN
F 3 "" H 6550 2950 50  0000 C CNN
	1    6550 2950
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 5A734200
P 5650 3900
F 0 "P5" H 5650 4050 50  0000 C CNN
F 1 "Switch Header" V 5750 3900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 5650 3900 50  0001 C CNN
F 3 "" H 5650 3900 50  0000 C CNN
	1    5650 3900
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 5A7343BD
P 5150 3750
F 0 "D1" H 5150 3850 50  0000 C CNN
F 1 "1N4148" H 5150 3650 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 5150 3750 50  0001 C CNN
F 3 "" H 5150 3750 50  0000 C CNN
	1    5150 3750
	-1   0    0    1   
$EndComp
$Comp
L D_Schottky D2
U 1 1 5A734466
P 5150 4050
F 0 "D2" H 5150 4150 50  0000 C CNN
F 1 "BAT85" H 5150 3950 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 5150 4050 50  0001 C CNN
F 3 "" H 5150 4050 50  0000 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D3
U 1 1 5A734499
P 4050 3550
F 0 "D3" H 4050 3650 50  0000 C CNN
F 1 "BAT85" H 4050 3450 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 4050 3550 50  0001 C CNN
F 3 "" H 4050 3550 50  0000 C CNN
	1    4050 3550
	-1   0    0    1   
$EndComp
$Comp
L R R1
U 1 1 5A7344CA
P 4650 3200
F 0 "R1" V 4730 3200 50  0000 C CNN
F 1 "4.7k" V 4650 3200 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 4580 3200 50  0001 C CNN
F 3 "" H 4650 3200 50  0000 C CNN
	1    4650 3200
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5A734574
P 6050 2900
F 0 "R2" V 6130 2900 50  0000 C CNN
F 1 "270R" V 6050 2900 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 5980 2900 50  0001 C CNN
F 3 "" H 6050 2900 50  0000 C CNN
	1    6050 2900
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR01
U 1 1 5A734BF0
P 4100 1800
F 0 "#PWR01" H 4100 1650 50  0001 C CNN
F 1 "+3.3V" H 4100 1940 50  0000 C CNN
F 2 "" H 4100 1800 50  0000 C CNN
F 3 "" H 4100 1800 50  0000 C CNN
	1    4100 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5A734C68
P 6200 3000
F 0 "#PWR02" H 6200 2750 50  0001 C CNN
F 1 "GND" H 6200 2850 50  0000 C CNN
F 2 "" H 6200 3000 50  0000 C CNN
F 3 "" H 6200 3000 50  0000 C CNN
	1    6200 3000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 5A7352E3
P 5800 2400
F 0 "#PWR03" H 5800 2250 50  0001 C CNN
F 1 "+3.3V" H 5800 2540 50  0000 C CNN
F 2 "" H 5800 2400 50  0000 C CNN
F 3 "" H 5800 2400 50  0000 C CNN
	1    5800 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2900 6200 2900
Wire Wire Line
	6200 3000 6350 3000
Wire Wire Line
	5800 2400 5800 2550
Wire Wire Line
	6000 2350 6600 2350
Wire Wire Line
	5800 2550 6250 2550
Wire Wire Line
	6250 2150 5550 2150
Wire Wire Line
	5550 2150 5550 1900
Wire Wire Line
	5550 1900 4550 1900
Wire Wire Line
	4550 1900 4550 2150
Wire Wire Line
	4550 2150 4800 2150
Wire Wire Line
	6250 2250 5600 2250
Wire Wire Line
	5600 2250 5600 2750
Wire Wire Line
	5600 2750 4550 2750
Wire Wire Line
	4550 2750 4550 2250
Wire Wire Line
	4550 2250 4800 2250
Wire Wire Line
	4800 2050 4100 2050
Wire Wire Line
	4100 2050 4100 1800
Wire Wire Line
	5300 2250 5400 2250
Wire Wire Line
	5400 2250 5400 1650
Wire Wire Line
	5400 1650 6000 1650
Wire Wire Line
	4800 3200 5000 3200
Wire Wire Line
	5000 3300 5000 3750
Wire Wire Line
	5300 3750 5450 3750
Wire Wire Line
	5450 3750 5450 3850
Wire Wire Line
	5450 4050 5450 3950
Wire Wire Line
	5300 4050 5450 4050
Wire Wire Line
	4850 4050 5000 4050
Wire Wire Line
	4850 3200 4850 4050
Connection ~ 4850 3200
Wire Wire Line
	5300 2550 5300 2850
Wire Wire Line
	5300 2850 3900 2850
Wire Wire Line
	3900 2850 3900 3550
Wire Wire Line
	4200 3550 4850 3550
Connection ~ 4850 3550
Wire Wire Line
	5900 2900 5500 2900
Wire Wire Line
	5500 2900 5500 2650
Wire Wire Line
	5500 2650 5300 2650
Connection ~ 5300 2650
$Comp
L D_Schottky D4
U 1 1 5A7358A2
P 4000 2550
F 0 "D4" H 4000 2650 50  0000 C CNN
F 1 "BAT85" H 4000 2450 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 4000 2550 50  0001 C CNN
F 3 "" H 4000 2550 50  0000 C CNN
	1    4000 2550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 2550 4800 2550
Wire Wire Line
	3850 2550 3850 4300
Connection ~ 5400 3750
$Comp
L PWR_FLAG #FLG04
U 1 1 5A735B43
P 4700 1300
F 0 "#FLG04" H 4700 1395 50  0001 C CNN
F 1 "PWR_FLAG" H 4700 1480 50  0000 C CNN
F 2 "" H 4700 1300 50  0000 C CNN
F 3 "" H 4700 1300 50  0000 C CNN
	1    4700 1300
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG05
U 1 1 5A735B78
P 5100 1300
F 0 "#FLG05" H 5100 1395 50  0001 C CNN
F 1 "PWR_FLAG" H 5100 1480 50  0000 C CNN
F 2 "" H 5100 1300 50  0000 C CNN
F 3 "" H 5100 1300 50  0000 C CNN
	1    5100 1300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 5A735BD9
P 4700 1500
F 0 "#PWR06" H 4700 1350 50  0001 C CNN
F 1 "+3.3V" H 4700 1640 50  0000 C CNN
F 2 "" H 4700 1500 50  0000 C CNN
F 3 "" H 4700 1500 50  0000 C CNN
	1    4700 1500
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR07
U 1 1 5A735C0E
P 5100 1500
F 0 "#PWR07" H 5100 1250 50  0001 C CNN
F 1 "GND" H 5100 1350 50  0000 C CNN
F 2 "" H 5100 1500 50  0000 C CNN
F 3 "" H 5100 1500 50  0000 C CNN
	1    5100 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 1300 4700 1500
Wire Wire Line
	5100 1500 5100 1300
Wire Wire Line
	3850 4300 5400 4300
Wire Wire Line
	5400 4300 5400 4050
Connection ~ 5400 4050
Wire Wire Line
	4500 3200 4300 3200
$Comp
L GND #PWR08
U 1 1 5A739DA1
P 4300 3200
F 0 "#PWR08" H 4300 2950 50  0001 C CNN
F 1 "GND" H 4300 3050 50  0000 C CNN
F 2 "" H 4300 3200 50  0000 C CNN
F 3 "" H 4300 3200 50  0000 C CNN
	1    4300 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5A739DD6
P 4850 2950
F 0 "#PWR09" H 4850 2700 50  0001 C CNN
F 1 "GND" H 4850 2800 50  0000 C CNN
F 2 "" H 4850 2950 50  0000 C CNN
F 3 "" H 4850 2950 50  0000 C CNN
	1    4850 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2950 5000 2950
Wire Wire Line
	5000 2950 5000 3100
Wire Wire Line
	6000 1650 6000 2350
$Comp
L CONN_01X04 P6
U 1 1 5A74F097
P 7050 2250
F 0 "P6" H 7050 2500 50  0000 C CNN
F 1 "MPU6050" V 7150 2250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 7050 2250 50  0001 C CNN
F 3 "" H 7050 2250 50  0000 C CNN
	1    7050 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2050 5300 1450
Wire Wire Line
	5300 1450 6850 1450
Wire Wire Line
	6850 1450 6850 2100
Wire Wire Line
	6350 2300 6350 2250
$Comp
L CONN_01X04 P7
U 1 1 5A74F2F6
P 7500 2250
F 0 "P7" H 7500 2500 50  0000 C CNN
F 1 "MPU6050" V 7600 2250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 7500 2250 50  0001 C CNN
F 3 "" H 7500 2250 50  0000 C CNN
	1    7500 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2400 7300 2400
Wire Wire Line
	6650 2300 7300 2300
Wire Wire Line
	6600 2200 7300 2200
Wire Wire Line
	6850 2100 7300 2100
Wire Wire Line
	6150 2150 6150 1800
Wire Wire Line
	6150 1800 6750 1800
Wire Wire Line
	6750 1800 6750 2400
Connection ~ 6850 2400
Connection ~ 6150 2150
Wire Wire Line
	6650 2300 6650 2700
Wire Wire Line
	6650 2700 6150 2700
Wire Wire Line
	6150 2700 6150 2250
Connection ~ 6150 2250
Connection ~ 6850 2300
Wire Wire Line
	6600 2350 6600 2200
Connection ~ 6850 2200
Connection ~ 6250 2350
$EndSCHEMATC