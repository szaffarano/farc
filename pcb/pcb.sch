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
LIBS:special
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
LIBS:pcb-cache
EELAYER 24 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "FaRC"
Date "15 mar 2015"
Rev "1.0"
Comp "Sebastián Zaffarano"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATTINY2313A-P IC1
U 1 1 54FB3F3F
P 4350 3250
F 0 "IC1" H 3200 4250 40  0000 C CNN
F 1 "ATTINY2313A-P" H 5300 2350 40  0000 C CNN
F 2 "DIP20" H 4350 3250 35  0000 C CIN
F 3 "" H 4350 3250 60  0000 C CNN
	1    4350 3250
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 54FB3F67
P 6950 3750
F 0 "D1" H 6950 3850 50  0000 C CNN
F 1 "LED" H 6950 3650 50  0000 C CNN
F 2 "" H 6950 3750 60  0000 C CNN
F 3 "" H 6950 3750 60  0000 C CNN
	1    6950 3750
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 54FB3F7B
P 6500 3750
F 0 "R1" V 6580 3750 40  0000 C CNN
F 1 "R" V 6507 3751 40  0000 C CNN
F 2 "" V 6430 3750 30  0000 C CNN
F 3 "" H 6500 3750 30  0000 C CNN
	1    6500 3750
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 54FB3F8F
P 6500 4000
F 0 "R2" V 6580 4000 40  0000 C CNN
F 1 "R" V 6507 4001 40  0000 C CNN
F 2 "" V 6430 4000 30  0000 C CNN
F 3 "" H 6500 4000 30  0000 C CNN
	1    6500 4000
	0    1    1    0   
$EndComp
$Comp
L LM317AT U1
U 1 1 54FB3FDC
P 1800 3450
F 0 "U1" H 1600 3650 40  0000 C CNN
F 1 "LM317AT" H 1800 3650 40  0000 L CNN
F 2 "TO-220" H 1800 3550 30  0000 C CIN
F 3 "" H 1800 3450 60  0000 C CNN
	1    1800 3450
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 54FB3FF0
P 1300 3600
F 0 "C1" H 1300 3700 40  0000 L CNN
F 1 "100 nF" H 1306 3515 40  0000 L CNN
F 2 "" H 1338 3450 30  0000 C CNN
F 3 "" H 1300 3600 60  0000 C CNN
	1    1300 3600
	1    0    0    -1  
$EndComp
$Comp
L CAPAPOL C2
U 1 1 54FB4004
P 2300 3600
F 0 "C2" H 2350 3700 40  0000 L CNN
F 1 "10 mF" H 2350 3500 40  0000 L CNN
F 2 "" H 2400 3450 30  0000 C CNN
F 3 "" H 2300 3600 300 0000 C CNN
	1    2300 3600
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 54FB4018
P 6550 3500
F 0 "SW1" H 6700 3610 50  0000 C CNN
F 1 "SW_PUSH" H 6550 3420 50  0000 C CNN
F 2 "" H 6550 3500 60  0000 C CNN
F 3 "" H 6550 3500 60  0000 C CNN
	1    6550 3500
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 54FB402C
P 4200 1800
F 0 "C3" H 4200 1900 40  0000 L CNN
F 1 "100 nF" H 4206 1715 40  0000 L CNN
F 2 "" H 4238 1650 30  0000 C CNN
F 3 "" H 4200 1800 60  0000 C CNN
	1    4200 1800
	1    0    0    -1  
$EndComp
$Comp
L CAPAPOL C4
U 1 1 54FB4040
P 4500 1800
F 0 "C4" H 4550 1900 40  0000 L CNN
F 1 "10 mF" H 4550 1700 40  0000 L CNN
F 2 "" H 4600 1650 30  0000 C CNN
F 3 "" H 4500 1800 300 0000 C CNN
	1    4500 1800
	1    0    0    -1  
$EndComp
$Comp
L CONN_3X2 P1
U 1 1 54FB4068
P 3450 4800
F 0 "P1" H 3450 5050 50  0000 C CNN
F 1 "ISP" V 3450 4850 40  0000 C CNN
F 2 "" H 3450 4800 60  0000 C CNN
F 3 "" H 3450 4800 60  0000 C CNN
	1    3450 4800
	1    0    0    -1  
$EndComp
$Comp
L CONN_4X2 P2
U 1 1 54FB407C
P 5250 4900
F 0 "P2" H 5250 5150 50  0000 C CNN
F 1 "RF24L01+" V 5250 4900 40  0000 C CNN
F 2 "" H 5250 4900 60  0000 C CNN
F 3 "" H 5250 4900 60  0000 C CNN
	1    5250 4900
	1    0    0    -1  
$EndComp
$Comp
L RELAY_2RT K1
U 1 1 55048CD6
P 7850 4400
F 0 "K1" H 7800 4800 70  0000 C CNN
F 1 "RELAY_2RT" H 8000 3900 70  0000 C CNN
F 2 "" H 7850 4400 60  0000 C CNN
F 3 "" H 7850 4400 60  0000 C CNN
	1    7850 4400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR01
U 1 1 55048CFA
P 4500 2000
F 0 "#PWR01" H 4500 2100 30  0001 C CNN
F 1 "VCC" H 4500 2100 30  0000 C CNN
F 2 "" H 4500 2000 60  0000 C CNN
F 3 "" H 4500 2000 60  0000 C CNN
	1    4500 2000
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR02
U 1 1 55048D0E
P 4350 4250
F 0 "#PWR02" H 4350 4250 30  0001 C CNN
F 1 "GND" H 4350 4180 30  0001 C CNN
F 2 "" H 4350 4250 60  0000 C CNN
F 3 "" H 4350 4250 60  0000 C CNN
	1    4350 4250
	1    0    0    -1  
$EndComp
Text Label 2800 2450 0    60   ~ 0
RST
NoConn ~ 3000 2750
NoConn ~ 3000 2950
Text Label 5950 3150 0    60   ~ 0
SCK
Text Label 5950 3050 0    60   ~ 0
MISO/DO
Text Label 5950 2950 0    60   ~ 0
MOSI/DI
Text Label 5950 2850 0    60   ~ 0
CSN
Text Label 5950 2750 0    60   ~ 0
CE
Text Label 5950 2650 0    60   ~ 0
IRQ
NoConn ~ 5700 2450
NoConn ~ 5700 2550
NoConn ~ 5700 3350
NoConn ~ 5700 3450
NoConn ~ 5700 3550
NoConn ~ 5700 3950
Text Label 4700 4750 0    60   ~ 0
GND
Text Label 4700 4850 0    60   ~ 0
CE
Text Label 4700 4950 0    60   ~ 0
SCK
Text Label 4550 5050 0    60   ~ 0
MOSI/DI
Text Label 5800 4750 0    60   ~ 0
VCC
Text Label 5800 4850 0    60   ~ 0
CSN
Text Label 5800 4950 0    60   ~ 0
MISO/DO
Text Label 5800 5050 0    60   ~ 0
IRQ
Text Label 2650 4650 0    60   ~ 0
MISO/DO
Text Label 2850 4750 0    60   ~ 0
SCK
Text Label 2850 4850 0    60   ~ 0
RST
Text Label 4050 4650 0    60   ~ 0
VCC
Text Label 4050 4750 0    60   ~ 0
MOSI/DI
Text Label 4050 4850 0    60   ~ 0
GND
Text Label 2550 3400 0    60   ~ 0
VCC
Text Label 1050 3400 0    60   ~ 0
Vin
Text Label 1800 3950 0    60   ~ 0
GND
$Comp
L GND #PWR03
U 1 1 55049081
P 1800 3950
F 0 "#PWR03" H 1800 3950 30  0001 C CNN
F 1 "GND" H 1800 3880 30  0001 C CNN
F 2 "" H 1800 3950 60  0000 C CNN
F 3 "" H 1800 3950 60  0000 C CNN
	1    1800 3950
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR04
U 1 1 55049095
P 2550 3400
F 0 "#PWR04" H 2550 3500 30  0001 C CNN
F 1 "VCC" H 2550 3500 30  0000 C CNN
F 2 "" H 2550 3400 60  0000 C CNN
F 3 "" H 2550 3400 60  0000 C CNN
	1    2550 3400
	-1   0    0    1   
$EndComp
$Comp
L NPN Q1
U 1 1 55049223
P 7000 4300
F 0 "Q1" H 7000 4150 50  0000 R CNN
F 1 "NPN" H 7000 4450 50  0000 R CNN
F 2 "" H 7000 4300 60  0000 C CNN
F 3 "" H 7000 4300 60  0000 C CNN
	1    7000 4300
	1    0    0    -1  
$EndComp
Text Label 7100 4000 0    60   ~ 0
VCC
Text Label 7100 4900 0    60   ~ 0
GND
Text Label 6750 4000 0    60   ~ 0
RX
NoConn ~ 8250 4550
NoConn ~ 8250 4350
NoConn ~ 8250 4250
NoConn ~ 8250 4050
NoConn ~ 7450 4150
NoConn ~ 7450 4450
Text Label 4350 1450 0    60   ~ 0
GND
Text Label 7300 3500 0    60   ~ 0
GND
Text Label 7300 3750 0    60   ~ 0
GND
$Comp
L PWR_FLAG #FLG05
U 1 1 550496DA
P 2300 3400
F 0 "#FLG05" H 2300 3495 30  0001 C CNN
F 1 "PWR_FLAG" H 2300 3580 30  0000 C CNN
F 2 "" H 2300 3400 60  0000 C CNN
F 3 "" H 2300 3400 60  0000 C CNN
	1    2300 3400
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 55049763
P 1300 3400
F 0 "#FLG06" H 1300 3495 30  0001 C CNN
F 1 "PWR_FLAG" H 1300 3580 30  0000 C CNN
F 2 "" H 1300 3400 60  0000 C CNN
F 3 "" H 1300 3400 60  0000 C CNN
	1    1300 3400
	1    0    0    -1  
$EndComp
$Comp
L DIODE D2
U 1 1 5505DD3C
P 6750 4650
F 0 "D2" H 6750 4750 40  0000 C CNN
F 1 "DIODE" H 6750 4550 40  0000 C CNN
F 2 "" H 6750 4650 60  0000 C CNN
F 3 "" H 6750 4650 60  0000 C CNN
	1    6750 4650
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5505DF45
P 5250 4400
F 0 "C5" H 5250 4500 40  0000 L CNN
F 1 "10 nf" H 5256 4315 40  0000 L CNN
F 2 "" H 5288 4250 30  0000 C CNN
F 3 "" H 5250 4400 60  0000 C CNN
	1    5250 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	2800 2450 3000 2450
Wire Wire Line
	5700 2950 5950 2950
Wire Wire Line
	5700 3050 5950 3050
Wire Wire Line
	5700 3150 5950 3150
Wire Wire Line
	5700 2750 5950 2750
Wire Wire Line
	5700 2850 5950 2850
Wire Wire Line
	5700 2650 5950 2650
Wire Wire Line
	5700 3650 6100 3650
Wire Wire Line
	5700 3750 6250 3750
Wire Wire Line
	5700 3850 6100 3850
Wire Wire Line
	4700 4750 4850 4750
Wire Wire Line
	4700 4850 4850 4850
Wire Wire Line
	4700 4950 4850 4950
Wire Wire Line
	4550 5050 4850 5050
Wire Wire Line
	5650 4750 5800 4750
Wire Wire Line
	5650 4850 5800 4850
Wire Wire Line
	5650 4950 5800 4950
Wire Wire Line
	5650 5050 5800 5050
Wire Wire Line
	3850 4650 4050 4650
Wire Wire Line
	3850 4750 4050 4750
Wire Wire Line
	3850 4850 4050 4850
Wire Wire Line
	2650 4650 3050 4650
Wire Wire Line
	3050 4750 2850 4750
Wire Wire Line
	3050 4850 2850 4850
Wire Wire Line
	2200 3400 2550 3400
Wire Wire Line
	1300 3800 2300 3800
Wire Wire Line
	1800 3700 1800 3950
Wire Wire Line
	6100 3850 6100 4000
Wire Wire Line
	6100 4000 6250 4000
Wire Wire Line
	6100 3650 6100 3500
Wire Wire Line
	6100 3500 6250 3500
Wire Wire Line
	7150 3750 7300 3750
Wire Wire Line
	6750 4000 6800 4000
Wire Wire Line
	6800 4000 6800 4300
Wire Wire Line
	7100 4100 7100 4000
Wire Wire Line
	7100 4500 7100 4650
Wire Wire Line
	6950 4650 7450 4650
Wire Wire Line
	7450 4750 7100 4750
Wire Wire Line
	7100 4750 7100 4900
Wire Wire Line
	4200 2000 4500 2000
Wire Wire Line
	4350 2150 4350 2000
Connection ~ 4350 2000
Wire Wire Line
	4200 1600 4500 1600
Wire Wire Line
	4350 1600 4350 1450
Connection ~ 4350 1600
Wire Wire Line
	6850 3500 7300 3500
Wire Wire Line
	1050 3400 1400 3400
Connection ~ 1300 3400
Connection ~ 7100 4650
Wire Wire Line
	6550 4650 6400 4650
Wire Wire Line
	6400 4650 6400 4800
Wire Wire Line
	6400 4800 7100 4800
Connection ~ 7100 4800
Wire Wire Line
	4850 4750 4850 4400
Wire Wire Line
	4850 4400 5050 4400
Wire Wire Line
	5450 4400 5650 4400
Wire Wire Line
	5650 4400 5650 4750
$EndSCHEMATC