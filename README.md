ESP32 Camera Tracker
Written to run on ESP32-Cam board + Lora + GPS

Original code by mkshrps - https://github.com/mkshrps/espcam
Some other ideas from Stuart - https://stuartsprojects.github.io/2020/08/27/ESP32CAM-TrackerAddOn-Board.html

1. Transmits jpg data over lora using SSDV protocol
2. Interleaves GPS UKHAS protocol tracking data packets 
3. Compatible with PITS Gateway receiver

ESP32-CAM board does not include lora device or GPS device which need to be connected.

Program through the U0R and U0T pins (serial pins) with GPIO0 connected to Gnd
Debugging output is also on the U0R and U0T pins.


PCB
---

The PCB is designed with many options, mainly around power.  The board is designed to be powered by 2 x AAA or a 3.7v Lipo.  With 2 x AAA I found it needs a boost converter if using the ATGM GPS.  With 1 3.7v Lipo I recomend using the buck convertor.  You could use the 3.3v limiting effect of the Boost regulator, but as it is a charge doubler style is uses double the required current from the battery.


GPS
---
Expects a NMEA GPS, tested with ATGM and UBLOX MAX GPS's


Lora
----
PCB is based on a Dorji DRF1278F LORA module.

Code hould support any of the LORA modules listed here:
https://github.com/sandeepmistry/arduino-LoRa


Cameras
-------
Tested with various view angle versions of the OV2640.  Some research on other options below.


OV2640 - 1/4" color CMOS UXGA 2 megapixel image sensor
https://www.uctronics.com/download/cam_module/OV2640DS.pdf

OV3660 - 3 megapixels
Supports up to FRAMESIZE_QXGA

OV5640 - 1/4" color CMOS QSXGA 5 megapixel image sensor
https://www.uctronics.com/download/Image_Sensor/OV5640_CSP3_DS.pdf

OV7670 - 0.3Mp
Not reasearched due only 0.3Mp!

OV7725 - 
Supports up to FRAMESIZE_VGA

OV9650 -
1.3Mp
https://www.uctronics.com/index.php/ov9650-13-mega-pixels-camera-module-p-817l.html

NT99141
Supports QVGA (320x240), VGA(640x480) and HD (1280x720)


https://github.com/espressif/esp32-camera/tree/master/sensors
Included on github latest:
	#define OV2640_PID     (0x26)
	#define OV3660_PID     (0x36)
	#define OV5640_PID     (0x56)
	#define OV7670_PID     (0x76)
	#define OV7725_PID     (0x77)
	#define OV9650_PID     (0x96)
	#define NT99141_PID     (0x14)

Included in esp32 v1.04:
	#define OV2640_PID     (0x26)
	#define OV3660_PID     (0x36)
	#define OV7725_PID     (0x77)
	#define OV9650_PID     (0x96)


ESP32 Cam module
----------------
https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/
https://docs.ai-thinker.com/esp32-cam
https://docs.ai-thinker.com/_media/esp32/docs/esp32_cam_sch.pdf
https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
https://github.com/espressif/esptool/wiki/ESP32-Boot-Mode-Selection

GPIO32 controls camera power, but if you power the camera down I cant seem to re-initialise it without a reboot (which is now done through a deep sleep)


Arduino IDE
-----------
I added a custom board definition for esp32camkw.upload.speed=921600 to speed uploads.
C:\Users\WALTONK\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4


Lora Mode
---------
Code setup for:
	Implicit Header (LoRa.beginPacket(1))
	SpreadingFactor(6);
    SignalBandwidth(20.8E3);
    ErrorCodingRate4(5);
    SyncWord(0x12);
    enableCrc();
= Dave Akerman PITS mode 1
https://github.com/PiInTheSky/lora-gateway

Power Consumption
------------------
ESP32 Cam module, regulator removed and flash LED transistor removed
	44ma - Default frequency
	14ma - Reduce to 20Mhz
	14ma - Turn off BT & WIFI
	16ma - Post SPI / Lora Setup
	42ma - Camera setup
	38ma - Disable camera XCLK
	15ma - Turn off camera
	
Board Power Consumption by Frequency, with Lora, GPS and Camera attached but not initialised
160Mhz, 55ma, 80Mhz, 48ma
40Mhz, 38ma, 30Mhz, 38ma
20Mhz, 36ma, 10Mhz, 35ma
8Mhz, 34ma, 4Mhz, 34ma
2Mhz, 34ma, 1Mhz, 34ma


ToDo
----
See if we can get Lora and SD Card working together, event though they share pins.  
Stuart did here, but didnt send images: 
https://stuartsprojects.github.io/2020/08/07/ESP32CAM-as-a-Balloon-Tracker-Really!-Copy.html
https://stuartsprojects.github.io/2020/08/27/ESP32CAM-TrackerAddOn-Board.html

Send packets multiple times or auto request resends?

RTTY Code
	Lora library currently in use doesn't appear to have RTTY mode options
	https://github.com/sandeepmistry/arduino-LoRa/blob/master/src/LoRa.cpp
	PITS FSK / RTTY mode details here:
		https://github.com/PiInTheSky/pits/commit/30943ab8ea988695e4e158648f1855fd9fa906bd
		http://www.daveakerman.com/?p=2350
	Stuarts RTTY option here:
		https://github.com/StuartsProjects/Tracker-Library/blob/master/FSK_RTTY2.h
		https://github.com/StuartsProjects/Tracker-Library/blob/master/FSK_RTTY3.h

Add Lora calling frequency tx.

Add use of HabPack binary protocol

Report gps signal quality?
	Base on https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/SatelliteTracker/SatelliteTracker.ino


PCB
---
Move reset switch hole to be slightly more aligned

Thinner middle pad for buck reg to allo different footprint

Vsupply resister divider on another adc pin too, no solder jumper neeeded.








