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

Launch
------

12.4g - Tracker inc 180deg Camera, guitar wire GPS & LoRa Aeril
25g - 3 x AAA soldered together with wire - lasted 14 hours in freezer
21g - 2 x AAA in holder with wire
40g - 2 x AA in holder with wire
14g - 450Ma LiPo

LiPo Version, 26.5g total weight
1 x Qualtex, 72Ltr Helium, 10g free lift, 0.120m3 unstretched volume, 6,400m burst, 1.7m/sec ascent, 60 mins to burst
2 x Qualtex, 58Ltr Helium each, 10g free lift each, 0.120m3 unstretched volume, 8,400m burst, 1.9m/sec ascent, 75 mins to burst

AA Version, 52.5g total weight
1 x 100g Pawan, 200ltr Helium, 50g free lift, 20,325m burst, 2.9m/sec ascent, 117mins to burst

AAA Version, 12.5g + 25g = 38g total weight
1 x Qualtex, 70Ltr Helium, 10g free lift, 0.120m3 unstretched volume, 4,900m burst, 1.7m/sec ascent, 48 mins to burst
2 x Qualtex, 62Ltr Helium each, 10g free lift each, 0.120m3 unstretched volume, 7,500m burst, 1.9m/sec ascent, 65 mins to burst
2 x Chineese, 58Ltr Helium each, 6g free lift each, 0.120m3 unstretched volume, 7,640m burst, 1.9m/sec ascent, 65 mins to float!



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

OV2640 - 1/4" color CMOS UXGA (1600x1200) 2 megapixel image sensor
Used XGA = 1024x768 or VGA = 640x480
https://www.uctronics.com/download/cam_module/OV2640DS.pdf

OV3660 - 3 megapixels
Supports up to FRAMESIZE_QXGA (2048x1536)
https://github.com/espressif/esp32-camera/files/3783752/OV3660_CSP3_DS_1.3_sida.pdf

OV5640 - 1/4" color CMOS QSXGA 5 megapixel image sensor
Supports up to FRAMESIZE_QSXGA (2560x1920)
https://www.uctronics.com/download/Image_Sensor/OV5640_CSP3_DS.pdf

OV7670 - 0.3Mp
Not reasearched due only 0.3Mp!

OV7725 - 
Supports up to FRAMESIZE_VGA (640x480), doesnt support JPEG

OV9650 - Not supported in main repo.
https://www.uctronics.com/index.php/ov9650-13-mega-pixels-camera-module-p-817l.html

NT99141
Supports QVGA (320x240 xskip or crop), VGA(640x480 xskip or crop) and HD (1280x720)


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

Increase PCB string holes size, and 4 corner holes to allow vertical down suspension

Lengthen PCB

2 x AA battery holders?

Different boost converter







