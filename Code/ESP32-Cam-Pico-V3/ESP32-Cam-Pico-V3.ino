/*
 * ESP32 Camera Tracker
 * Written to run on ESPCam board + Lora + GPS
 * 
 * Original code by mkshrps - https://github.com/mkshrps/espcam
 * 
 * Modified Kevin Walton - https://github.com/KevWal
*/

// Specify how many times we send each image packet.
// High values take more time to send an image but less risk of missing packets
#define PACKETS 2

// transmit a telemetry packet every X SSDV packets
#define INTERLEAVE_TELEM            8

// Specify the power configuration
// BUCK assumes that all components are powered by the buck regulator, and that a 3.2v lipo cell or 3 x AAA's are used.
// BOOST assumes a 3.3v boost regulator powers the ESP32-CAM and the GPS module (DRF module ok down to 1.8v)
// NONE assumes 2 x AAA batteries power all components, and the board uses a UBLOX MAX 8C GPS module (1.65v min)
//#define BOOST
#define BUCK
//#define NONE

#ifdef BOOST
  #define CALLSIGN "KEW02"          // Set your balloon callsig, max 6 characters
  long frq = 434720000;             // Set transmit frequency, xx on my gateway
#elif defined(BUCK)
  #define CALLSIGN "KEW03"
  long frq = 434730000;             //434.738,0 on my gateway
#elif defined(NONE)
  #define CALLSIGN "KEW01"
  long frq = 434710000;             // xx on my gateway
#endif
// IR2030/1/10 433.05-434.79 MHz 10 mW e.r.p. Duty cycle limit 10% or
// IR2030/1/12 433.04-434.79 MHz 10 mW e.r.p. Channel Spacing <= 25 kHz
// 434.5 to 435 is Scandanavia repeater output

//================================================================================


// Edit Wifi ssid & password
//#define SSID "NSA"
//#define PASSWORD "orange"

#define LORA_CSS  15   // Lora CSS GPIO - will be 2,4 or 15 (Standard HSPI = 15) 
#define CORE_FREQ  20  // Mhz, reduced to save power.  20Mhz seems to be the sweet spot, 10mhz stops Lora module working

#include <SPI.h>
#include <WiFi.h>
#include <Preferences.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>
//#include "esp_pm.h"
//#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp.h"
#include <esp_camera.h>

// Forward declare cam clock stop/start and (depreciated) temperature read. 
extern "C" {
esp_err_t camera_enable_out_clock(camera_config_t *config);
void camera_disable_out_clock();
uint8_t temprature_sens_read();
}

#include <LoRa.h>      // Install from https://github.com/sandeepmistry/arduino-LoRa or "LoRa" by Sandeep Mistry in Library Manager
#include <TinyGPS++.h> // Install from http://arduiniana.org/libraries/tinygpsplus/ or TinyGPSPlus by Mikal Hart in Library Manager
#include <ssdv.h>      // Install from https://github.com/fsphil/ssdv

// Time
//#include <time.h>           // ESP32 native time library - graceful NTP server synchronization
//#include <lwip/apps/sntp.h> // ESP32 Lightweight IP SNTP client API
//#include <lwip/err.h>       // ESP32 Lightweight IP errors

// MicroSD definitions
//#include <driver/sdmmc_host.h>
//#include <driver/sdmmc_defs.h>
//#include <sdmmc_cmd.h>
//#include <esp_vfs_fat.h>

#define SerialDebug Serial    // Nice names for Serial ports
#define SerialGPS Serial2

#include "ESP32-Cam-GPS.h" // Our GPS data structure and GPS routines

// The Preferences object, used to store things in EEPROM
Preferences preferences;

// The TinyGPS++ object
TinyGPSPlus gps;

// ssdv definitions
unsigned int SSDV_quality;

#define IMG_BUFF_SIZE               128   // size of the buffer feeding SSDV process
#define LORA_BUFFER                 255   // size of the lora packet buffer

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32 //
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0 //
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22 //

// Global Variables
//long current_millis;
//long last_capture_millis = 0;
static esp_err_t cam_err = -1;
camera_config_t my_cam_config;
//bool internet_connected = false;
//struct tm timeinfo;
uint8_t imgBuff[IMG_BUFF_SIZE];
uint8_t loraBuff[LORA_BUFFER + 1];
char gpsMessage[256];
uint16_t imageID = 0;
RTC_DATA_ATTR unsigned int gpsMessageCnt = 0; // GPS Message Number for HabHub Sentance ID, saved in memory that survives a deep sleep, but not a power cycle.
uint16_t ssdvPacketCount = 0;

// Time and SSDV objects
//time_t now;
ssdv_t ssdv;

int readV() {
  int Vcc = analogReadMilliVolts(LORA_CSS)*2115/1000-540; // This one uses the calibration but was not in previous release

 /*  If you dont have analogReadMilliVolts use the below
  // analogRead() results vary depending on supply voltage source as GPIO15 has an internal pull up.
  // Divide by 1000 to allow integer maths
#if defined(BOOST)
  int Vcc = ((analogRead(LORA_CSS)*4908/1000)-4513); // tested with Supply of 2.3v to 4.8v with reg @3.3v.
  //SerialDebug.println("  Boost Regulator.");
#elif defined (BUCK)
  int raw = analogRead(LORA_CSS);
  int Vcc = (0.0035*raw*raw) - (6.0911*raw) + 5000; // tested with Supply of 2.3v to 5.0v with reg @3v.
  // y = 0.0035*x*x - 6.0911*x + 5000
  //SerialDebug.println("  Buck Regulator.");
#elif defined(NONE)
  int Vcc = ((analogRead(LORA_CSS)*1744/1000)+566); // tested with Supply of 2.4v to 4.9v with no reg
  //SerialDebug.println("  No Regulator.");
#endif
 */

  pinMode(LORA_CSS, OUTPUT); // Set pin back to an output for SPI / Lora
 
  return Vcc;
}

// Read Camera Buffer
int iread(uint8_t *buffer,int numBytes,camera_fb_t *fb, int fbIndex ){
  int bufSize = 0;
  // have we reached past end of imagebuffer
  if((fbIndex + numBytes ) < fb->len) {
    bufSize = numBytes;
  }  else  {
    bufSize = fb->len - fbIndex;
  }
  // clear the dest buffer
  memset(buffer,0,numBytes);
  memcpy(buffer,&fb->buf[fbIndex],bufSize);
  return bufSize;
}


// Read GPS data
void checkGps(){
  //SerialDebug.println("checkGps()");
  
  while (SerialGPS.available() > 0){  
    gps.encode(SerialGPS.read()); 
  }

  if (!GPS.isValid) {  // If we havent yet had a valid GPS signal, check if it is valid yet
    if (gps.location.isValid() && gps.altitude.isValid() && gps.time.isValid()){
      GPS.isValid = true; // We now have an initial valid GPS signal
    }
  }

}


// Int to Hex
char Hex(uint8_t index)
  {
    char HexTable[] = "0123456789ABCDEF";
    
    return HexTable[index];
  }


// Build HabHub Position Message including CRC
void buildPITSMessage(){
  char tempStr[50];
  
  SerialDebug.println("buildPITSMessage()");
  SerialDebug.printf("  Sats: %d, Sentences passed: %d, Sentences failed: %d\n", gps.satellites.value(), gps.passedChecksum(), gps.failedChecksum());

  gpsMessageCnt++; // Everytime we build a message, build it with a new id.

  // Get data from GPS
  if (gps.location.isValid()){
    GPS.Longitude = (float) gps.location.lng();
    GPS.Latitude = (float) gps.location.lat();
  }
  
  if (gps.altitude.isValid()){
    GPS.Altitude = (long) gps.altitude.meters();
  }
  
  if(gps.time.isValid()){
    GPS.Hours = (uint8_t) gps.time.hour();
    GPS.Minutes = (uint8_t) gps.time.minute();
    GPS.Seconds = (uint8_t) gps.time.second();
  }

  GPS.Speed = gps.speed.value();
  GPS.Direction = gps.course.deg();
  GPS.Satellites = gps.satellites.value();
  
  // If gps data is all ready
  if (gps.location.isValid() && gps.altitude.isValid() && gps.time.isValid() ){
    SerialDebug.println("  GPS location, altitude and time valid.");
  } else {
    SerialDebug.println("  Error: PITS message built, but GPS location, altitude or time not yet valid.");
  }
      
  // $$ ID, count, HH:MM:SS, lat, lng, alt, Speed, Heading, sats, mV, passedChecksum, imageID, ssdvPacketCount
  strcpy(gpsMessage, "$$");
  strcat(gpsMessage, CALLSIGN);
      
  snprintf(tempStr,sizeof(tempStr),",%d,%02d:%02d:%02d,",gpsMessageCnt,GPS.Hours,GPS.Minutes,GPS.Seconds);
  strcat(gpsMessage,tempStr);
      
  // dtostrf() converts float or double
  dtostrf(GPS.Latitude,2,6,tempStr);
  strcat(gpsMessage,tempStr);

  dtostrf(GPS.Longitude,2,6,tempStr);
  strcat(gpsMessage,",");
  strcat(gpsMessage,tempStr);

  ltoa(GPS.Altitude,tempStr,10);
  strcat(gpsMessage,",");
  strcat(gpsMessage,tempStr);

  snprintf(tempStr,40,",%d,%d,%d,%d,%d,%d,%d",GPS.Speed,GPS.Direction,GPS.Satellites,readV(),gps.passedChecksum(),imageID,ssdvPacketCount);
    // Temperature sensor not avaliable on newer hardware
  strcat(gpsMessage,tempStr);

  //KW TODO - what other data can we add here?  sat signal quality etc.
      
  int Count = strlen(gpsMessage);

  // Calculate CRC and append to message
  unsigned int CRC = 0xffff;           // Seed
  for (int i = 2; i < Count; i++){   
      CRC ^= (((unsigned int)gpsMessage[i]) << 8);
      for (uint8_t j=0; j<8; j++) {
          if (CRC & 0x8000)
              CRC = (CRC << 1) ^ 0x1021; // xPolynomial
          else
              CRC <<= 1;
      }
  }

  gpsMessage[Count++] = '*';
  gpsMessage[Count++] = Hex((CRC >> 12) & 15);
  gpsMessage[Count++] = Hex((CRC >> 8) & 15);
  gpsMessage[Count++] = Hex((CRC >> 4) & 15);
  gpsMessage[Count++] = Hex(CRC & 15);
  gpsMessage[Count++] = '\n';  
  gpsMessage[Count++] = '\0';
  
  SerialDebug.println(gpsMessage);
  }


// Main routine
void process_ssdv(camera_fb_t *fb){
  SerialDebug.println("process_ssdv()");

  int index = 0, c = 0;
  ssdvPacketCount = 0;
  
  // initialise ssdv config structure
  ssdv_enc_init(&ssdv, SSDV_TYPE_NOFEC, CALLSIGN, imageID++, SSDV_quality); //Lora inc FEC, so no point having in SSDV Lib
  // SSDV_TYPE_NORMAL or SSDV_TYPE_NOFEC // Cant turn Lora CRC off as Hab gateway assumes it is on, so no point in FEC inside SSDV
  
  // Save the new image ID in EEPROM, so that if we reboot we start from the next image ID
  preferences.putUInt("imageID", imageID);
  
  // set the output lora packet buffer for ssdv where the final ssdv packet will end up
  ssdv_enc_set_buffer(&ssdv, loraBuff);

  while(1){ // Only exits when a full image is sent or the SSDV library gives an error
    checkGps();

    SerialDebug.printf("  Sec: %d, Sats: %d, S passed: %d, S failed: %d, mV: %d.\n", 
                      (int)millis()/1000, gps.satellites.value(), gps.passedChecksum(), gps.failedChecksum(), readV());
    
    // Test to see if Location Packet is to be sent // Change to time based?
    if((ssdvPacketCount % INTERLEAVE_TELEM) == 0){
      buildPITSMessage();
      delay(10);
      LoRa.beginPacket(1);                             // initialise implicit/explicit mode and reset FIFO
      LoRa.write((const uint8_t *)gpsMessage, sizeof(gpsMessage));     // load data into FIFO
      LoRa.endPacket(0);                                // execute transmission, return once complete
      
      // todo Has the GPS position changed, if not do a reboot

/*
      // Test code for no GPS
      delay(10);
      LoRa.beginPacket(1);
      LoRa.write((const uint8_t *)"$$KEW00,217,21:20:43,52.323616,-0.708233,-1,1319,283,4,0,0,0,0*E112",67);
      LoRa.endPacket(0);
*/      
    } // Send Location Packet  
    

    // If the Camera is ok then encode and send image packets
    if (cam_err == ESP_OK) { //ESP_OK
      // Feed the SSDV encoder the image.  Encoder saves it to loraBuff 
      while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME) {
        //size_t r = fread(imageBuffer, 1, 128, fin);
        //SerialDebug.print("read into buffer");
        // read packet worth of bytes from image buffer
        index += iread(imgBuff, IMG_BUFF_SIZE, fb, index);
        SerialDebug.printf("  Feeding SSDV Encoder, index = %d\n", index);

        // ssdv =  struct, imageBuffer = buffer containing image packet, r = size
        ssdv_enc_feed(&ssdv, imgBuff, IMG_BUFF_SIZE);
        //SerialDebug.println(" bfed ");
      }
    
      if(c == SSDV_EOI) {
        SerialDebug.println("  SSDV End of Image");
        break; // Return to main()
      }
      else if(c != SSDV_OK){
        SerialDebug.println("  ERROR: SSDV Error");
        break; // Return to main()
      }

      // move lora data backwards 1 byte This seems needed for pits gateway to read it. (TT7)
      for(uint16_t i = 0; i < 256; i++) {
        loraBuff[i] = loraBuff[i+1]; //KW TODO sort warning: iteration 255u invokes undefined behavior
      }

      // Send each packet multiple times
      for(byte i = 0; i < PACKETS; i++) {
        // Lora transmit image packet
        SerialDebug.printf("  Packet sending: %d.\n", ssdvPacketCount);
        delay(10);
        LoRa.beginPacket(1);          // initialise implicit mode and reset FIFO
        LoRa.write(loraBuff, LORA_BUFFER);     // load data into FIFO
        LoRa.endPacket();             // execute transmission return once complete
      }
      
    } else {// if (cam_err == ESP_OK)
      delay(500); // dont go around the loop so quickly if we are not transmitting SSDV packets
    }

    ssdvPacketCount++; // Increment ssdvPacketCount even if the camera is not ok, otherwise we wont hit INTERLEAVE_TELEM

    if (ssdvPacketCount > 300) {
      SerialDebug.println("  ERROR: ssdvPacketCount > 300");
      break; // incase we get stuck in this loop, for example when the camera isn't on
    }
    
  } // while (1)
}

/* // Init Wifi
bool init_wifi() {
  uint8_t connAttempts = 0;
  SerialDebug.printf("  Connecting to: %s\n", SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500);
    SerialDebug.print(".");
    if (connAttempts > 10) return false;
    connAttempts++;
  }
  return true;
} */

/*
 // Init the time
void init_time() {
  char sntpservername[] = "pool.ntp.org"; // Needed to resolve ISO C++ forbids converting a string constant to 'char*' error
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, sntpservername);
  sntp_init();
*/

/*  Don't wait for time to be set
  // wait for time to be set
  timeinfo = { 0 };
  uint8_t retry = 0;
  const int retry_count = 10;

  while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
    SerialDebug.printf("Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
    delay(2000);
    time(&now);
    localtime_r(&now, &timeinfo);
  }
}
*/

/* //Init the SD Card
esp_err_t init_sdcard() {
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 1,
  };
  sdmmc_card_t *card;

  SerialDebug.print("  Mounting SD card...");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    SerialDebug.println(" successfull.");
  }  else  {
    SerialDebug.println("  failed. Error: " + String(esp_err_to_name(ret))); 
  }
  return ret;
} */

// Take and send a photo via Lora
void send_photo() {
  SerialDebug.println("send_photo()");
  camera_fb_t *fb; // Declare the variable for the pointer to the framebuffer

  if (cam_err == ESP_OK) {  // If the camera is ok
    SerialDebug.printf("  Taking picture: %d\n", imageID);
    setCpuFrequencyMhz(80); // Camera seems to need more than 40mhz to get a picture  TODO more research on lowering power here
    camera_enable_out_clock(&my_cam_config); // TODO Check result from this
    delay(1500);
    SerialDebug.printf("System Voltage: %d.\n", readV());
    fb = esp_camera_fb_get(); // Get the current frame buffer
    SerialDebug.printf("System Voltage: %d.\n", readV());
    delay(10);
    //esp_camera_deinit(); // Tried to see if this would allow us to power down the camera and power it back up again, it didn't.
    //camera_disable_out_clock(); // Saves about 10ma if we didn't turn the camera off anyway
    digitalWrite(PWDN_GPIO_NUM, HIGH); // Turn the camera power off, now need to reboot before taking next picture
    setCpuFrequencyMhz(CORE_FREQ);  // Reduce CPU frequency back to energy saving
    SerialDebug.printf("  Picture length: %d\n", fb->len);
    process_ssdv(fb);  // Main routine
    esp_camera_fb_return(fb);
  } else {
    SerialDebug.printf("WARNING: Camera Not Ok, but running process_ssdv anyway to send telemetry.\n");
    fb = 0;
    process_ssdv(fb);  // Main routine, run even if we dont have a picture as this is where we send location too.
  }
}

/* // Take and save a numbered photo to the SD Card
void save_photo_numbered() {
  SerialDebug.println("save_photo_numbered()");
  file_number++;
  SerialDebug.printf("Taking picture: %d\n", file_number);
  camera_fb_t *fb = esp_camera_fb_get();

  char *filename = (char*)malloc(21 + sizeof(int));
  sprintf(filename, "/sdcard/capture_%d.jpg", file_number);

  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    SerialDebug.printf("  File saved: %s\n", filename);
  }  else  {
    SerialDebug.printf("  ERROR: Could not open file: %s\n", filename);
  }
  fclose(file);
  esp_camera_fb_return(fb);
  free(filename);
} */

/* // take and save a dated photo to the SD Card
void save_photo_dated(){

  char strftime_buf[64];
  
  SerialDebug.println("save_photo_dated()");
  SerialDebug.println("  Taking picture...");
  camera_fb_t *fb = esp_camera_fb_get();

  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);

  char *filename = (char*)malloc(21 + sizeof(strftime_buf));
  sprintf(filename, "/sdcard/capture_%s.jpg", strftime_buf);

  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    SerialDebug.printf("  File saved: %s\n", filename);
  }  else  {
    SerialDebug.printf("  ERROR: Could not save file: %s\n", filename);
  }
  fclose(file);
  esp_camera_fb_return(fb);
  free(filename);
} */

/* // Call save_photo dated() or numbered()
void save_photo() {
  if (timeinfo.tm_year < (2016 - 1900) || internet_connected == false) { // if no internet or time not set
    save_photo_numbered(); // filenames in numbered order
  } else {
    save_photo_dated(); // filenames with date and time
  }
} */

// Setup the camera
esp_err_t camera_init() {
    //power up the camera if PWDN pin is defined
    if(PWDN_GPIO_NUM != -1) {
        pinMode(PWDN_GPIO_NUM, OUTPUT);
        digitalWrite(PWDN_GPIO_NUM, LOW);
    }

    setCpuFrequencyMhz(80); // Camera seems to need more than 40mhz  TODO more research on saving power here
    delay(10);
    
    esp_err_t err = esp_camera_init(&my_cam_config);  //initialize the camera

    delay(10);
    setCpuFrequencyMhz(CORE_FREQ);

    return err;
}


//#######################################################################################
// Main setup routine, runs once
void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  setCpuFrequencyMhz(CORE_FREQ); //reduce from 160 to save power
  
  SerialDebug.begin(115200);
  SerialDebug.println("\r\nsetup()");
  SerialDebug.flush();

  //while (1) {
    SerialDebug.printf("System Voltage: %d.\n", readV());
    delay(1000);
  //}

  // Help us to confirm we have the right config for the right board!
#if defined(BOOST)
  SerialDebug.println("  Boost Regulator."); SerialDebug.flush();
#elif defined (BUCK)
  SerialDebug.println("  Buck Regulator."); SerialDebug.flush();
#elif defined(NONE)
  SerialDebug.println("  No Regulator."); SerialDebug.flush();
#else
  SerialDebug.println("  ERROR: Incorrect regulator config"); SerialDebug.flush();
  delay(30000);
  ESP.restart();
#endif

/*
  // Test Frequency vs power
  int freq[] = {160,80,40,30,20,18, 16, 14, 12, 10, 8, 6, 4, 2, 1};
  for (int i = 0; i<sizeof freq/sizeof freq[0]; i++) {
    SerialDebug.printf("Frequency set to %d\n", freq[i]);
    setCpuFrequencyMhz(freq[i]);
    delay(5000);
  }
*/
  
  // GPIO4 is the Flash LED, make sure it is off
  // Flash LED Transistor now removed as GPIO used for GPS_TXD
  // pinMode(4, OUTPUT);
  // digitalWrite(4, LOW);

  // Dont think this works in Arduino
  //esp_pm_config_esp32_t pm_config;
  //pm_config.max_freq_mhz = 80;
  //pm_config.min_freq_mhz = 13;
  //pm_config.light_sleep_enable = true;
  //ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );

  // Turn off adc, wifi and Bluetooth for flight mode
  SerialDebug.println("Turning off WIFI and BT."); SerialDebug.flush();
  WiFi.mode(WIFI_OFF);
  btStop();

  // Recomended additional power savings
  esp_wifi_stop(); // May caused a panic, comment out if it does
  esp_bt_controller_disable();
  //adc_power_off(); // Need adc for reading Vcc
  
  //internet_connected = false;

  // Flash onboard red LED
  pinMode(33, OUTPUT);
  for (int f = 0; f < 2; f++){   
    digitalWrite(33, LOW);
    delay(500);
    digitalWrite(33, HIGH);
    delay(500);
  }

  SerialDebug.printf("System Voltage: %d.\n", readV());
    
  // Setup GPS
  SerialGPS.begin(9600,SERIAL_8N1,4,2,false);  // GPS 3.3v serial
  delay(1000);
  setGPS_AirBorne();  // Called for ATGM or UBLOX, not needed for ATGM
  
  // TODO Disable un needed GPS sentances

/*
  // GPS Test
  while (1) {
    if (SerialGPS.available()){  
      SerialDebug.write(SerialGPS.read()); 
    }
    if (SerialDebug.available()){
      SerialGPS.write(SerialDebug.read());
    }
  }
*/

  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a (mac 15 char) namespace name to prevent key name collisions. 
  // We will open storage in RW-mode (second parameter has to be false).
  preferences.begin("my-app", false);
  
  // Get the picture counter value, if the key does not exist, return a default value of 0
  // Key name is limited to 15 chars.
  imageID = preferences.getUInt("imageID", 0);
   
/*  Dont bother with WiFi
  // Setup WiFi and time
  if (init_wifi()) { // Connected to WiFi
    internet_connected = true;
    SerialDebug.println("  WiFi connected.");
    init_time();
    time(&now);
    setenv("TZ", "GMT0BST,M3.5.0/01,M10.5.0/02", 1);
    tzset();
  }
*/
  SerialDebug.printf("System Voltage: %d.\n", readV());
  
  // Set up Lora Module SPI and Lora Library
  SPI.begin(14,12,13,LORA_CSS);    //  s(l)ck = gpio14, miso = gpio12, mosi = gpio13, (n)ss = gpio15
  SPI.setFrequency(100000);       // Originally 4mhz
  LoRa.setPins(LORA_CSS,-1,-1);    // (n)ss = 15, reset = -1, dio0 = -1
  //KW TODO research void LoRa.setSPI(SPIClass& spi);
  //KW TODO research void LoRa.setSPIFrequency(uint32_t frequency);

  if(!LoRa.begin(frq)){
    SerialDebug.println("ERROR: LoRa module not detected, going to reboot in 5 seconds.");
    delay(5000);
    preferences.end();
    ESP.restart(); // If we can't transmit at least our position, no point carrying on.
  }
  else {
    SerialDebug.println("LoRa module detected OK");
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(20.8E3);
    LoRa.setCodingRate4(5);
    //LoRa.setPreambleLength(preambleLength);
    LoRa.setSyncWord(0x12);
    LoRa.enableCrc();
  }

  SerialDebug.printf("System Voltage: %d.\n", readV());
  
  // Get a GPS location
  SerialDebug.println("Wait a while for a GPS Signal, check we get some GPS sentances.");
  while (!GPS.isValid && (millis() <= 20000) ) {
    checkGps();
    if (millis() % 5000 == 0) {
      SerialDebug.printf("  Second: %d, Sats: %d, Sentences passed: %d, Sentences failed: %d\n", 
                      (int)millis()/1000, gps.satellites.value(), gps.passedChecksum(), gps.failedChecksum());
      delay(5);
    }
  }
  if (GPS.isValid) {
    SerialDebug.printf("  Initial GPS Signal valid.\n  Sats: %d, Sentences passed: %d, Sentences failed: %d\n\n", 
                        gps.satellites.value(), gps.passedChecksum(), gps.failedChecksum());
  } else {
    SerialDebug.printf("  WARNING: No Initial GPS Signal.\n  Sats: %d, Sentences passed: %d, Sentences failed: %d\n\n", 
                        gps.satellites.value(), gps.passedChecksum(), gps.failedChecksum());
  }

  // Setup config structure for camera
  my_cam_config.ledc_channel = LEDC_CHANNEL_0;
  my_cam_config.ledc_timer = LEDC_TIMER_0;
  my_cam_config.pin_d0 = Y2_GPIO_NUM;
  my_cam_config.pin_d1 = Y3_GPIO_NUM;
  my_cam_config.pin_d2 = Y4_GPIO_NUM;
  my_cam_config.pin_d3 = Y5_GPIO_NUM;
  my_cam_config.pin_d4 = Y6_GPIO_NUM;
  my_cam_config.pin_d5 = Y7_GPIO_NUM;
  my_cam_config.pin_d6 = Y8_GPIO_NUM;
  my_cam_config.pin_d7 = Y9_GPIO_NUM;
  my_cam_config.pin_xclk = XCLK_GPIO_NUM;
  my_cam_config.pin_pclk = PCLK_GPIO_NUM;
  my_cam_config.pin_vsync = VSYNC_GPIO_NUM;
  my_cam_config.pin_href = HREF_GPIO_NUM;
  my_cam_config.pin_sscb_sda = SIOD_GPIO_NUM;
  my_cam_config.pin_sscb_scl = SIOC_GPIO_NUM;
  my_cam_config.pin_pwdn = PWDN_GPIO_NUM;
  my_cam_config.pin_reset = RESET_GPIO_NUM;
  my_cam_config.xclk_freq_hz = 20000000;  // TODO can we reduce this to save power? Default 20000000
  my_cam_config.pixel_format = PIXFORMAT_JPEG;
 
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    SerialDebug.println("PS RAM Found.");
    my_cam_config.frame_size = FRAMESIZE_UXGA;
    my_cam_config.jpeg_quality = 10;
    my_cam_config.fb_count = 2;
  } else {
    SerialDebug.println("WARNING: PS RAM Not Found.");
    my_cam_config.frame_size = FRAMESIZE_SVGA;
    my_cam_config.jpeg_quality = 12;
    my_cam_config.fb_count = 1;
  }

  // Alternate between image sizes
  if (imageID % 3 == 0) {
    // Overide the above, https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h
    my_cam_config.frame_size = FRAMESIZE_XGA; // QVGA = 320x240, VGA = 640x480, SVGA = 800x600 (Not div 16!), XGA = 1024x768, SXGA = 1280x1024, UXGA = 1600x1200
    my_cam_config.jpeg_quality = 10;  // Quality of JPEG output. 0-63, lower means higher quality. TODO Should we take the highest quality and let the SSDV encoder shrink it?
    my_cam_config.fb_count = 1; // If more than one, i2s runs in continuous mode. Use only with JPEG
    SSDV_quality = 3;     // 0-7 corresponding to JPEG quality: 13, 18, 29, 43, 50, 71, 86 and 100.  
                          // Default 4, Above 4 the improvements were not detectable 
                         // http://tt7hab.blogspot.com/2017/03/ssdv-slow-scan-digital-video.html
  } else {
    my_cam_config.frame_size = FRAMESIZE_VGA;
    my_cam_config.jpeg_quality = 10;
    my_cam_config.fb_count = 1;
    SSDV_quality = 4;  // Slightly higher quality for smaller VGA images
  }

  // XGA picture needs 2.75v to take without colour errors
  // ATGM GPS needs at least 2.7v to reliably start
#if defined(BOOST)
  if (readV() > 2650) { // Boost regulator with 2 x AA's
#elif defined (BUCK)
  if (readV() > 3000) { // Buck regulator with 3 x AA's 
#elif defined (NONE)
  if (readV() > 2750) { // No reg with 2 x AA's
#endif
    cam_err = camera_init(); // If this fails, we seem to jump straight to main().
    if (cam_err != ESP_OK) {
      SerialDebug.printf("ERROR: Camera init failed with error 0x%x\n", cam_err);
      digitalWrite(PWDN_GPIO_NUM, HIGH); // might as well turn camera of
    } else {
      SerialDebug.println("Camera initialised OK.");
      //esp_camera_deinit(); // TODO try this before power down, see if it means we can re init?
      //digitalWrite(PWDN_GPIO_NUM, HIGH); // Turn the camera power off // Cant do this as it wont re initialise
      camera_disable_out_clock(); // Save power by pausing camera.  Saves ~10ma
    }
  } else {
    SerialDebug.printf("ERROR: Vcc not high enough for picture size, not starting camera\n");
    cam_err = -1;
    digitalWrite(PWDN_GPIO_NUM, HIGH); // Turn camera off
  }

/*  We can't currently have the SD Card and the Lora module active without another pin for Lora SS
  // SD card init
  esp_err_t card_err;
  card_err = init_sdcard();
  if (card_err != ESP_OK) {
    SerialDebug.printf("SD Card init failed with error 0x%x", card_err);
    return;
  }
*/

  SerialDebug.printf("System Voltage: %d.\n", readV());

} // setup()


//#######################################################################################
// Main Loop
void loop()
{
  delay(1000);
  SerialDebug.println("main()");
  SerialDebug.flush();

  // Needed as if camera init fails in setup() we skip straight to loop()
  if (cam_err != ESP_OK) {
    SerialDebug.printf("ERROR: Camera init failed with error 0x%x\n", cam_err);
    digitalWrite(PWDN_GPIO_NUM, HIGH); // turn camera of!
  }

  send_photo();

  // Deep Sleep reset here to reset camera.  Allows us to turn the camera off as soon as we take a picture and then reboot to reinitialise it.
  // Dont reboot if below a certain voltage as we wont make it back (Brownout WDT kicks in before we can turn it off and we reboot loop).  
  // ESP32 module will run to about 2.15v if not rebooted
#if defined (BUCK) // If Buck dont reboot below this voltage
  if (readV() > 2700) {
#else
  if (readV() > 2500) { // If 2 x AA / AAA dont reboot below this voltage
#endif
    periph_module_reset(PERIPH_I2C0_MODULE);
    esp_sleep_enable_timer_wakeup(15 * 1000000l); // 1000000 us per Second
    esp_deep_sleep_start(); // Go into deep sleep for a legth of time to save power, and then reboot.
    ESP.restart(); // Should never get here
  } else {
    cam_err = -1; // Dont take any more pictures
    SerialDebug.printf("ERROR: Voltage below 2500mV, dont reboot or else we may not restart\n");
    digitalWrite(PWDN_GPIO_NUM, HIGH); // turn camera of!
  }

}
