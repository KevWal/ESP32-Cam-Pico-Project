#define _MAIN_
#include <Arduino.h>
#include <esp_camera.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa_LIB.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include "main.h"

// Time
#include <time.h>
#include <lwip/err.h>
#include <lwip/apps/sntp.h>
// MicroSD
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_defs.h>
#include <sdmmc_cmd.h>
#include <esp_vfs_fat.h>
#include <ssdv.h>

#define LORA_CSS  15   // will be 2,4 or 15 (Standard HSPI = 15) 

// The TinyGPS++ object
TinyGPSPlus gps;

// Edit ssid, password, capture_interval:
const char* ssid = "NSA";
const char* password = "orange";
int capture_interval = 15000; // microseconds between captures

long current_millis;
long last_capture_millis = 0;
static esp_err_t cam_err;
static esp_err_t card_err;
char strftime_buf[64];
int file_number = 0;
bool internet_connected = false;
struct tm timeinfo;
time_t now;

int t,n=0;
long frq = 434500000;
int counter = 0;
int loraBufferSize = 255;
bool firstRun = false;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
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
#define PCLK_GPIO_NUM     22

// ssdv definitions
#define JPEG_RESOLUTION             4                                // 0-8 corresponfing to 320x240, 352x288, 640x480, 800x480, 1024x768, 1280x960, 1600x1200, 2048x1536, 2592x1944
#define JPEG_QUALITY                3                                // 0-16 corresponding to 96.7, 93.7, 87.3, 81.2, 74.8, 68.6, 62.3, 56.2, 50.0, 44.4, 39.9, 36.3, 33.2, 30.7, 28.5, 26.6, 25.8
#define SSDV_QUALITY                4                                // 0-7 corresponding to JPEG quality: 13, 18, 29, 43, 50, 71, 86 and 100
#define IMG_BUFF_SIZE               128                               // size of the buffer feeding SSDV process 
#define INTERLEAVE_TELEM            5                               // transmit a telemetry packet every X SSDV packets
#define LORA_BUFFER 255;

static const char callsign[] = "KEW01";                    // maximum of 6 characters
uint8_t jpgRes                     = JPEG_RESOLUTION;
uint8_t jpgQuality                 = JPEG_QUALITY;
uint32_t imgSize                   = 0;
uint32_t imgCount                  = 1;
uint8_t ssdvQuality                = SSDV_QUALITY;
uint8_t imgBuff[IMG_BUFF_SIZE];
ssdv_t ssdv;
uint8_t loraBuff[256];
char tempStr[40];
char gpsMessage[256];

bool gpsSerialActive = false;

int imageID = 0;


int iread(uint8_t *buffer,int numBytes,camera_fb_t *fb, int fbIndex ){

  int bufSize = 0;
  // have we reached past end of imagebuffer
  if((fbIndex + numBytes ) < fb->len){
  
    bufSize = numBytes;
  }
  else{

    bufSize = fb->len - fbIndex;
  }
  // clear the dest buffer
  memset(buffer,0,numBytes);
  memcpy(buffer,&fb->buf[fbIndex],bufSize);
  return bufSize;
}


void checkGps(){
  //delay(1000);
  
  gpsSerialActive = false;
  while (Serial2.available() > 0){
    char dat = Serial2.read();
    gpsSerialActive = true;
    gps.encode(dat);
  }
  //if(gpsSerialActive){
  //  Serial.println("  gps serial active.");
  //}

  //  Serial.println("\n decoded GPS");
  
  if(gps.location.isUpdated()){

    if (gps.location.isValid()){
      GPS.Longitude = (float) gps.location.lng();
      GPS.Latitude = (float) gps.location.lat();
    }

    if (gps.altitude.isValid()){
      GPS.Altitude = (long) gps.altitude.meters();
    }

    // if gps data is ready
    if (gps.location.isValid() && gps.altitude.isValid()){
      GPS.isValid = true;
      //Serial.println("values valid");
    }

    
    if(gps.time.isValid() && gps.time.isUpdated() ){
    GPS.Hours = (uint8_t) gps.time.hour();
    GPS.Minutes = (uint8_t) gps.time.minute();
    GPS.Seconds = (uint8_t) gps.time.second();
    }

    /*
    if(GPS.isValid ){
      // get course and distance if we have a remote tracker

      GPS.courseTo =gps.courseTo(GPS.Latitude,GPS.Longitude,remote_data.latitude,remote_data.longitude);
      GPS.distancem = gps.distanceBetween(GPS.Latitude,GPS.Longitude,remote_data.latitude,remote_data.longitude);
      //remote_data.cardinalCourseTo = gps.cardinal(remote_data.courseTo);
    }
    */

  }
    GPS.Satellites = (unsigned int) gps.satellites.value();
    GPS.failedCS = (unsigned int) gps.failedChecksum();
    Serial.print("  GPS Satellites: ");
    Serial.print(GPS.Satellites);
    Serial.print(" : sentances failed chk: ");
    Serial.println(GPS.failedCS);
}

char Hex(uint8_t index)
  {
    char HexTable[] = "0123456789ABCDEF";
    
    return HexTable[index];
  }


void buildPITSMessage(){
      
      //char gpsMessage[100];
      //char tempStr[40];
      // $$ ID, count, HH:MM:SS,Lat,lng,alt,Speed,Heading,sats,int temp,ext temp,pressure,humidity 
      strcpy(gpsMessage,"$$");
      strcat(gpsMessage,callsign);
      
      snprintf(tempStr,sizeof(tempStr),",%d,%02d:%02d:%02d,",GPS.flightCount,GPS.Hours,GPS.Minutes,GPS.Seconds);
      strcat(gpsMessage,tempStr);
      // convert using dtostrf() will convert float or double
      dtostrf(GPS.Latitude,2,6,tempStr);
      strcat(gpsMessage,tempStr);

      dtostrf(GPS.Longitude,2,6,tempStr);
      strcat(gpsMessage,",");
      strcat(gpsMessage,tempStr);

      ltoa(GPS.Altitude,tempStr,10);
      strcat(gpsMessage,",");
      strcat(gpsMessage,tempStr);

      snprintf(tempStr,40,",%d,%d,%d,%d,%d,%d,%d",GPS.Speed,GPS.Direction,GPS.Satellites,0,0,0,0);
      strcat(gpsMessage,tempStr);
      
      int Count = strlen(gpsMessage);

      unsigned int CRC = 0xffff;           // Seed
      //unsigned int xPolynomial = 0x1021;
      // do CRC calc and append to message
      for (int i = 2; i < Count; i++)
      {   
          CRC ^= (((unsigned int)gpsMessage[i]) << 8);
          for (int j=0; j<8; j++)
          {
              if (CRC & 0x8000)
                  CRC = (CRC << 1) ^ 0x1021;
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
  
      //Serial.println(gpsMessage);
  }

  

void process_ssdv(camera_fb_t *fb){

  int index=0,c = 0,ssdvPacketCount=0;

  // initialise ssdv config structure
  ssdv_enc_init(&ssdv, SSDV_TYPE_NOFEC, (char *)callsign, imageID++, ssdvQuality);
  // set the output lora packet buffer for ssdv where the final ssdv packet will end up
  ssdv_enc_set_buffer(&ssdv, loraBuff);
  Serial.print("Sending Image: length = ");
  Serial.println(fb->len);

  while(1){

    checkGps();
    if(GPS.isValid){
      // test to see if GPS is to be sent
      if((ssdvPacketCount % INTERLEAVE_TELEM) == 0){
        buildPITSMessage();
        LoRa.beginPacket(1);          // initialise implicit/explicit mode and reset FIFO
        LoRa.write((const uint8_t *)gpsMessage,255);     // load data into FIFO
        LoRa.endPacket();             // execute transmission return once complete

      Serial.print("lat/lng ");
      Serial.print(GPS.Latitude);
      Serial.print("/");
      Serial.println(GPS.Longitude);


      }  
    }
   
  
    while((c = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
    {
        //size_t r = fread(imageBuffer, 1, 128, fin);
        //Serial.print("read into buffer");
        // read packet worth of bytes from image buffer
        index += iread(imgBuff, IMG_BUFF_SIZE,fb,index);
        Serial.print("index = ");
        Serial.println(index);
        
        // ssdv =  struct, imageBuffer = buffer containing image packet, r = size
        ssdv_enc_feed(&ssdv, imgBuff, IMG_BUFF_SIZE);
        //Serial.println(" bfed ");
    }
    
    if(c == SSDV_EOI)
    {
        Serial.println("ssdv EOI");
        break;
    }
    else if(c != SSDV_OK)
    {
        Serial.println("ssdv Error");
        break;
    }

    // move lora data backwrds 1 byte This seems needed for pits gateway to read it. (TT7)
    for(uint16_t i = 0; i < 256; i++) {
      loraBuff[i] = loraBuff[i+1];
    }

    
    // lora transmit
    Serial.print("  Sending packet: ");
    Serial.println(ssdvPacketCount);
    delay(10);
    LoRa.beginPacket(1);          // initialise implicit/explicit mode and reset FIFO
    LoRa.write(loraBuff,255);     // load data into FIFO
    LoRa.endPacket();             // execute transmission return once complete
    ssdvPacketCount++;
    delay(10);

    //fwrite(pkt, 1, SSDV_PKT_SIZE, fout);
    
  }
}
  


void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  internet_connected = false;
  Wire.begin();//Change to Wire.begin() for non ESP.
  // GPS
  Serial2.begin(9600,SERIAL_8N1,2,16,false);

// turn off wifi and BLE for flight mode
  WiFi.mode(WIFI_OFF);
  btStop();

  // set up lora SPI
  //SPI.begin(14,12,13,15);    // wrks for ESP32 DOIT board 
  SPI.begin(14,12,13,LORA_CSS);    // use GPIO04 - HS2 DATA1 as NSS 
  SPI.setFrequency(1000000);       // Was 4000000, but LoRa was doing weird stuff
  LoRa.setPins(LORA_CSS,-1,-1); 

  if(!LoRa.begin(frq)){
    Serial.println("Lora not detected");
  } else {
    Serial.println("LORA OK");
  }

  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(20.8E3);
  LoRa.setCodingRate4(5);
    
  //LoRa.setPreambleLength(preambleLength);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();

/*
  if (init_wifi()) { // Connected to WiFi
    internet_connected = true;
    Serial.println("Internet connected");
    init_time();
    time(&now);
    setenv("TZ", "GMT0BST,M3.5.0/01,M10.5.0/02", 1);
    tzset();
  }
*/

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
// overide the above for testing
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // camera init
  cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", cam_err);
    return;
  }
  
/*
  // SD camera init
  card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    return;
  }
*/
}

bool init_wifi()
{
  int connAttempts = 0;
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
    if (connAttempts > 10) return false;
    connAttempts++;
  }
  return true;
}

void init_time()
{
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
  // wait for time to be set
  time_t now = 0;
  timeinfo = { 0 };
  int retry = 0;
  const int retry_count = 10;
/*
  while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
    Serial.printf("Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
    delay(2000);
    time(&now);
    localtime_r(&now, &timeinfo);
  }

*/
}

void init_sdcard()
{
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 1,
  };
  sdmmc_card_t *card;

  Serial.println("Mounting SD card...");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    Serial.println("SD card mount successfully!");
  }  else  {
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
  }
}

void save_photo_numbered()
{
  file_number++;
  Serial.print("Taking picture: ");
  Serial.println(file_number);
  camera_fb_t *fb = esp_camera_fb_get();
  process_ssdv(fb);

/*
  char *filename = (char*)malloc(21 + sizeof(int));
  sprintf(filename, "/sdcard/capture_%d.jpg", file_number);

  Serial.println(filename);
  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    Serial.printf("File saved: %s\n", filename);
  }  else  {
    Serial.println("Could not open file");
  }
  fclose(file);
*/

  // send out streamed image

  esp_camera_fb_return(fb);

  // free(filename);
}

void save_photo_dated()
{
  Serial.println("Taking picture...");
  camera_fb_t *fb = esp_camera_fb_get();

  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);

  char *filename = (char*)malloc(21 + sizeof(strftime_buf));
  sprintf(filename, "/sdcard/capture_%s.jpg", strftime_buf);

  Serial.println(filename);
  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    Serial.printf("File saved: %s\n", filename);

  }  else  {
    Serial.println("Could not open file");
  }
  fclose(file);
  esp_camera_fb_return(fb);
  free(filename);
}

void save_photo()
{
  if (timeinfo.tm_year < (2016 - 1900) || internet_connected == false) { // if no internet or time not set
    save_photo_numbered(); // filenames in numbered order
  } else {
    save_photo_dated(); // filenames with date and time
  }
}

void loop()
{
  if(!firstRun){
    firstRun = true;
    save_photo();
    
  }
  current_millis = millis();
  if (current_millis - last_capture_millis > capture_interval) { // Take another picture
    last_capture_millis = millis();
    save_photo();
  }


}
