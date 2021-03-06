/*
 * ESP32 Camera Tracker
 * Written to run on ESPCam board + Lora + GPS
 * 
 * Original code by mkshrps - https://github.com/mkshrps/espcam
 * 
 * Modified Kevin Walton - https://github.com/KevWal
*/

//#ifdef _MAIN_

struct TGPS
{
  uint32_t time; 
  uint8_t Hours, Minutes, Seconds;
  unsigned long SecondsInDay;         // Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  unsigned int failedCS;
  int Speed;
  int Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
  char callSign[12];
  int flightCount;
  bool isValid = false;
}  GPS;


/* 
#else

extern struct TGPS
{
  uint32_t  time;
  uint8_t Hours, Minutes, Seconds;
  unsigned long SecondsInDay;         // Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  unsigned int failedCS;
  int Speed;
  int Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
  char callSign[12];
  int flightCount;
  bool isValid ;

} GPS;


#endif

*/

//void setPitsMode(int mode);
//void setCustomLoRaMode(int sf,int cr,float bw);







