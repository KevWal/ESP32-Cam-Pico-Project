// GPS Struc and routines


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

void sendUBX(const uint8_t *MSG, uint8_t len)
{
  
  SerialGPS.flush();
  SerialGPS.write(0xFF);
  delay(500);
  for (int i = 0; i < len; i++) {
    //Serial.write(MSG[i]);
    SerialGPS.write(pgm_read_byte(MSG + i));
  }
}

boolean getUBX_ACK(const uint8_t *MSG)
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  uint32_t startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = pgm_read_byte(MSG + 2); // ACK class
  ackPacket[7] = pgm_read_byte(MSG + 3); // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      SerialDebug.printf("  UBX_ACK success.\n");
      return true;
    }
    // Timeout if no valid response in UBX_TIMEOUT seconds
    if (millis() - startTime > 2000) {
      SerialDebug.printf("  UBX_ACK timeout.\n");
      return false;
    }
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0; // Reset and look again, invalid order, likley to end in a timeout
        //DEBUG_PRINTLN(F("  UBX_ACK bad order"));
      }
    }
  }
}


void setGPS_AirBorne() // < 1g
{
  //int gps_set_sucess = 0;
  const static uint8_t PROGMEM setdm6[44] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };

  SerialDebug.printf("setGPS_airborne()\n");
  
  // Dont do this as a while, otherwise we could get stuck here until wdt resets.
  // while(!gps_set_sucess)
  //{
  //sendUBX(setdm6, sizeof(setdm6) / sizeof(uint8_t));
  sendUBX(setdm6, 44);
  //getUBX_ACK(setdm6);
  //}
}

void gps_reset()
{
  //int gps_set_sucess = 0;
  const static uint8_t PROGMEM set_reset[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  
  SerialDebug.printf("gps_reset()\n");
  sendUBX(set_reset, 12);
  getUBX_ACK(set_reset);
}


void gps_set_max()
{
  //int gps_set_sucess = 0;
  const static uint8_t PROGMEM setMax[10] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
  
  SerialDebug.printf("gps_max()\n");
  
  // Dont do this as a while, otherwise we could get stuck here until wdt resets.
  // while (!gps_set_sucess)
  //{
  //sendUBX(setMax, sizeof(setMax) / sizeof(uint8_t));
  sendUBX(setMax, 10);
  getUBX_ACK(setMax);
  //}
}


// Set minimum performance mode to save power whilst we transmit.
void gps_set_min()
{
  //int gps_set_sucess = 0;
  // Cyclic 1 second, from here: https://ukhas.org.uk/guides:ublox_psm
  const static uint8_t PROGMEM setPSM[10] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
  
  SerialDebug.printf("gps_min()\n");
  
  // Dont do this as a while, otherwise we could get stuck here until wdt resets.
  // while (!gps_set_sucess)
  //{
  //sendUBX(setPowerSaveMode, sizeof(setPowerSaveMode) / sizeof(uint8_t));
  sendUBX(setPSM, 10);
  getUBX_ACK(setPSM);
  //}
}
