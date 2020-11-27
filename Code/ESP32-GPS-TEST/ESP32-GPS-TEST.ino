

#define SerialDebug Serial    // Nice names for Serial ports
#define SerialGPS Serial2

void gps_reset()
{
  SerialGPS.flush();
  SerialGPS.write(0xFF);
  delay(500);
  SerialGPS.println(F("$PCAS10,3*1F"));
  SerialGPS.flush();
}

void setup() {

  SerialDebug.begin(115200);
  SerialDebug.println("\r\nsetup()");
  SerialDebug.flush();

  SerialGPS.begin(9600,SERIAL_8N1,4,2,false);

  while (millis() < 10000) {
    while (SerialGPS.available() > 0){  
      SerialDebug.write(SerialGPS.read()); 
    }
  }

  SerialDebug.println("\r\ngps_reset()");
  gps_reset();

  while (millis() < 20000) {
    while (SerialGPS.available() > 0){  
      SerialDebug.write(SerialGPS.read()); 
    }
  }

  SerialDebug.println("\r\nend.");
}

void loop() {

}
