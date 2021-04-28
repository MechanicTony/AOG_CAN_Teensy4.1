
#define GPS Serial3

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));


//**************************************************************

void GPS_setup()
{
  GPS.begin(57600);
}

//**************************************************************

void Forward_GPS()
{
  while (GPS.available())
  {
    char c = GPS.read();
    if (nmea.process(c)) {
      (nmea.getSentence());
      //Serial.write(nmeaBuffer, sizeof(nmeaBuffer));
      //Serial.write(10);
        Udp.beginPacket(remote, AOGPort);
        Udp.write(nmeaBuffer, sizeof(nmeaBuffer));
        Udp.println();
        Udp.endPacket();
  }
 }
}

//**************************************************************

void Forward_Ntrip()
{

//Check for UDP Packet (Ntrip 2233)

    int NtripSize = NtripUdp.parsePacket();
    
    if (NtripSize) {
        NtripUdp.read(NtripData, sizeof(NtripData));
        //Serial.print("Ntrip Data ="); 
        //Serial.write(NtripData, sizeof(NtripData)); 
        //Serial.write(10);
        GPS.write(NtripData, sizeof(NtripData)); 
  }
}
    

     
   
