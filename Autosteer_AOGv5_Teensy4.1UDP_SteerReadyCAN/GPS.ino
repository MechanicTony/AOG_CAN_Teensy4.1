
char rxbuffer[512];   //Extra serial rx buffer
char txbuffer[512];   //Extra serial tx buffer

char nmeaBuffer[150];
int count=0;
bool stringComplete = false;

int test = 0;

//**************************************************************

void GPS_setup()
{
  SerialGPS.begin(baudGPS);
  SerialGPS.addMemoryForRead(rxbuffer, 512);
  SerialGPS.addMemoryForWrite(txbuffer, 512);

  //the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
      
}

//**************************************************************

void Forward_GPS()
{ 
  IMU_currentTime = systick_millis_count;
  
  if (useBNO08x)
    {  
      if (isTriggered && IMU_currentTime - IMU_lastTime >= IMU_DELAY_TIME)
      {
        //Load up BNO08x data from gyro loop ready for takeoff
        imuHandler();

        //reset the timer 
        isTriggered = false;
      }      
    }

  if (useCMPS)
    { 
      if (isTriggered && IMU_currentTime - gpsReadyTime >= CMPS_DELAY_TIME)
      {
        imuHandler(); //Get data from CMPS (Heading, Roll, Pitch) and load up ready for takeoff
        BuildPANDA(); //Send Panda

        //reset the timer 
        isTriggered = false;
      }
    }
  
  while (SerialGPS.available())
  {
    parser << SerialGPS.read();
    if(stringComplete == true)break;
   } 
  stringComplete == false;
}

//**************************************************************

void Forward_Ntrip()
{

//Check for UDP Packet (Ntrip 2233)

    int NtripSize = NtripUdp.parsePacket();
    
    if (NtripSize) {
        NtripUdp.read(NtripData, NtripSize);
        //Serial.print("Ntrip Data ="); 
        //Serial.write(NtripData, sizeof(NtripData)); 
        //Serial.write(10);
        Serial.println("Ntrip Forwarded");
        SerialGPS.write(NtripData, NtripSize); 
  }
} 
