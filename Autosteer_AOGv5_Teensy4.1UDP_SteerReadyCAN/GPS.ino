
#define GPS Serial3
char rxbuffer[512];   //Extra serial rx buffer
char txbuffer[512];   //Extra serial tx buffer

char nmeaBuffer[200];
int count=0;
bool stringComplete = false;

int test = 0;

//**************************************************************

void GPS_setup()
{
  if(gpsMode == 1 || gpsMode == 3)  GPS.begin(115200);
  else GPS.begin(460800);
  GPS.addMemoryForRead(rxbuffer, 512);
  GPS.addMemoryForWrite(txbuffer, 512);
}

//**************************************************************

void Read_IMU()
{

    //-----IMU Timed Loop-----

    IMU_currentTime = millis();

    if (isTriggered && (IMU_currentTime - IMU_lastTime) >= IMU_DELAY_TIME)
    {
        isTriggered = false;
        int16_t temp = 0;

        if (useCMPS)
        {
            Wire.beginTransmission(CMPS14_ADDRESS);
            Wire.write(0x02);
            Wire.endTransmission();

            Wire.requestFrom(CMPS14_ADDRESS, 2);
            while (Wire.available() < 2);

            //the heading x10
            data[6] = Wire.read();
            data[5] = Wire.read();

            //roll
            Wire.beginTransmission(CMPS14_ADDRESS);
            Wire.write(0x1C);
            Wire.endTransmission();

            Wire.requestFrom(CMPS14_ADDRESS, 2);
            while (Wire.available() < 2);

            data[8] = Wire.read();
            data[7] = Wire.read();
        }

        else if (useBNO08x)
        {
            //the heading x10
            data[5] = (uint8_t)bno08xHeading10x;
            data[6] = bno08xHeading10x >> 8;

            //the roll x10
            temp = (int16_t)roll;
            data[7] = (uint8_t)temp;
            data[8] = temp >> 8;
        }

        //checksum
        int16_t CK_A = 0;

        for (int16_t i = 2; i < dataSize - 1; i++)
        {
            CK_A = (CK_A + data[i]);
        }

        data[dataSize - 1] = CK_A;

        if (useCMPS || useBNO08x)
        {
            //off to AOG
            Udp.beginPacket(ipDestination, 9999);
            Udp.write(data, dataSize);
            Udp.endPacket();
        }
    }
    //-----End IMU Timed Loop-----

    //Gyro Timmed loop

  IMU_currentTime = millis();

  if ((IMU_currentTime - lastGyroTime) >= GYRO_LOOP_TIME)
    {
      lastGyroTime = IMU_currentTime;
      
      if(useBNO08x)
      {
        if (bno08x.dataAvailable() == true)
       {
            bno08xHeading = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI; // Convert yaw / heading to degrees
            bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

            if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180;180] to [0;360]
            {
                bno08xHeading = bno08xHeading + 360;
            }

            //roll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI;
            roll = (bno08x.getPitch()) * CONST_180_DIVIDED_BY_PI;

            roll = roll * 10;
            bno08xHeading10x = (int16_t)(bno08xHeading * 10);
        }
      }     
    }
//-----End Gyro Timed Loop-----

}

//**************************************************************

void Panda_GPS()
{



}

//**************************************************************

void Forward_GPS()
{
  while (GPS.available())
  {
    char c = GPS.read();
    nmeaBuffer[count++] = c;
    if(c == '\n')stringComplete = true;
    if(count == 200 || stringComplete == true)break;
   } 

  if(count == 200 || stringComplete == true){ 
    if (stringComplete == true){  
      Udp.beginPacket(ipDestination, AOGPort);
      Udp.write(nmeaBuffer,count);
      Udp.endPacket();
    }
    clearBufferArray();
    count = 0;
  }
  
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
        GPS.write(NtripData, NtripSize); 
  }
}
    
//-------------------------------------------------------------------------------------------------

void clearBufferArray()
{
  /*
  for (int i=0; i<count; i++)
  {
    nmeaBuffer[i]=NULL;
    stringComplete = false;
  }
  */
  
  strcpy(nmeaBuffer, "");
  stringComplete = false;

}
     
   
