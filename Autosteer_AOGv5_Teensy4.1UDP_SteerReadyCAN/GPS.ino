
#define GPS Serial3
char rxbuffer[512];         //Extra serial rx buffer
char txbuffer[512];         //Extra serial tx buffer

#define RadioRTK Serial7
#define RadioBaudRate 115200
char RTKrxbuffer[512];      //Extra serial rx buffer

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

  RadioRTK.begin(RadioBaudRate);
  RadioRTK.addMemoryForRead(RTKrxbuffer, 512);

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  parser.addHandler("G-ZDA", ZDA_Handler);

}

//**************************************************************

void Read_IMU()
{

  //Gyro Timmed loop
  IMU_currentTime = millis();

  if ((IMU_currentTime - lastGyroTime) >= GYRO_LOOP_TIME)
    {
      lastGyroTime = IMU_currentTime;
      
      if(useBNO08x)
      {
        if (bno08x.dataAvailable() == true)
        {
            float dqx, dqy, dqz, dqw, dacr;
            uint8_t dac;

            //get quaternion
            bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);

            float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
            dqw = dqw / norm;
            dqx = dqx / norm;
            dqy = dqy / norm;
            dqz = dqz / norm;

            float ysqr = dqy * dqy;

            // yaw (z-axis rotation)
            float t3 = +2.0 * (dqw * dqz + dqx * dqy);
            float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
            yaw = atan2(t3, t4);

            // Convert yaw to degrees x10
            yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
            if (yaw < 0) yaw += 3600;

            // pitch (y-axis rotation)
            float t2 = +2.0 * (dqw * dqy - dqz * dqx);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            //            pitch = asin(t2) * RAD_TO_DEG_X_10;

                        // roll (x-axis rotation)
            float t0 = +2.0 * (dqw * dqx + dqy * dqz);
            float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
            //            roll = atan2(t0, t1) * RAD_TO_DEG_X_10;

            if (steerConfig.IsUseY_Axis)
            {
                roll = asin(t2) * RAD_TO_DEG_X_10;
                pitch = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
            else
            {
                pitch = asin(t2) * RAD_TO_DEG_X_10;
                roll = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
        }
      }     
    }
  //-----End Gyro Timed Loop-----

}

//**************************************************************

void Panda_GPS()
{
    while (GPS.available())
    {
        parser << GPS.read();
    }
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
    
    if (NtripSize) 
    {
        NtripUdp.read(NtripData, NtripSize);
        //Serial.print("Ntrip Data ="); 
        //Serial.write(NtripData, sizeof(NtripData)); 
        //Serial.write(10);
        //Serial.println("Ntrip Forwarded");
        GPS.write(NtripData, NtripSize); 
    }

//Check for Radio RTK
    if (RadioRTK.available())
    {
        GPS.write(RadioRTK.read());
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
     
   
