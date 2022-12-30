
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
  GPS.begin(115200);
  GPS.addMemoryForRead(rxbuffer, 512);
  GPS.addMemoryForWrite(txbuffer, 512);
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
     
   
