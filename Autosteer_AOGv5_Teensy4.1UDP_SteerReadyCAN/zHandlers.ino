// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// If odd characters showed up.
void errorHandler()
{
  //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);

    // satellite #
    parser.getArg(6, numSats);

    // HDOP
    parser.getArg(7, HDOP);

    // altitude
    parser.getArg(8, altitude);

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    if (useBNO08x || useCMPS)
    {
       imuHandler();          //Get IMU data ready
       BuildNmea();           //Build & send data GPS data to AgIO (Both Dual & Single)
    }
    else
    {
        itoa(0, imuYawRate, 10);
        itoa(0, imuRoll, 10);
        itoa(0, imuPitch, 10);
        itoa(65535, imuHeading, 10);       //65535 is max value to stop AgOpen using IMU in Panda
        BuildNmea();
    }
}

void imuHandler()
{
    int16_t temp = 0;

    if (useCMPS)
    {
        //the heading x10
        Wire.beginTransmission(CMPS14_ADDRESS);
        Wire.write(0x1C);
        Wire.endTransmission();

        Wire.requestFrom(CMPS14_ADDRESS, 3);
        while (Wire.available() < 3);

        roll = int16_t(Wire.read() << 8 | Wire.read());

        // the heading x10
        Wire.beginTransmission(CMPS14_ADDRESS);
        Wire.write(0x02);
        Wire.endTransmission();

        Wire.requestFrom(CMPS14_ADDRESS, 3);
        while (Wire.available() < 3);

        temp = Wire.read() << 8 | Wire.read();
        itoa(temp, imuHeading, 10);

        // 3rd byte pitch
        int8_t pitch = Wire.read();
        itoa(pitch, imuPitch, 10);

        // the roll x10
        temp = (int16_t)roll;
        itoa(temp, imuRoll, 10);

        // YawRate - 0 for now
        itoa(0, imuYawRate, 10);
    }

    else if (useBNO08x)
    {
        //BNO is reading in its own timer    
        // Fill rest of Panda Sentence - Heading
        temp = yaw;
        itoa(temp, imuHeading, 10);

        // the pitch x10
        temp = (int16_t)pitch;
        itoa(temp, imuPitch, 10);

        // the roll x10
        temp = (int16_t)roll;
        itoa(temp, imuRoll, 10);

        // YawRate - 0 for now
        itoa(0, imuYawRate, 10);
    }
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");


    //off to AOG
    int len = strlen(nmea);
    Udp.beginPacket(ipDestination, 9999);
    Udp.write(nmea, len);
    Udp.endPacket();

}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);

  // vtg Speed knots
  parser.getArg(4, speedKnots);


}
