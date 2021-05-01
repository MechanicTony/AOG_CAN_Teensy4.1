  /*
   * UDP Autosteer code for Teensy 4.1
   * For AgOpenGPS and CAN-Bus Autosteer ready tractors
   * 4 Feb 2021, Brian Tischler
   * Like all Arduino code - copied from somewhere else :)
   * So don't claim it as your own
   */

//----------------------------------------------------------
//This CAN setup is for FENDT steering controller
//Setup for button engage via ISO-BUS/disengage via V-Bus or Button on PCB
//PWM value drives set curve up & down
//Current Sensor Selected will operate work switch from steer switch
//----------------------------------------------------------

  ////////////////// User Settings /////////////////////////  

  //How many degrees before decreasing Max PWM
  #define LOW_HIGH_DEGREES 5.0

  /*  PWM Frequency -> 
   *   490hz (default) = 0
   *   122hz = 1
   *   3921hz = 2
   */
  #define PWM_Frequency 0
  
  /////////////////////////////////////////////

  // if not in eeprom, overwrite 
  #define EEP_Ident 5100 

  // Address of CMPS14 shifted right one bit for arduino wire library
  #define CMPS14_ADDRESS 0x60

  // BNO08x definitions
  #define REPORT_INTERVAL 90 //Report interval in ms (same as the delay at the bottom)

  //   ***********  Motor drive connections  **************888
  //Connect ground only for cytron, Connect Ground and +5v for IBT2
    
  //Dir1 for Cytron Dir, Both L and R enable for IBT2
  #define DIR1_RL_ENABLE  4  //PD4

  //PWM1 for Cytron PWM, Left PWM for IBT2
  #define PWM1_LPWM  3  //PD3

  //Not Connected for Cytron, Right PWM for IBT2
  #define PWM2_RPWM  9 //D9

  //--------------------------- Switch Input Pins ------------------------
  #define STEERSW_PIN 6 //PD6
  #define WORKSW_PIN 7  //PD7
  #define REMOTE_PIN 8  //PB0

  #define CONST_180_DIVIDED_BY_PI 57.2957795130823

  #include <Wire.h>
  #include <EEPROM.h> 

  #include "BNO08x_AOG.h"
  
//----Teensy 4.1 Ethernet--Start---------------------
  #include <NativeEthernet.h>
  #include <NativeEthernetUdp.h>
  
  // Module IP Address / Port
  IPAddress ip(192, 168, 1, 177);
  unsigned int localPort = 8888;      
  
  // AOG IP Address / Port
  IPAddress remote(192, 168, 1, 255);
  unsigned int AOGPort = 9999;

  //MAC address
  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  
  // Buffer For Receiving UDP Data
  byte udpData[UDP_TX_PACKET_MAX_SIZE];  // Incomming Buffer

  // An EthernetUDP instance to let us send and receive packets over UDP
  EthernetUDP Udp;

//----Teensy 4.1 Ethernet--End---------------------

//----Teensy 4.1 CANBus--Start---------------------

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> K_Bus;    //Tractor / Control Bus
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ISO_Bus;  //ISO Bus
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> V_Bus;    //Steering Valve Bus

#define ledPin 5     //Option for LED, CAN Valve Ready To Steer.
#define engageLED 24 //Option for LED, to see if Engage message is recived.

uint32_t Time;          //Time Arduino has been running
uint32_t relayTime;     //Time to keep "Button Pressed" from CAN Message
boolean engageCAN = 0;  //Variable for Engage from ISO-Bus
boolean cutoutCAN = 0;  //Variable for Cutout from V-Bus

int16_t setCurve = 0;       //Variable for Set Curve to V-Bus
int16_t estCurve = 0;       //Variable for WAS from V-Bus

boolean sendCAN = 0;              //Send CAN message every 2nd cycle (If needed ?)
uint8_t steeringValveReady = 0;   //Variable for Steering Valve State from CAN
boolean intendToSteer = 0;        //Do We Intend to Steer?

//----Teensy 4.1 CANBus--End-----------------------
    
  //loop time variables in microseconds  
  const uint16_t LOOP_TIME = 40;  //25Hz   
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;

  const uint16_t WATCHDOG_THRESHOLD = 100;
  const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
  uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
  
   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;

  //show life in AgIO
  uint8_t helloAgIO[] = {0x80,0x81, 0x7f, 0xC7, 1, 0, 0x47 };
  uint8_t helloCounter=0;

  //fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
  int16_t AOGSize = sizeof(AOG);

  // booleans to see if we are using CMPS or BNO08x
  bool useCMPS = false;
  bool useBNO08x = false;

  // BNO08x address variables to check where it is
  const uint8_t bno08xAddresses[] = {0x4A,0x4B};
  const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses)/sizeof(bno08xAddresses[0]);
  uint8_t bno08xAddress;
  BNO080 bno08x;

  float bno08xHeading = 0;
  double bno08xRoll = 0;
  double bno08xPitch = 0;

  int16_t bno08xHeading10x = 0;
  int16_t bno08xRoll10x = 0;
 
  //EEPROM
  int16_t EEread = 0;

  //Relays
  bool isRelayActiveHigh = true;
  uint8_t relay = 0, relayHi = 0, uTurn = 0;
  uint8_t tram = 0;
    
  //Switches
  uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

  //On Off
  uint8_t guidanceStatus = 0;

  //speed sent as *10
  float gpsSpeed = 0;
  
  //steering variables
  float steerAngleActual = 0;
  float steerAngleSetPoint = 0; //the desired angle from AgOpen
  int16_t steeringPosition = 0; //from steering sensor
  float steerAngleError = 0; //setpoint - actual
  
  //pwm variables
  int16_t pwmDrive = 0, pwmDisplay = 0;
  float pValue = 0;
  float errorAbs = 0;
  float highLowPerDeg = 0; 

  //Steer switch button  
  uint8_t currentState = 1, reading, previous = 0;
  uint8_t pulseCount = 0; // Steering Wheel Encoder
  bool encEnable = false; //debounce flag
  uint8_t thisEnc = 0, lastEnc = 0;

   //Variables for settings  
   struct Storage {
      uint8_t Kp = 40;  //proportional gain
      uint8_t lowPWM = 10;  //band of no action
      int16_t wasOffset = 0;
      uint8_t minPWM = 9;
      uint8_t highPWM = 60;//max PWM value
      float steerSensorCounts = 30;        
      float AckermanFix = 1;     //sent as percent
  };  Storage steerSettings;  //11 bytes

   //Variables for settings - 0 is false  
   struct Setup {
      uint8_t InvertWAS = 0;
      uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
      uint8_t MotorDriveDirection = 0;
      uint8_t SingleInputWAS = 1;
      uint8_t CytronDriver = 1;
      uint8_t SteerSwitch = 0;  //1 if switch selected
      uint8_t SteerButton = 0;  //1 if button selected
      uint8_t ShaftEncoder = 0;
      uint8_t PressureSensor = 0;
      uint8_t CurrentSensor = 0;
      uint8_t PulseCountMax = 5; 
      uint8_t IsDanfoss = 0; 
  };  Setup steerConfig;          //9 bytes

//*******************************************************************************
 
  void setup()
  {    
    
   //keep pulled high and drag low to activate, noise free safe   
    pinMode(WORKSW_PIN, INPUT_PULLUP); 
    pinMode(STEERSW_PIN, INPUT_PULLUP); 
    pinMode(REMOTE_PIN, INPUT_PULLUP); 
    pinMode(DIR1_RL_ENABLE, OUTPUT);
    
    if (steerConfig.CytronDriver) pinMode(PWM2_RPWM, OUTPUT); 
    
    //set up communication
    Wire.begin();
    Serial.begin(38400);

  /*    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
  
    //test if CMPS working
    uint8_t error;
    Wire.beginTransmission(CMPS14_ADDRESS);
    error = Wire.endTransmission();
    
    if (error == 0)
    {
      Serial.println("Error = 0");
      Serial.print("CMPS14 ADDRESs: 0x");
      Serial.println(CMPS14_ADDRESS, HEX);
      Serial.println("CMPS14 Ok.");
      useCMPS = true;
    }
    else 
    {
      Serial.println("Error = 4");
      Serial.println("CMPS not Connected or Found");
      useCMPS = false;
    }

    // Check for BNO08x
    if(!useCMPS)
    {
      for(int16_t i = 0; i < nrBNO08xAdresses; i++)
      {
        bno08xAddress = bno08xAddresses[i];
        
        Serial.print("\r\nChecking for BNO08X on ");
        Serial.println(bno08xAddress, HEX);
        Wire.beginTransmission(bno08xAddress);
        error = Wire.endTransmission();
    
        if (error == 0)
        {
          Serial.println("Error = 0");
          Serial.print("BNO08X ADDRESs: 0x");
          Serial.println(bno08xAddress, HEX);
          Serial.println("BNO08X Ok.");
          
          // Initialize BNO080 lib        
          if (bno08x.begin(bno08xAddress))
          {
            Wire.setClock(400000); //Increase I2C data rate to 400kHz
  
            // Use gameRotationVector
            bno08x.enableGameRotationVector(REPORT_INTERVAL); //Send data update every REPORT_INTERVAL in ms for BNO085
  
            // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
            if (bno08x.getFeatureResponseAvailable() == true)
            {
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, REPORT_INTERVAL) == false) bno08x.printGetFeatureResponse();

              // Break out of loop
              useBNO08x = true;
              break;
            }
            else 
            {
              Serial.println("BNO08x init fails!!");
            }
          }
          else
          {
            Serial.println("BNO080 not detected at given I2C address.");
          }
        }
        else 
        {
          Serial.println("Error = 4");
          Serial.println("BNO08X not Connected or Found"); 
        }
      }
    }
    
    //50Khz I2C
    TWBR = 144;
  
    EEPROM.get(0, EEread);              // read identifier
      
    if (EEread != EEP_Ident)   // check on first start and write EEPROM
    {           
      EEPROM.put(0, EEP_Ident);
      EEPROM.put(10, steerSettings);   
      EEPROM.put(40, steerConfig);
    }
    else 
    { 
      EEPROM.get(10, steerSettings);     // read the Settings
      EEPROM.get(40, steerConfig);
    }
    
    // for PWM High to Low interpolator
    highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
         
//----Teensy 4.1 Ethernet--Start---------------------

  Serial.println("\r\nStarting Ethernet, Is Cable Connected to Router?");
     
  Ethernet.begin(mac, ip);

  Udp.begin(localPort);

//----Teensy 4.1 Ethernet--End---------------------

//----Teensy 4.1 CANBus--Start---------------------

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(engageLED,OUTPUT);
  digitalWrite(engageLED,LOW);

  Serial.println("\r\nStarting CAN-Bus Ports");
  CAN_setup();

//----Teensy 4.1 CANBus--End---------------------
  
  Serial.println("\r\nSetup complete, waiting for AgOpenGPS");

  }
// End of Setup

  void loop()
  {

    currentTime = millis();

//--Timed Loop----------------------------------   
    if (currentTime - lastTime >= LOOP_TIME)
    {
      lastTime = currentTime;
  
      //reset debounce
      encEnable = true;
      
      //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
      if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;
      
     //read all the switches

//CANBus     
  if (cutoutCAN == 0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  
      workSwitch = digitalRead(WORKSW_PIN);  // read work switch
      
      if (steerConfig.SteerSwitch == 1)         //steer switch on - off
      {
        steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
      }
      else if(steerConfig.SteerButton == 1)     //steer Button momentary
      {
        reading = digitalRead(STEERSW_PIN);  
//CAN
  if (engageCAN == 1) reading = 0;         //CAN Engage is ON (Button is Pressed)
              
        if (reading == LOW && previous == HIGH) 
        {
          if (currentState == 1)
          {
            currentState = 0;
            steerSwitch = 0;
          }
          else
          {
            currentState = 1;
            steerSwitch = 1;
          }
        }      
        previous = reading;
        
//--------CAN CutOut--------------------------

   if (cutoutCAN != 0){            
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = HIGH;
      cutoutCAN = 0;
      }

//--------CAN CutOut-------------------------- 
      }
      else                                      // No steer switch and no steer button
      {
        // So set the correct value. When guidanceStatus = 1, 
        // it should be on because the button is pressed in the GUI
        // But the guidancestatus should have set it off first
        if (guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
        {
          steerSwitch = 0;
          previous = 1;
        }

        // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
        if (guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
        {
          steerSwitch = 1;
          previous = 0;
        }
      }
      
      if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax) 
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
 
      
      // Current sensor?
      if (steerConfig.CurrentSensor)
      {
//CAN
      workSwitch = steerSwitch;  //****** If Current Sensor is selected, steer switch will paint lines.
      }

      // Pressure sensor?
      if (steerConfig.PressureSensor)
      {

      }
      
      remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
      switchByte = 0;
      switchByte |= (remoteSwitch << 2); //put remote in bit 2
      switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
      switchByte |= workSwitch;   
    
      /*
      #if Relay_Type == 1
          SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
          SetuTurnRelays();  //turn on off uTurn relays
      #endif
      */
     
     //get steering position       
     
     //DETERMINE ACTUAL STEERING POSITION  *********From CAN-Bus************
      

     if(intendToSteer == 0) setCurve = estCurve; 

      steeringPosition = (setCurve + steerSettings.wasOffset); 
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;

        
      //Ackerman fix
      if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);
      
      if (watchdogTimer < WATCHDOG_THRESHOLD)
      { 
       //Enable H Bridge for IBT2, hyd aux, etc for cytron
        if (steerConfig.CytronDriver) 
        {
          if (steerConfig.IsRelayActiveHigh) 
          {
            digitalWrite(PWM2_RPWM, 0); 
          }
          else  
          {
            digitalWrite(PWM2_RPWM, 1);       
          }        
        }
        else digitalWrite(DIR1_RL_ENABLE, 1);     
        
        steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
        //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;
        
        calcSteeringPID();  //do the pid
        motorDrive();       //out to motors the pwm value
        intendToSteer = 1; //CAN Curve Inteeded for Steering
      }
      else
      {
        //we've lost the comm to AgOpenGPS, or just stop request
        //Disable H Bridge for IBT2, hyd aux, etc for cytron
        if (steerConfig.CytronDriver) 
        {
          if (steerConfig.IsRelayActiveHigh) 
          {
            digitalWrite(PWM2_RPWM, 1); 
          }
          else  
          {
            digitalWrite(PWM2_RPWM, 0);       
          }
        }
        else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2
        
        intendToSteer = 0; //CAN Curve NOT Inteeded for Steering        
        pwmDrive = 0; //turn off steering motor
        motorDrive(); //out to motors the pwm value
        pulseCount=0;
      }

//-------CAN Set Curve START---------------

    VBus_Send();

//-------CAN Set Curve END---------------  

      //send empty pgn to AgIO to show activity
      if (++helloCounter > 10)
      {
      Udp.beginPacket(remote, AOGPort);
      Udp.write(helloAgIO, sizeof(helloAgIO));
      Udp.endPacket();
      helloCounter = 0;
      }
    } //end of timed loop

    //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense, CAN data etc
    delay(1); 

//--CAN--Start--

  VBus_Receive();
  ISO_Receive();

  if ((millis()) > relayTime){
    digitalWrite(engageLED,LOW);
    engageCAN = 0;
    }

//--CAN--End-----
    
 //Check for UDP Packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
     // Serial.println("UDP Data Avalible"); 
      udpSteerRecv();
    }

    if (encEnable)
    {
      thisEnc = digitalRead(REMOTE_PIN);
      if (thisEnc != lastEnc)
      {
        lastEnc = thisEnc;
        if ( lastEnc) EncoderFunc();
      }
    }
      
  } // end of main loop

//********************************************************************************

void udpSteerRecv()
{
  
Udp.read(udpData, UDP_TX_PACKET_MAX_SIZE);

  if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
  {
    if (udpData[3] == 0xFE)  //254
    {
      gpsSpeed = ((float)(udpData[5] | udpData[6] << 8))*0.1;

      guidanceStatus = udpData[7];
      
      //Bit 8,9    set point steer angle * 100 is sent
      steerAngleSetPoint = ((float)(udpData[8] | ((int8_t)udpData[9]) << 8))*0.01; //high low bytes
      
      //Serial.println(gpsSpeed); 
      
	  if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
	  {
		  watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
	  }
	  else          //valid conditions to turn on autosteer
	  {
		  watchdogTimer = 0;  //reset watchdog
	  }

      //Bit 10 Tram 
      tram = udpData[10];
      
      //Bit 11
      relay = udpData[11];

      //Bit 12
      relayHi = udpData[12];
     
      //----------------------------------------------------------------------------
      //Serial Send to agopenGPS
      
      int16_t sa = (int16_t)(steerAngleActual*100);
      
      AOG[5] = (uint8_t)sa;
      AOG[6] = sa >> 8;
      
      if (useCMPS)
      {
        Wire.beginTransmission(CMPS14_ADDRESS);  
        Wire.write(0x02);                     
        Wire.endTransmission();
        
        Wire.requestFrom(CMPS14_ADDRESS, 2); 
        while(Wire.available() < 2);       
      
        //the heading x10
        AOG[8] = Wire.read();
        AOG[7] = Wire.read();
       
        Wire.beginTransmission(CMPS14_ADDRESS);  
        Wire.write(0x1C);                    
        Wire.endTransmission();
       
        Wire.requestFrom(CMPS14_ADDRESS, 2);  
        while(Wire.available() < 2);        
      
        //the roll x10
        AOG[10] = Wire.read();
        AOG[9] = Wire.read();            
      }
      else if(useBNO08x)
      {
        if (bno08x.dataAvailable() == true)
        {
          bno08xHeading = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI; // Convert yaw / heading to degrees
          bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data
          
          if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180°;180°] to [0;360°]
          {
            bno08xHeading = bno08xHeading + 360;
          }
              
          //bno08xRoll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI; //Convert roll to degrees
          bno08xRoll = (bno08x.getPitch())* CONST_180_DIVIDED_BY_PI; // Convert pitch to degrees
  
          bno08xHeading10x = (int16_t)(bno08xHeading * 10);
          bno08xRoll10x = (int16_t)(bno08xRoll * 10);

         /* Serial.print(bno08xHeading10x);
          Serial.print(",");
          Serial.println(bno08xRoll10x); 
          Serial.println(steerAngleSetPoint);
          Serial.println(steeringPosition);
          Serial.println(steerAngleActual);
          Serial.println(steerAngleError);
          Serial.println(steerSettings.wasOffset);
          Serial.println(setCurve*10);*/
          
          
          //the heading x10
          AOG[7] = (uint8_t)bno08xHeading10x;
          AOG[8] = bno08xHeading10x >> 8;
          
  
          //the roll x10
          AOG[9] = (uint8_t)bno08xRoll10x;
          AOG[10] = bno08xRoll10x >> 8;
        }
      }
      else
      { 
        //heading         
        AOG[7] = (uint8_t)9999;
        AOG[8] = 9999 >> 8;

        //roll
        AOG[9] = (uint8_t)8888;  
        AOG[10] = 8888 >> 8;
      }        
      
      AOG[11] = switchByte;
      AOG[12] = (uint8_t)pwmDisplay;
          
      //checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < AOGSize - 1; i++)      
        CK_A = (CK_A + AOG[i]);
      
      AOG[AOGSize - 1] = CK_A;
      
      //off to AOG
      Udp.beginPacket(remote, 9999);
      Udp.write(AOG, AOGSize);
      Udp.endPacket();

      // Stop sending the helloAgIO message
      helloCounter = 0;

      //Serial.println(steerAngleActual); 
      //--------------------------------------------------------------------------    
    }

    //steer settings
    else if (udpData[3] == 0xFC)  //252
    {      
      //PID values
      steerSettings.Kp = ((float)udpData[5]);   // read Kp from AgOpenGPS
      
      steerSettings.highPWM = udpData[6]; // read high pwm
      
      steerSettings.lowPWM = (float)udpData[7];   // read lowPWM from AgOpenGPS              
  
      steerSettings.minPWM = udpData[8]; //read the minimum amount of PWM for instant on
      
      steerSettings.steerSensorCounts = udpData[9]; //sent as setting displayed in AOG
  
      steerSettings.wasOffset = (udpData[10]);  //read was zero offset Lo

      steerSettings.wasOffset |= (((int8_t)udpData[11]) << 8);  //read was zero offset Hi

      steerSettings.AckermanFix = (float)udpData[12] * 0.01;  
      
      //crc
      //udpData[13];
        
      //store in EEPROM
      EEPROM.put(10, steerSettings);           
  
      // for PWM High to Low interpolator
      highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
    }

    else if (udpData[3] == 0xFB)  //251 FB - SteerConfig
    {        
      uint8_t sett = udpData[5]; //setting0
       
      if (bitRead(sett,0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
      if (bitRead(sett,1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
      if (bitRead(sett,2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
      if (bitRead(sett,3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
      if (bitRead(sett,4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
      if (bitRead(sett,5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
      if (bitRead(sett,6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
      if (bitRead(sett,7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;
      
      steerConfig.PulseCountMax = udpData[6];

      //was speed
      //udpData[7]; 

      sett = udpData[8]; //setting1 - Danfoss valve etc
      
      if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
      if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
      if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
      
      //crc
      //udpData[13];        
       
      EEPROM.put(40, steerConfig);
  
      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0; 
       
    }//end FB
  } //end if 80 81 7F  
} //end udp callback


//ISR Steering Wheel Encoder

  void EncoderFunc()
  {        
     if (encEnable) 
     {
        pulseCount++; 
        encEnable = false;
     }            
  }
