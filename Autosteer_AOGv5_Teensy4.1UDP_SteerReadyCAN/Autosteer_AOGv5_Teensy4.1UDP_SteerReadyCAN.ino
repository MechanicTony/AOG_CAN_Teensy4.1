  /*
   * UDP Autosteer code for Teensy 4.1
   * For AgOpenGPS and CANBUS Autosteer ready tractors
   * 4 Feb 2021, Brian Tischler
   * Like all Arduino code - copied from somewhere else :)
   * So don't claim it as your own
   */

//----------------------------------------------------------

//Tony / @Commonrail Version 02.07.2022
//30.06.2022 Ryan / @RGM Added JCB CAN engage message
//02.07.2022  - Added Claas headland from Ryan (May still need newer version?)
//            - Fix up pilot valve output for Ryan Claas wiring mod    

//GPS to Serial3 @ 115200, Forward to AgIO via UDP
//Forward Ntrip from AgIO (Port 2233) to Serial3
//BNO08x/CMPS14 Data sent as IMU message (Not in Steering Message), timmed from steering message from AgOpen.
//BNO08x sampled at 100hz (every 10msec), only last one is used
//CMPS14 sampled once per steering message when needed

//This CAN setup is for CANBUS based steering controllers as below:
//Danfoss PVED-CL & PVED-CLS (Claas, JCB, Massey Fergerson, CaseIH, New Holland, Valtra, Deutz, Lindner)
//Fendt SCR, S4, Gen6, FendtOne Models need Part:ACP0595080 3rd Party Steering Unlock Installed
//Late model Valtra & Massey with PVED-CC valve (Steering controller in Main Tractor ECU)
//!!Model is selected via serial monitor service tool!! (One day we will will get a CANBUS setup page in AgOpen)

//For engage & disengage via CAN or Button on PCB, select "Button" as switch option in AgOpen 
//For engage via AgOpen tablet & disengage via CAN, select "None" as switch option and make sure "Remote" is on the steering wheel icon
//For engage & disengage via PCB switch only select "Switch" as switch option

//PWM value drives set curve up & down, so you need to set the PWM settings in AgOpen
//Normal settings P=15, Max=254, Low=5, Min=1
//Some tractors have very fast valves, this smooths out the setpoint from AgOpen

//Workswitch can be operated via PCB or CAN (Will need to setup CAN Messages in ISOBUS section)
//17.09.2021 - If Pressure Sensor selected, Work switch will be operated when hitch is less than pressure setting (0-250 x 0.4 = 0-100%) 
//             Note: The above is temporary use of unused variable, as one day we will get hitch % added to AgOpen

//Fendt K-Bus - (Not FendtOne models)
//Big Go/End is operated via hitch control in AgOpen 
//Arduino Hitch settings must be enableded and sent to module
//"Invert Relays" Uses section 1 to trigger hitch (Again temporary)

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
  #define EEP_Ident 0x5417

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

    struct ConfigIP {
        uint8_t ipOne = 192;
        uint8_t ipTwo = 168;
        uint8_t ipThree = 1;
    };  ConfigIP networkAddress;   //3 bytes
  
  // Module IP Address / Port
  static uint8_t ip[] = { 0,0,0,126 };
  unsigned int localPort = 8888;  
  unsigned int NtripPort = 2233;    
  
  // AOG IP Address / Port
  static uint8_t ipDestination[] = {0,0,0,255};
  unsigned int AOGPort = 9999;

  //MAC address
  byte mac[] = { 0x00,0x00,0x56,0x00,0x00,0x7E };
  
  // Buffer For Receiving UDP Data
  byte udpData[UDP_TX_PACKET_MAX_SIZE];  // Incomming Buffer
  byte NtripData[512];   

  // An EthernetUDP instance to let us send and receive packets over UDP
  EthernetUDP Udp;
  EthernetUDP NtripUdp;         

//----Teensy 4.1 Ethernet--End---------------------

//----Teensy 4.1 CANBus--Start---------------------

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> K_Bus;    //Tractor / Control Bus
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ISO_Bus;  //ISO Bus
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> V_Bus;    //Steering Valve Bus

#define ledPin 5        //Option for LED, CAN Valve Ready To Steer.
#define engageLED 24    //Option for LED, to see if Engage message is recived.

uint8_t Brand;          //Variable to set brand via serial monitor.

uint32_t Time;          //Time Arduino has been running
uint32_t relayTime;     //Time to keep "Button Pressed" from CAN Message
boolean engageCAN = 0;  //Variable for Engage from CAN
boolean workCAN = 0;    //Variable for Workswitch from CAN
uint8_t ISORearHitch = 250;   //Variable for hitch height from ISOBUS (0-250 *0.4 = 0-100%)
boolean Service = 0;    //Variable for Danfoss Service Tool Mode
boolean ShowCANData = 0;    //Variable for Showing CAN Data

boolean goDown = false, endDown = false , bitState = false, bitStateOld = false;  //CAN Hitch Control
byte hydLift = 0;
byte goPress[8]        = {0x15, 0x20, 0x06, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press big go
byte goLift[8]         = {0x15, 0x20, 0x06, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift big go
byte endPress[8]       = {0x15, 0x21, 0x06, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press big end
byte endLift[8]        = {0x15, 0x21, 0x06, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift big end
//byte goPress[8]        = {0x15, 0x22, 0x06, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press little go
//byte goLift[8]         = {0x15, 0x22, 0x06, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift little go
//byte endPress[8]       = {0x15, 0x23, 0x06, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press little end
//byte endLift[8]        = {0x15, 0x23, 0x06, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift little end

byte csm1Press[8]        = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A} ; // CLAAS CSM1 button press
byte csm2Press[8]        = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22} ; // CLAAS CSM2 button press

uint16_t setCurve = 32128;       //Variable for Set Curve to CAN
uint16_t estCurve = 32128;       //Variable for WAS from CAN
int16_t FendtEstCurve = 0;       //Variable for WAS from CAN (Fendt Only)
int16_t FendtSetCurve = 0;       //Variable for Set Curve to CAN CAN (Fendt Only)

boolean sendCAN = 0;              //Send CAN message every 2nd cycle (If needed ?)
uint8_t steeringValveReady = 0;   //Variable for Steering Valve State from CAN
boolean intendToSteer = 0;        //Do We Intend to Steer?

//----Teensy 4.1 CANBus--End-----------------------
    
  //Main loop time variables in microseconds  
  const uint16_t LOOP_TIME = 40;  //25Hz      
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;

  //IMU message
  const uint16_t IMU_DELAY_TIME = 70;    //70ms after steering message, 10hz GPS
  uint32_t IMU_lastTime = IMU_DELAY_TIME;
  uint32_t IMU_currentTime = IMU_DELAY_TIME;

  //IMU data                            
  const uint16_t GYRO_LOOP_TIME = 10;   //100Hz IMU 
  uint32_t lastGyroTime = GYRO_LOOP_TIME;

  bool isTriggered = false, blink;

  //100hz summing of gyro
  float gyro, gyroSum;
  float lastHeading;

  float roll, rollSum;
  float pitch, pitchSum;

  // booleans to see if we are using CMPS or BNO08x
  bool useCMPS = false;
  bool useBNO08x = false;

  // Address of CMPS14 shifted right one bit for arduino wire library
  #define CMPS14_ADDRESS 0x60

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

  const uint16_t WATCHDOG_THRESHOLD = 100;
  const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
  uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
  
  //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;

  //show life in AgIO - v5.5
  uint8_t helloAgIO[] = {0x80,0x81, 0x7f, 0xC7, 1, 0, 0x47 };
  uint8_t helloCounter=0;
  
  //Heart beat hello AgIO - v5.6
  uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
  int16_t helloSteerPosition = 0;
    
  //fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
  int16_t AOGSize = sizeof(AOG);

  //fromAutoSteerData FD 250 - sensor values etc
  uint8_t PGN_250[] = {0x80,0x81, 0x7f, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC }; 
  int8_t PGN_250_Size = sizeof(PGN_250) - 1;
  uint8_t aog2Count = 0;
  uint8_t pressureReading;
  uint8_t currentReading;

  //IMU PGN - 211 - 0xD3
  uint8_t data[] = {0x80,0x81,0x7D,0xD3,8, 0,0,0,0, 0,0,0,0, 15};
  int16_t dataSize = sizeof(data);
 
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
  uint8_t previousStatus = 0;

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

    //Variables for config - 0 is false - Machine Config
  struct Config {
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t enableToolLift = 0;
    uint8_t isRelayActiveHigh = 0; //if zero, active low (default)
  
  };  Config aogConfig;   //4 bytes

//*******************************************************************************
 
  void setup()
  {    
    
   //keep pulled high and drag low to activate, noise free safe   
    pinMode(WORKSW_PIN, INPUT_PULLUP); 
    pinMode(STEERSW_PIN, INPUT_PULLUP); 
    pinMode(REMOTE_PIN, INPUT_PULLUP); 
    pinMode(DIR1_RL_ENABLE, OUTPUT);
    pinMode(13, OUTPUT);
    
    pinMode(PWM2_RPWM, OUTPUT); 
    
    //set up communication
    Wire.begin();
    Serial.begin(115200);

    delay (2000);

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
  
            // Use GameRotationVector
            bno08x.enableGameRotationVector(GYRO_LOOP_TIME); 
  
            // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
            if (bno08x.getFeatureResponseAvailable() == true)
            {
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME)) == false) bno08x.printGetFeatureResponse();

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
  
    EEPROM.get(0, EEread);              // read identifier
      
    if (EEread != EEP_Ident)   // check on first start and write EEPROM
    {           
      EEPROM.put(0, EEP_Ident);
      EEPROM.put(6, aogConfig); //Machine
      EEPROM.put(10, steerSettings);   
      EEPROM.put(40, steerConfig);
      EEPROM.put(60, networkAddress);      
    }
    else 
    { 
      EEPROM.get(6, aogConfig); //Machine
      EEPROM.get(10, steerSettings);     // read the Settings
      EEPROM.get(40, steerConfig);
      EEPROM.get(60, networkAddress);      
    }
    
    // for PWM High to Low interpolator
    highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
         
//----Teensy 4.1 Ethernet--Start---------------------

  Ethernet.begin(mac,0);          // Start Ethernet with IP 0.0.0.0
  
  delay(500);
  
  if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("\r\nEthernet cable is not connected - Who cares we will start ethernet anyway.");
  }  
  
//grab the ip from EEPROM
  ip[0] = networkAddress.ipOne;
  ip[1] = networkAddress.ipTwo;
  ip[2] = networkAddress.ipThree;

  ipDestination[0] = networkAddress.ipOne;
  ipDestination[1] = networkAddress.ipTwo;
  ipDestination[2] = networkAddress.ipThree;
      
  Ethernet.setLocalIP(ip);  // Change IP address to IP set by user
  Serial.println("\r\nEthernet status OK");
  Serial.print("IP set Manually: ");
  Serial.println(Ethernet.localIP());
    
  Udp.begin(localPort);
  NtripUdp.begin(NtripPort);

  GPS_setup();

//----Teensy 4.1 Ethernet--End---------------------

//----Teensy 4.1 CANBus--Start---------------------

  pinMode(ledPin, OUTPUT);    //CAN Valve Ready LED
  digitalWrite(ledPin, LOW);

  pinMode(engageLED,OUTPUT);  //CAN engage LED
  digitalWrite(engageLED,LOW);

  Brand = EEPROM.read(70);

  Serial.println("\r\nStarting CAN-Bus Ports");
  if (Brand == 0) Serial.println("Brand = Claas (Set Via Service Tool)");
  else if (Brand == 1) Serial.println("Brand = Valtra / Massey (Set Via Service Tool)");
  else if (Brand == 2) Serial.println("Brand = CaseIH / New Holland (Set Via Service Tool)");
  else if (Brand == 3) Serial.println("Brand = Fendt SCR,S4,Gen6 (Set Via Service Tool)");
  else if (Brand == 4) Serial.println("Brand = JCB (Set Via Service Tool)");
  else if (Brand == 5) Serial.println("Brand = FendtOne (Set Via Service Tool)");
  else if (Brand == 6) Serial.println("Brand = Lindner (Set Via Service Tool)");
  else if (Brand == 7) Serial.println("Brand = AgOpenGPS (Set Via Service Tool)");
  else Serial.println("No Tractor Brand Set, Set Via Service Tool");

  delay (3000);
  CAN_setup();   //Run the Setup void (CAN page)

//----Teensy 4.1 CANBus--End---------------------

  Serial.print("\r\nAgOpenGPS Tony UDP CANBUS Ver 02.07.2022");
  Serial.println("\r\nSetup complete, waiting for AgOpenGPS");
  Serial.println("\r\nTo Start AgOpenGPS CANBUS Service Tool Enter 'S'");

  }
// End of Setup

  void loop()
  {

    currentTime = millis();

//--Main Timed Loop----------------------------------   
    if (currentTime - lastTime >= LOOP_TIME)
    {
      lastTime = currentTime;
  
      //reset debounce
      encEnable = true;
      
      //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
      if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;
      
     //read all the switches

//CANBus     
  if (steeringValveReady == 20 || steeringValveReady == 16) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  
      workSwitch = digitalRead(WORKSW_PIN);     // read work switch (PCB pin)
      if (workCAN == 1) workSwitch = 0;         // If CAN workswitch is on, set workSwitch ON
      
      if (steerConfig.SteerSwitch == 1)         //steer switch on - off
      {
        steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
      }
      else if(steerConfig.SteerButton == 1)     //steer Button momentary
      {
        reading = digitalRead(STEERSW_PIN);  
//CAN
  if (engageCAN == 1) reading = 0;              //CAN Engage is ON (Button is Pressed)
              
        if (reading == LOW && previous == HIGH) 
        {
          if (currentState == 1)
          {
            if (Brand == 3) steeringValveReady = 16;  //Fendt Valve Ready To Steer 
            if (Brand == 5) steeringValveReady = 16;  //FendtOne Valve Ready To Steer 
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
   if (steeringValveReady != 20 && steeringValveReady != 16){            
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = HIGH;
      }
//--------CAN CutOut-------------------------- 
      }
      
      else     // No steer switch and no steer button 
      {
        
   if (steeringValveReady != 20 && steeringValveReady != 16){            
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = HIGH;
      }
      
        if (previousStatus != guidanceStatus) 
        {
          if (guidanceStatus == 1 && steerSwitch == 1 && previousStatus == 0)
          {
            if (Brand == 3) steeringValveReady = 16;  //Fendt Valve Ready To Steer 
            if (Brand == 5) steeringValveReady = 16;  //FendtOne Valve Ready To Steer  
            steerSwitch = 0;
          }
          else
          {
            steerSwitch = 1;
          }
        }      
        previousStatus = guidanceStatus;
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
 
      }

      // Pressure sensor?
      if (steerConfig.PressureSensor)
      {

      }
      
      remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
      switchByte = 0;
      switchByte |= (remoteSwitch << 2);  //put remote in bit 2
      switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
      switchByte |= workSwitch;   
     
     //get steering position       
     
     //DETERMINE ACTUAL STEERING POSITION  *********From CAN-Bus************
      if (Brand == 7)
      {
        
      }
      else
      {
      if(intendToSteer == 0) setCurve = estCurve;  //Not steering so setCurve = estCurve

      steeringPosition = (setCurve - 32128 + steerSettings.wasOffset); 
      if (Brand == 3) steerAngleActual = (float)(steeringPosition) / (steerSettings.steerSensorCounts * 10);  //Fendt Only
      else if (Brand == 5) steerAngleActual = (float)(steeringPosition) / (steerSettings.steerSensorCounts * 10);  //Fendt Only
      else steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
      }
      
      //Ackerman fix
      if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);
      
      if (watchdogTimer < WATCHDOG_THRESHOLD)
      { 
        //We are good to steer
        digitalWrite(PWM2_RPWM, 1);       
      
        steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
        //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;
        
          if (Brand !=7 ){
            calcSteeringPID();  //do the pid
            motorDrive();       //out to motors the pwm value
          }
        intendToSteer = 1; //CAN Curve Inteeded for Steering
          
      }
      else
      {
        //we've lost the comm to AgOpenGPS, or just stop request
        //****** If CAN engage is ON (1), don't turn off saftey valve ******
          if (engageCAN == 0){  
            digitalWrite(PWM2_RPWM, 0); 
          }

        intendToSteer = 0; //CAN Curve NOT Inteeded for Steering   
        if (Brand !=7 ){     
        pwmDrive = 0; //turn off steering motor
        motorDrive(); //out to motors the pwm value
        }
        pulseCount=0;
      }

//-------CAN Set Curve ---------------

    VBus_Send();

//-------CAN Hitch Control--------------- 
    
if (Brand == 3) SetRelaysFendt();  //If Brand = Fendt run the hitch control bottom of this page
if (Brand == 0) SetRelaysClaas();  //If Brand = Claas run the hitch control bottom of this page

 //send empty pgn to AgIO to show activity
      if (++helloCounter > 10)
      {
      Udp.beginPacket(ipDestination, AOGPort);
      Udp.write(helloAgIO, sizeof(helloAgIO));
      Udp.endPacket();
      helloCounter = 0;
      }
    } 
    
//end of main timed loop--------------------------------------------------------------------------------------------------------------

//-----IMU Timed Loop-----

//IMU message loop

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
        while(Wire.available() < 2);       
      
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
      
      else if(useBNO08x)
      {
          //the heading x10
          data[5] = (uint8_t)bno08xHeading10x;
          data[6] = bno08xHeading10x >> 8;
  
          //the roll x10
          temp = (int16_t)rollSum;
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

            if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180???;180???] to [0;360???]
            {
                bno08xHeading = bno08xHeading + 360;
            }

            roll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI;
            pitch = (bno08x.getPitch()) * CONST_180_DIVIDED_BY_PI;
            pitch = pitch * -1;

            roll = roll * 10;
            pitch = pitch * 10;
            bno08xHeading10x = (int16_t)(bno08xHeading * 10);

            //Complementary filter
            rollSum = roll;
            pitchSum = 0.9 * pitchSum + 0.1 * pitch;
        }
      }     
    }
//-----End Gyro Timed Loop-----

    //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense, CAN data etc
    delay(1); 

//--CAN--Start--

  VBus_Receive();
  ISO_Receive();
  K_Receive();

  if ((millis()) > relayTime){
    digitalWrite(engageLED,LOW);
    engageCAN = 0;
    }
//Service Tool
  if (Serial.available()){        // Read Data From Serial Monitor 
    byte b = Serial.read();
    
    while (Serial.available()){
      Serial.read();              //Clear the serial buffer
    }
    
    if ( b == 'S') {
      Service = 1; 
      Service_Tool();
    }
}

//--CAN--End-----

//**GPS**
  Forward_GPS();
  Forward_Ntrip();
  
 //Check for UDP Packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      //Serial.println("UDP Data Avalible"); 
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
      
	  if ((bitRead(guidanceStatus, 0) == 0) || (steerSwitch == 1))        //AgOpen or Steer switch off
	  {
		  watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
	  }
    else if (Brand != 3 && gpsSpeed < 0.1 && Brand != 5)                //Speed < 0.1 and not Fendt
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
      
        //heading         
        AOG[7] = (uint8_t)9999;
        AOG[8] = 9999 >> 8;

        //roll
        AOG[9] = (uint8_t)8888;  
        AOG[10] = 8888 >> 8;       
      
      AOG[11] = switchByte;
      AOG[12] = (uint8_t)pwmDisplay;
          
      //checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < AOGSize - 1; i++)      
        CK_A = (CK_A + AOG[i]);
      
      AOG[AOGSize - 1] = CK_A;
      
      //off to AOG
      Udp.beginPacket(ipDestination, 9999);
      Udp.write(AOG, AOGSize);
      Udp.endPacket();

      //Steer Data 2 -------------------------------------------------
        if (aog2Count++ > 2)
        {
         //Send fromAutosteer2
           if ( steerConfig.CurrentSensor) PGN_250[5] = (byte)currentReading;
           else if ( steerConfig.PressureSensor) PGN_250[5] = (byte)pressureReading;
           else PGN_250[5] = 0;

           //add the checksum for AOG2
           CK_A = 0;
           for (uint8_t i = 2; i < PGN_250_Size; i++)
           {
              CK_A = (CK_A + PGN_250[i]);
           }
           PGN_250[PGN_250_Size] = CK_A;

           Udp.beginPacket(ipDestination, 9999);
           Udp.write(PGN_250, sizeof(PGN_250));
           Udp.endPacket();
           aog2Count = 0;
         }

      // Stop sending the helloAgIO message
      helloCounter = 0;

      IMU_lastTime = millis();
      isTriggered = true;

      if (blink)
        digitalWrite(13, HIGH);
      else digitalWrite(13, LOW);
      blink = !blink;

      //Serial.println(steerAngleActual); 
      //--------------------------------------------------------------------------    
    }
    
    else if (udpData[3] == 200) // Hello from AgIO
     {
       int16_t sa = (int16_t)(steerAngleActual * 100);

       helloFromAutoSteer[5] = (uint8_t)sa;
       helloFromAutoSteer[6] = sa >> 8;

       helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
       helloFromAutoSteer[8] = helloSteerPosition >> 8;
       helloFromAutoSteer[9] = switchByte;
       
       Udp.beginPacket(ipDestination, 9999);
       Udp.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
       Udp.endPacket();
           
      }
          
//Machine Data
    else if (udpData[3] == 0xEF)  //239 Machine Data
    {
      hydLift = udpData[7];

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }

//Machine Settings
    else if (udpData[3] == 0xEE) //238 Machine Settings 
    {         
      aogConfig.raiseTime = udpData[5];
      aogConfig.lowerTime = udpData[6];  
      //aogConfig.enableToolLift = udpData[7]; //This is wrong AgOpen is putting enable in sett,1
      
      //set1 
      uint8_t sett = udpData[8];  //setting0     
      if (bitRead(sett,0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;  
      if (bitRead(sett,1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;  

      //crc
      //udpData[13];        //crc
  
      //save in EEPROM and restart
      EEPROM.put(6, aogConfig);

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0;
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

      //Send Config Via CAN
      if (Brand == 7) canConfig();
  
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

      //Send Config Via CAN
      if (Brand == 7) canConfig();
  
      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0; 
       
    }//end FB

    else if (udpData[3] == 201)
    {
     //make really sure this is the subnet pgn
     if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
     {
      networkAddress.ipOne = udpData[7];
      networkAddress.ipTwo = udpData[8];
      networkAddress.ipThree = udpData[9];

      //save in EEPROM and restart
      EEPROM.put(60, networkAddress);
      SCB_AIRCR = 0x05FA0004; //Teensy Reset
      }
    }//end 201

    //Who Am I ?
    else if (udpData[3] == 202)
    {
     //make really sure this is the reply pgn
     if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
     {
      //hello from AgIO
      uint8_t scanReply[] = { 128, 129, 126, 203, 4,
      networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 126, 23 };

      //checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
       {
        CK_A = (CK_A + scanReply[i]);
       }
      scanReply[sizeof(scanReply)-1] = CK_A;

      static uint8_t ipDest[] = { 255,255,255,255 };
      uint16_t portDest = 9999; //AOG port that listens

      //off to AOG
       Udp.beginPacket(ipDest, portDest);
       Udp.write(scanReply, sizeof(scanReply));
       Udp.endPacket();
      }
     }//end 202
    
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

//Hitch Control------------------------------------------------------------
void SetRelaysFendt(void)
{
  if (goDown)   liftGo();   //Lift Go button if pressed - CAN Page
  if (endDown)   liftEnd(); //Lift End button if pressed - CAN Page

//If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1){
    bitState = (bitRead(relay, 0));
  }
//If not selected hitch command is used on headland used as Trigger  
  else{
    if (hydLift == 1) bitState = 1;
    if (hydLift == 2) bitState = 0;    
  }
//Only if tool lift is enabled AgOpen will press headland buttions via CAN
if (aogConfig.enableToolLift == 1){
  if (bitState  && !bitStateOld) pressGo(); //Press Go button - CAN Page
  if (!bitState && bitStateOld) pressEnd(); //Press End button - CAN Page
}

  bitStateOld = bitState;

}

void SetRelaysClaas(void)
{
//If Invert Relays is selected in hitch settings, Section 1 is used as trigger.  
if (aogConfig.isRelayActiveHigh == 1){
    bitState = (bitRead(relay, 0));
  }
//If not selected hitch command is used on headland used as Trigger  
  else{
    if (hydLift == 1) bitState = 1;
    if (hydLift == 2) bitState = 0;    
  }
//Only if tool lift is enabled AgOpen will press headland buttions via CAN
if (aogConfig.enableToolLift == 1){
  if (bitState  && !bitStateOld) pressCSM1(); //Press Go button - CAN Page
  if (!bitState && bitStateOld) pressCSM2(); //Press End button - CAN Page
}

  bitStateOld = bitState;
}
