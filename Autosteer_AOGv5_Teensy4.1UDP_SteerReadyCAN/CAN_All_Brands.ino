
//  !! Set Brand via Service Tool (Serial Monitor) !! 
//  0 = Claas (1E/30 Navagation Controller, 13/19 Steering Controller) - See Claas Notes on Service Tool Page
//  1 = Valtra, Massey Fergerson (Standard Danfoss ISO 1C/28 Navagation Controller, 13/19 Steering Controller)
//  2 = CaseIH, New Holland (AA/170 Navagation Controller, 08/08 Steering Controller)
//  3 = Fendt (2C/44 Navagation Controller, F0/240 Steering Controller)
//  4 = JCB (AB/171 Navagation Controller, 13/19 Steering Controller)
//  5 = FendtOne - Same as Fendt but 500kbs K-Bus.
//  6 = Lindner (F0/240 Navagation Controller, 13/19 Steering Controller)
//  7 = AgOpenGPS - Remote CAN/PWM module (1C/28 Navagation Controller, 13/19 Steering Controller)

//---Start Teensy CANBus Ports and Claim Addresses - If needed 

void CAN_setup (void) {

//V_Bus is CAN-3 and is the Steering BUS
  V_Bus.begin();
  V_Bus.setBaudRate(250000);
  V_Bus.enableFIFO();
  V_Bus.setFIFOFilter(REJECT_ALL);
if (Brand == 0){
  V_Bus.setFIFOFilter(0, 0x0CAC1E13, EXT);  //Claas Curve Data & Valve State Message
  V_Bus.setFIFOFilter(1, 0x18EF1CD2, EXT);  //Claas Engage Message
  V_Bus.setFIFOFilter(2, 0x1CFFE6D2, EXT);  //Claas Work Message (CEBIS Screen MR Models)
  }
if (Brand == 1){
  V_Bus.setFIFOFilter(0, 0x0CAC1C13, EXT);  //Valtra Curve Data & Valve State Message
  V_Bus.setFIFOFilter(1, 0x18EF1C32, EXT);  //Valtra Engage Message
  }  
if (Brand == 2){
  V_Bus.setFIFOFilter(0, 0x0CACAA08, EXT);  //CaseIH Curve Data & Valve State Message
  V_Bus.setFIFOUserFilter(1, 0x0CEFAA08, 0x0CEF08AA, 0x0000FF00, EXT);
  }   
if (Brand == 3){
  V_Bus.setFIFOFilter(0, 0x0CEF2CF0, EXT);  //Fendt Curve Data & Valve State Message
  }   
if (Brand == 4){
  V_Bus.setFIFOFilter(0, 0x0CACAB13, EXT);  //JCB Curve Data & Valve State Message
  V_Bus.setFIFOFilter(1, 0x18EFAB27, EXT);  //JCB engage message
  }
if (Brand == 5){
  V_Bus.setFIFOFilter(0, 0x0CEF2CF0, EXT);  //FendtONE Curve Data & Valve State Message
  }   
if (Brand == 6){
  V_Bus.setFIFOFilter(0, 0x0CACF013, EXT);  //Lindner Curve Data & Valve State Message
  }
if (Brand == 7){
  V_Bus.setFIFOFilter(0, 0x0CAC1C13, EXT);  //AgOpenGPS Curve Data & Valve State Message
  V_Bus.setFIFOFilter(1, 0x19EF1C13, EXT);  //AgOpenGPS error message
  }   
  
// Claim V_Bus Address 
if (Brand >= 0 && Brand <= 7){
  CAN_message_t msgV;
  if (Brand == 0) msgV.id = 0x18EEFF1E;       //Claas
  else if (Brand == 1) msgV.id = 0x18EEFF1C;  //Massey, Valtra, ETC
  else if (Brand == 2) msgV.id = 0x18EEFFAA;  //Case, Hew Holland
  else if (Brand == 3) msgV.id = 0x18EEFF2C;  //Fendt
  else if (Brand == 4) msgV.id = 0x18EEFFAB;  //JCB
  else if (Brand == 5) msgV.id = 0x18EEFF2C;  //FendtONE
  else if (Brand == 6) msgV.id = 0x18EEFFF0;  //Linder
  else if (Brand == 7) msgV.id = 0x18EEFF1C;  //AgOpenGPS
  msgV.flags.extended = true;
  msgV.len = 8;
  msgV.buf[0] = 0x00;
  msgV.buf[1] = 0x00;
  msgV.buf[2] = 0xC0;
  msgV.buf[3] = 0x0C;
  msgV.buf[4] = 0x00;
  msgV.buf[5] = 0x17;
  msgV.buf[6] = 0x02;
  msgV.buf[7] = 0x20;
  V_Bus.write(msgV);
}
delay(500);

//ISO_Bus is CAN-2 
  ISO_Bus.begin();
  ISO_Bus.setBaudRate(250000);
  ISO_Bus.enableFIFO();
  ISO_Bus.setFIFOFilter(REJECT_ALL);
//Put filters into here to let them through (All blocked by above line)
  ISO_Bus.setFIFOFilter(0,0x0CFE45F0, EXT);  //ISOBUS Rear Hitch Infomation
  
if (Brand == 3){
  ISO_Bus.setFIFOFilter(1,0x18EF2CF0, EXT);  //Fendt Engage Message
  } 

if (Brand == 5){
  ISO_Bus.setFIFOFilter(1,0x18EF2CF0, EXT);  //Fendt Engage Message
  }  

if (Brand >= 0 && Brand <= 7){
  CAN_message_t msgISO;
  if (Brand == 0) msgISO.id = 0x18EEFF1E;       //Claas
  else if (Brand == 1) msgISO.id = 0x18EEFF1C;  //Massey, Valtra, ETC
  else if (Brand == 2) msgISO.id = 0x18EEFFAA;  //Case, Hew Holland
  else if (Brand == 3) msgISO.id = 0x18EEFF2C;  //Fendt
  else if (Brand == 4) msgISO.id = 0x18EEFFAB;  //JCB
  else if (Brand == 5) msgISO.id = 0x18EEFF2C;  //FendtOne
  else if (Brand == 6) msgISO.id = 0x18EEFFF0;  //Linder
  else if (Brand == 7) msgISO.id = 0x18EEFF1C;  //AgOpenGPS
  msgISO.flags.extended = true;
  msgISO.len = 8;
  msgISO.buf[0] = 0x00;
  msgISO.buf[1] = 0x00;
  msgISO.buf[2] = 0xC0;
  msgISO.buf[3] = 0x0C;
  msgISO.buf[4] = 0x00;
  msgISO.buf[5] = 0x17;
  msgISO.buf[6] = 0x02;
  msgISO.buf[7] = 0x20;
  ISO_Bus.write(msgISO);
}

delay (500); 

//K_Bus is CAN-1 and is the Main Tractor Bus
  K_Bus.begin();
  if (Brand == 5) K_Bus.setBaudRate(500000);
  else K_Bus.setBaudRate(250000);
  K_Bus.enableFIFO();
  K_Bus.setFIFOFilter(REJECT_ALL);
//Put filters into here to let them through (All blocked by above line)
if (Brand == 3){
  K_Bus.setFIFOFilter(0, 0x613, STD);  //Fendt Arm Rest Buttons
  } 
if (Brand == 5){
  K_Bus.setFIFOFilter(0, 0xCFFD899, EXT);  //FendtOne Engage
  }
if (Brand == 2){
  K_Bus.setFIFOFilter(0, 0x14FF7706, EXT);  //CaseIH Engage Message
  K_Bus.setFIFOFilter(1, 0x18FE4523, EXT);  //CaseIH Rear Hitch Infomation
  } 
  
  delay (500); 
} //End CAN SETUP


//---Send V_Bus message

void VBus_Send(){
    CAN_message_t VBusSendData;
if (Brand == 0){
    VBusSendData.id = 0x0CAD131E;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 0;
    VBusSendData.buf[4] = 0;
    VBusSendData.buf[5] = 0;
    VBusSendData.buf[6] = 0;
    VBusSendData.buf[7] = 0;
    V_Bus.write(VBusSendData);
}
else if (Brand == 1){
    VBusSendData.id = 0x0CAD131C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 255;
    VBusSendData.buf[4] = 255;
    VBusSendData.buf[5] = 255;
    VBusSendData.buf[6] = 255;
    VBusSendData.buf[7] = 255;
    V_Bus.write(VBusSendData);
}
else if (Brand == 2){
    VBusSendData.id = 0x0CAD08AA;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 255;
    VBusSendData.buf[4] = 255;
    VBusSendData.buf[5] = 255;
    VBusSendData.buf[6] = 255;
    VBusSendData.buf[7] = 255;
    V_Bus.write(VBusSendData);
}
else if (Brand == 3){
    FendtSetCurve = setCurve - 32128;
    VBusSendData.id = 0x0CEFF02C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 6;
    VBusSendData.buf[0] = 5;
    VBusSendData.buf[1] = 9;
    VBusSendData.buf[3] = 10;
    if (intendToSteer == 1){  
      VBusSendData.buf[2] = 3;
      VBusSendData.buf[4] = highByte(FendtSetCurve);
      VBusSendData.buf[5] = lowByte(FendtSetCurve);
    }
    else{
      VBusSendData.buf[2] = 2;
      VBusSendData.buf[4] = 0;
      VBusSendData.buf[5] = 0; 
    }
    V_Bus.write(VBusSendData);
}
else if (Brand == 4){
    VBusSendData.id = 0x0CAD13AB;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 255;
    VBusSendData.buf[4] = 255;
    VBusSendData.buf[5] = 255;
    VBusSendData.buf[6] = 255;
    VBusSendData.buf[7] = 255;
    V_Bus.write(VBusSendData);
}
else if (Brand == 5){
    FendtSetCurve = setCurve - 32128;
    VBusSendData.id = 0x0CEFF02C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 6;
    VBusSendData.buf[0] = 5;
    VBusSendData.buf[1] = 9;
    VBusSendData.buf[3] = 10;
    if (intendToSteer == 1){  
      VBusSendData.buf[2] = 3;
      VBusSendData.buf[4] = highByte(FendtSetCurve);
      VBusSendData.buf[5] = lowByte(FendtSetCurve);
    }
    else{
      VBusSendData.buf[2] = 2;
      VBusSendData.buf[4] = 0;
      VBusSendData.buf[5] = 0; 
    }
    V_Bus.write(VBusSendData);
}
else if (Brand == 6){
    VBusSendData.id = 0x0CAD13F0;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    VBusSendData.buf[0] = lowByte(setCurve);
    VBusSendData.buf[1] = highByte(setCurve);
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 255;
    VBusSendData.buf[4] = 255;
    VBusSendData.buf[5] = 255;
    VBusSendData.buf[6] = 255;
    VBusSendData.buf[7] = 255;
    V_Bus.write(VBusSendData);
}
else if (Brand == 7){
    VBusSendData.id = 0x0CAD131C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 8;
    int16_t sp = (int16_t)(steerAngleSetPoint*100);
    VBusSendData.buf[0] = (uint8_t)sp;
    VBusSendData.buf[1] = sp >> 8;
    if (intendToSteer == 1)VBusSendData.buf[2] = 253;
    if (intendToSteer == 0)VBusSendData.buf[2] = 252;
    VBusSendData.buf[3] = 0;
    VBusSendData.buf[4] = 0;
    VBusSendData.buf[5] = 0;
    VBusSendData.buf[6] = 0;
    VBusSendData.buf[7] = 0;
    V_Bus.write(VBusSendData);
}
}

//---Receive V_Bus message

void VBus_Receive()
{
    CAN_message_t VBusReceiveData;
    if (V_Bus.read(VBusReceiveData)) {

        if (Brand == 0)
        {
          //**Current Wheel Angle & Valve State**
          if (VBusReceiveData.id == 0x0CAC1E13){        
                estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
                steeringValveReady = (VBusReceiveData.buf[2]); 
          } 
  
          //**Engage Message**
          if (VBusReceiveData.id == 0x18EF1CD2)
          {
            if ((VBusReceiveData.buf[1])== 0 && (VBusReceiveData.buf[2])== 0)   //Ryan Stage5 Models?
            {
                engageCAN = bitRead(VBusReceiveData.buf[0],2);
                Time = millis();
                digitalWrite(engageLED,HIGH); 
                relayTime = ((millis() + 1000));
                //*****Turn saftey valve ON**********
                if (engageCAN == 1) digitalWrite(PWM2_RPWM, 1);       
            }

            else if ((VBusReceiveData.buf[0]) == 39 && (VBusReceiveData.buf[2]) == 241)   //Ryan MR Models?
            {
              engageCAN = bitRead(VBusReceiveData.buf[1],0);
              Time = millis();
              digitalWrite(engageLED,HIGH); 
              relayTime = ((millis() + 1000));
              //*****Turn saftey valve ON**********
              if (engageCAN == 1) digitalWrite(PWM2_RPWM, 1);       
            }
  
            else if ((VBusReceiveData.buf[1])== 0 && (VBusReceiveData.buf[2])== 125) //Tony Non MR Models? Ryan Mod to bit read engage bit
            {
               engageCAN = bitRead(VBusReceiveData.buf[0],2);
               Time = millis();
               digitalWrite(engageLED,HIGH); 
               relayTime = ((millis() + 1000));
               //*****Turn saftey valve ON**********
               if (engageCAN == 1) digitalWrite(PWM2_RPWM, 1);       
            }
          } 

          //**Work Message**
          if (VBusReceiveData.id == 0x1CFFE6D2)
          {
            if ((VBusReceiveData.buf[0])== 144)
            {
             workCAN = bitRead(VBusReceiveData.buf[6],0);
            }
          }
        }//End Brand == 0

        if (Brand == 1)
        {
            //**Current Wheel Angle & Valve State**
            if (VBusReceiveData.id == 0x0CAC1C13)
            {        
                estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
                steeringValveReady = (VBusReceiveData.buf[2]); 
            } 
  
            //**Engage Message**
            if (VBusReceiveData.id == 0x18EF1C32)
            {
                if ((VBusReceiveData.buf[0])== 15 && (VBusReceiveData.buf[1])== 96 && (VBusReceiveData.buf[2])== 1)
                {   
                    Time = millis();
                    digitalWrite(engageLED,HIGH); 
                    engageCAN = 1;
                    relayTime = ((millis() + 1000));
                }
            } 
        }//End Brand == 1   

        if (Brand == 2)
        {
          //**Current Wheel Angle & Valve State**
          if (VBusReceiveData.id == 0x0CACAA08)
          {        
                estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
                steeringValveReady = (VBusReceiveData.buf[2]); 
          } 
  
        }//End Brand == 2 

        if (Brand == 3)
        {
            //**Current Wheel Angle**
            if (VBusReceiveData.len == 8 && VBusReceiveData.buf[0] == 5 && VBusReceiveData.buf[1] == 10)
            {
                FendtEstCurve = (((int8_t)VBusReceiveData.buf[4] << 8) + VBusReceiveData.buf[5]);
                estCurve = FendtEstCurve + 32128;
            }
      
            //**Cutout CAN Message** 
            if (VBusReceiveData.len == 3 && VBusReceiveData.buf[2] == 0) steeringValveReady = 80;      // Fendt Stopped Steering So CAN Not Ready
    
        }//End Brand == 3  

        if (Brand == 4)
        {
            //**Current Wheel Angle & Valve State**
            if (VBusReceiveData.id == 0x0CACAB13)
            {        
                estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
                steeringValveReady = (VBusReceiveData.buf[2]); 
            }

            //**Engage Message**
            if (VBusReceiveData.id == 0x18EFAB27)
            {
                if ((VBusReceiveData.buf[0])== 15 && (VBusReceiveData.buf[1])== 96 && (VBusReceiveData.buf[2])== 1)
                {
                    Time = millis();
                    digitalWrite(engageLED,HIGH); 
                    engageCAN = 1;
                    relayTime = ((millis() + 1000));
                }
            }    
   
        }//End Brand == 4  

        if (Brand == 5)
        {
            //**Current Wheel Angle**
             if (VBusReceiveData.len == 8 && VBusReceiveData.buf[0] == 5 && VBusReceiveData.buf[1] == 10)
             {
                  FendtEstCurve = (((int8_t)VBusReceiveData.buf[4] << 8) + VBusReceiveData.buf[5]);
                  estCurve = FendtEstCurve + 32128;
             }
      
            //**Cutout CAN Message** 
             if (VBusReceiveData.len == 3 && VBusReceiveData.buf[2] == 0) steeringValveReady = 80;      // Fendt Stopped Steering So CAN Not Ready
    
        }//End Brand == 5 

        if (Brand == 6)
        {
          //**Current Wheel Angle & Valve State**
          if (VBusReceiveData.id == 0x0CACF013)
          {        
                estCurve = ((VBusReceiveData.buf[1] << 8) + VBusReceiveData.buf[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
                steeringValveReady = (VBusReceiveData.buf[2]); 
          } 
   
        }//End Brand == 6  

        if (Brand == 7)
        {
              //**Current Wheel Angle & Valve State**
              if (VBusReceiveData.id == 0x0CAC1C13)
              {        
                    steerAngleActual = float(int16_t(VBusReceiveData.buf[1] << 8| VBusReceiveData.buf[0])) / 100; 
                    steeringValveReady = (VBusReceiveData.buf[2]);
                    pwmDisplay = (VBusReceiveData.buf[3]);
                    pressureReading = (VBusReceiveData.buf[4]);
                    currentReading = (VBusReceiveData.buf[5]);
              }
        
        }//End Brand == 7 

        if (ShowCANData == 1)
        {
            Serial.print(Time);
            Serial.print(", V-Bus"); 
            Serial.print(", MB: "); Serial.print(VBusReceiveData.mb);
            Serial.print(", ID: 0x"); Serial.print(VBusReceiveData.id, HEX );
            Serial.print(", EXT: "); Serial.print(VBusReceiveData.flags.extended );
            Serial.print(", LEN: "); Serial.print(VBusReceiveData.len);
            Serial.print(", DATA: ");
            for ( uint8_t i = 0; i < 8; i++ ) 
            {
              Serial.print(VBusReceiveData.buf[i]); Serial.print(", ");
            }
  
            Serial.println("");
        }//End Show Data

    }//End if message 
}//End Receive V-Bus Void


//---Receive ISO_Bus message
void ISO_Receive()
{
    CAN_message_t ISOBusReceiveData;
    if (ISO_Bus.read(ISOBusReceiveData)) 
    { 
      //Put code here to sort a message out from ISO-Bus if needed 
  
      //**Work Message**
      if (ISOBusReceiveData.id == 0x0CFE45F0){
        ISORearHitch = (ISOBusReceiveData.buf[0]); 
        if(Brand != 7) pressureReading = ISORearHitch;
        if (steerConfig.PressureSensor == 1 && ISORearHitch < steerConfig.PulseCountMax && Brand != 7) workCAN = 1; 
        else workCAN = 0; 
      }
  
      if (Brand == 3)
      {
          if (ISOBusReceiveData.id == 0x18EF2CF0)   //**Fendt Engage Message**  
          {
            if ((ISOBusReceiveData.buf[0])== 0x0F && (ISOBusReceiveData.buf[1])== 0x60 && (ISOBusReceiveData.buf[2])== 0x01){   
              Time = millis();
              digitalWrite(engageLED,HIGH); 
              engageCAN = 1;
              relayTime = ((millis() + 1000));
            }
          }
      }

        if (ShowCANData == 1){
            Serial.print(Time);
            Serial.print(", ISO-Bus"); 
            Serial.print(", MB: "); Serial.print(ISOBusReceiveData.mb);
            Serial.print(", ID: 0x"); Serial.print(ISOBusReceiveData.id, HEX );
            Serial.print(", EXT: "); Serial.print(ISOBusReceiveData.flags.extended );
            Serial.print(", LEN: "); Serial.print(ISOBusReceiveData.len);
            Serial.print(", DATA: ");
            for ( uint8_t i = 0; i < 8; i++ ) 
            {
              Serial.print(ISOBusReceiveData.buf[i]); Serial.print(", ");
            }
  
            Serial.println("");
        }//End Show Data
  
    }
}

//---Receive K_Bus message
void K_Receive()
{
    CAN_message_t KBusReceiveData;
    if (K_Bus.read(KBusReceiveData)) { 
      //Put code here to sort a message out from K-Bus if needed 
  
      if (Brand == 3)
      {
        if (KBusReceiveData.buf[0]==0x15 && KBusReceiveData.buf[2]==0x06 && KBusReceiveData.buf[3]==0xCA)
        {
       
          if(KBusReceiveData.buf[1]==0x8A && KBusReceiveData.buf[4]==0x80) steeringValveReady = 80;      // Fendt Auto Steer Active Pressed So CAN Not Ready
      
          if (KBusReceiveData.buf[1]==0x88 && KBusReceiveData.buf[4]==0x80) // Fendt Auto Steer Go   
          {
              Time = millis();
              digitalWrite(engageLED,HIGH); 
              engageCAN = 1;
              relayTime = ((millis() + 1000));
          }
        }                                                             
      }

      if (Brand == 5)
      {
          if (KBusReceiveData.id == 0xCFFD899)   //**FendtOne Engage Message**  
          {
            if ((KBusReceiveData.buf[3])== 0xF6)
            {   
              Time = millis();
              digitalWrite(engageLED,HIGH); 
              engageCAN = 1;
              relayTime = ((millis() + 1000));
            }
          }
      }

      //CaseIH info from /buched Emmanuel
      if (Brand == 2)
      {
          if (KBusReceiveData.id == 0x14FF7706)   //**case IH Engage Message**  
          {
            if ((KBusReceiveData.buf[0])== 130 && (KBusReceiveData.buf[1])== 1)
            {   
              Time = millis();
              digitalWrite(engageLED,HIGH); 
              engageCAN = 1;
              relayTime = ((millis() + 1000));
            }

            if ((KBusReceiveData.buf[0])== 178 && (KBusReceiveData.buf[1])== 4)
            {   
              Time = millis();
              digitalWrite(engageLED,HIGH); 
              engageCAN = 1;
              relayTime = ((millis() + 1000));
            }
          }

          if (KBusReceiveData.id == 0x18FE4523)
          {
            KBUSRearHitch = (KBusReceiveData.buf[0]); 
            pressureReading = KBUSRearHitch;
            if (steerConfig.PressureSensor == 1 && KBUSRearHitch < steerConfig.PulseCountMax) workCAN = 1; 
            else workCAN = 0; 
          }
      }

        if (ShowCANData == 1)
        {
            Serial.print(Time);
            Serial.print(", K-Bus"); 
            Serial.print(", MB: "); Serial.print(KBusReceiveData.mb);
            Serial.print(", ID: 0x"); Serial.print(KBusReceiveData.id, HEX );
            Serial.print(", EXT: "); Serial.print(KBusReceiveData.flags.extended );
            Serial.print(", LEN: "); Serial.print(KBusReceiveData.len);
            Serial.print(", DATA: ");
            for ( uint8_t i = 0; i < 8; i++ ) 
            {
              Serial.print(KBusReceiveData.buf[i]); Serial.print(", ");
            }
  
            Serial.println("");
        }//End Show Data
   
    }
}

//Fendt K-Bus Buttons

void pressGo()
{
    CAN_message_t buttonData;
    buttonData.id = 0X613;
    buttonData.len = 8;
    for (uint8_t i = 0; i < sizeof(goPress); i++)
    {
        buttonData.buf[i] = goPress[i];
    }
    K_Bus.write(buttonData);
    goDown = true;
    Serial.println("Press Go");
}

void liftGo()
{
     CAN_message_t buttonData;
     buttonData.id = 0X613;
     buttonData.len = 8;
     for (uint8_t i = 0; i < sizeof(goLift); i++)
     {
         buttonData.buf[i] = goLift[i];
     }
     K_Bus.write(buttonData);
     goDown = false;
}

void pressEnd() {
     CAN_message_t buttonData;
     buttonData.id = 0X613;
     buttonData.len = 8;
     for (uint8_t i = 0; i < sizeof(endPress); i++)
     {
         buttonData.buf[i] = endPress[i];
     }
     K_Bus.write(buttonData);
     endDown = true;
     Serial.println("Press End");
}

void liftEnd(){
     CAN_message_t buttonData;
     buttonData.id = 0X613;
     buttonData.len = 8;
     for (uint8_t i = 0; i < sizeof(endLift); i++)
     {
         buttonData.buf[i] = endLift[i];
     }
     K_Bus.write(buttonData);
     endDown = false;
}

// CLAAS CSM buttons Start
void pressCSM1()
{                                     
     CAN_message_t buttonData;
     buttonData.id = 0x14204146;
     buttonData.flags.extended = true;
     buttonData.len = 8;
     for (uint8_t i = 0; i < sizeof(csm1Press); i++)
     {
         buttonData.buf[i] = csm1Press[i];
     }
     K_Bus.write(buttonData);
     goDown = true;
     Serial.println("Press CSM1");
}

void pressCSM2() {
     CAN_message_t buttonData;
     buttonData.id = 0x14204146;
     buttonData.flags.extended = true;
     buttonData.len = 8;
     for (uint8_t i = 0; i < sizeof(csm2Press); i++)
     {
         buttonData.buf[i] = csm2Press[i];
     }
    K_Bus.write(buttonData);
    endDown = true;
    Serial.println("Press CSM2");
}

//AgOpen CAN module
void canConfig(){

    CAN_message_t config251;
    config251.id = 0x19EF131C;
    config251.flags.extended = true;
    config251.len = 8;
    config251.buf[0] = 251;
    config251.buf[1] = uint8_t(steerSettings.wasOffset);
    config251.buf[2] = uint8_t(steerSettings.wasOffset >> 8);
      uint8_t sett0 = 0;
        if (steerConfig.InvertWAS == 1) bitSet(sett0,0); 
        if (steerConfig.IsRelayActiveHigh == 1) bitSet(sett0,1); 
        if (steerConfig.MotorDriveDirection == 1) bitSet(sett0,2); 
        if (steerConfig.SingleInputWAS == 1) bitSet(sett0,3); 
        if (steerConfig.CytronDriver == 1) bitSet(sett0,4); 
        if (steerConfig.SteerSwitch == 1) bitSet(sett0,5); 
        if (steerConfig.SteerButton == 1) bitSet(sett0,6);
        if (steerConfig.ShaftEncoder == 1) bitSet(sett0,7);  
    config251.buf[3] = sett0;
    config251.buf[4] = steerConfig.PulseCountMax;
      uint8_t sett1 = 0;
        if (steerConfig.IsDanfoss == 1) bitSet(sett1,0); 
        if (steerConfig.PressureSensor == 1) bitSet(sett1,1); 
        if (steerConfig.CurrentSensor == 1) bitSet(sett1,2);    
    config251.buf[5] = sett1;
    config251.buf[6] = 0;
    config251.buf[7] = 0;
    V_Bus.write(config251);
    
        
    CAN_message_t config252;
    config252.id = 0x19EF131C;
    config252.flags.extended = true;
    config252.len = 8;
    config252.buf[0] = 252;
    config252.buf[1] = uint8_t(steerSettings.Kp);
    config252.buf[2] = uint8_t(steerSettings.highPWM);
    config252.buf[3] = uint8_t(steerSettings.lowPWM);
    config252.buf[4] = uint8_t(steerSettings.minPWM);
    config252.buf[5] = uint8_t(steerSettings.steerSensorCounts);
    config252.buf[6] = uint8_t(steerSettings.AckermanFix * 100);
    config252.buf[7] = 0;
    V_Bus.write(config252);
}
