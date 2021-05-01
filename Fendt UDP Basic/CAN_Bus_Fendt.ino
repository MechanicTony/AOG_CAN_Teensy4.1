
//---Start Teensy CANBUS Ports and Claim Addresses

void CAN_setup (void) {

//V_Bus is CAN-3 
  V_Bus.begin();
  V_Bus.setBaudRate(250000);
  V_Bus.enableFIFO();
  V_Bus.setFIFOFilter(REJECT_ALL);
  V_Bus.setFIFOFilter(0, 0x0CEF2CF0, EXT);  //Fendt Data To Navagation Controller
  
  CAN_message_t msgV;
  msgV.id = 0x18EEFF2C;
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

//ISO_Bus is CAN-2 
  ISO_Bus.begin();
  ISO_Bus.setBaudRate(250000);
  ISO_Bus.enableFIFO();
  ISO_Bus.setFIFOFilter(REJECT_ALL);
  ISO_Bus.setFIFOFilter(0, 0x18EF2CF0, EXT);  //Fendt Data To Navagation Controller
  
  CAN_message_t msgISO;
  msgISO.id = 0x18EEFF2C;
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

delay (500);   
}

//---Send V_Bus message

void VBus_Send(){
  
    CAN_message_t VBusSendData;
    VBusSendData.id = 0x0CEFF02C;
    VBusSendData.flags.extended = true;
    VBusSendData.len = 6;
    VBusSendData.buf[0] = 5;
    VBusSendData.buf[1] = 9;
    if (intendToSteer == 1)VBusSendData.buf[2] = 3;
    if (intendToSteer == 0)VBusSendData.buf[2] = 2;
    VBusSendData.buf[3] = 10;
    if (intendToSteer == 1){
      VBusSendData.buf[4] = highByte(setCurve*10);
      VBusSendData.buf[5] = lowByte(setCurve*10);
    }
    else{
      VBusSendData.buf[4] = 0;
      VBusSendData.buf[5] = 0;
    }
    V_Bus.write(VBusSendData);

//Fendt Ask For Steering State - Send With Curve Command
  CAN_message_t valveState;
  valveState.id = 0x0CEFF02C;
  valveState.flags.extended = true;
  valveState.len = 2;
  valveState.buf[0] = 0x5;
  valveState.buf[1] = 0x19;
  V_Bus.write(valveState);
}

//---Receive V_Bus message

void VBus_Receive(){
  CAN_message_t VBusReceiveData;
if (V_Bus.read(VBusReceiveData)) {

//**Current Wheel Angle**
 if (VBusReceiveData.len == 8 && VBusReceiveData.buf[0] == 5 && VBusReceiveData.buf[1] == 10){
      estCurve = (((int8_t)VBusReceiveData.buf[4] << 8) + VBusReceiveData.buf[5]);
      estCurve = (estCurve / 10);
      }
      
//**Cutout CAN Message** 
 if (VBusReceiveData.len == 3 && (VBusReceiveData.buf[2] == 0)){
  cutoutCAN = 1;
  }
    
 }
}

//---Receive ISO_Bus message

void ISO_Receive(){
    CAN_message_t ISOBusReceiveData;
if (ISO_Bus.read(ISOBusReceiveData)) {
  
//**Engage Message**
  if ((ISOBusReceiveData.buf[0])== 0x0F || (ISOBusReceiveData.buf[1])== 0x60){
    if ((ISOBusReceiveData.buf[2])== 0x01){
      Time = millis();
      digitalWrite(engageLED,HIGH); 
      engageCAN = 1;
      relayTime = ((millis() + 1000));
   }
  } 
  
 }
}
