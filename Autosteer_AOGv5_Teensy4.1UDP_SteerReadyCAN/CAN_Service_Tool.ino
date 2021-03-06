
// Danfoss PVED-CL Service Tool for use with AgOpenGPS
// The Danfoss PVED-CL parts are not in here yet, but will be copied over soon

//----------------------------------------------------------
//Notes from Ryan (When Changing Claas Parameter 64007) 
//
//Mother Regulated CLAAS tractors:
//Should work on Axion A64, A44, A51, A61, A60, A50, 
//Arion A97, A96, A95, A94, A74, A76, A75, A77
//Only tested on Axion A60
//
//Parameter 64007(external set point generator address, AKA GPS)
//needs to be changed to 30 (dec) from 27 (dec) then commited to memory. Power to PEVD-CL must then be cycled
//before PVED will send/receive AOG teensy AD,AC messages.
//
//Work Switch is triggerd via CAN, set work triggers in CEBIS implement menu
//and make sure a task in CEBIS is playing 
//
//CAUTION
//This is a modification to the tractor standard safety checks
//Operator is responsible for ensuring ATP switch is off when not using Autosteer
//This modification will most likley disable dynamic (varable ratio)steering
//and CEBIS gives a warning when CLAAS engage button is pressed but it disapears after a second (MR Models)
//
//Pre Mother Regulated tractors:
//Arion A36,A37,A34,A35, Axion A23,A40,A41
//
//Need to make wiring to connect to tractor steering CAN at ATP module
//Wiring will make CLAAS steer rocker switch power PVED-CL directly along with safety/reaction valve
//could also wire so safety/reaction valve is powered by PCB only when autosteer is actualy steering
//
//---------------------------------------------------------

void Service_Tool (void) {
  Serial.println("\r\nAgOpenGPS CANBUS Service Tool Mode:");
  Help();
  
  while (Service == 1) {
  
  if (Serial.available()){    // Read Data From Serail Monitor 
    byte b = Serial.read();
    if ( b == '?') Help();          
    else if ( b == 'X') Service = 0; //Exit Service Mode
    else if ( b == '0') Claas();
    else if ( b == '1') Valtra();
    else if ( b == '2') CNH();
    else if ( b == '3') Fendt();
    else if ( b == '4') JCB();
    else if ( b == '5') FendtOne();
    else if ( b == '6') Lindner();
    else if ( b == '7') AgOpenGPS();
    else if ( b == 'R') ReadCAN();
    else if ( b == 'S') StopCAN();

    else{
    Serial.println("No command, send ? for help");
    Serial.println(" ");
    delay(50);
  }
  while (Serial.available()){
  Serial.read();                //Clear the serial buffer
 }
}

 }
}

//**************************************************************************************
void Help(){
  Serial.println("? = Help");
  Serial.println("X = Exit Service Mode");
  Serial.println("0 = Set Brand as Claas");
  Serial.println("1 = Set Brand as Valtra / Massey");
  Serial.println("2 = Set Brand as CaseIH / New Holland");
  Serial.println("3 = Set Brand as Fendt SCR, S4, Gen6");
  Serial.println("4 = Set Brand as JCB");
  Serial.println("5 = Set Brand as FendtOne");
  Serial.println("6 = Set Brand as Lindner");
  Serial.println("7 = Set Brand as AgOpenGPS");
  Serial.println("R = Show CAN Data");
  Serial.println("S = Stop Data");
  Serial.println(" ");
}

//**************************************************************************************
void Claas(){
  EEPROM.update(70,0); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Claas, Power Off Teensy");
  Serial.println(" ");
}

//**************************************************************************************
void Valtra(){
  EEPROM.update(70,1); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Valtra / Massey, Power Off Teensy");
  Serial.println(" ");
}

//**************************************************************************************
void CNH(){
  EEPROM.update(70,2); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set CaseIH / New Holland, Power Off Teensy");
  Serial.println(" ");
}

//**************************************************************************************
void Fendt(){
  EEPROM.update(70,3); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Fendt SCR/S4/Gen6, Power Off Teensy");
  Serial.println(" ");
}
//**************************************************************************************
void JCB(){
  EEPROM.update(70,4); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set JCB, Power Off Teensy");
  Serial.println(" ");
}
//**************************************************************************************
void FendtOne(){
  EEPROM.update(70,5); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set FendtOne, Power Off Teensy");
  Serial.println(" ");
}
//**************************************************************************************
void Lindner(){
  EEPROM.update(70,6); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Lindner, Power Off Teensy");
  Serial.println(" ");
}
//**************************************************************************************
void AgOpenGPS(){
  EEPROM.update(70,7); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set AgOpenGPS, Power Off Teensy");
  Serial.println(" ");
}
//**************************************************************************************
void ReadCAN(){
ShowCANData = 1;
  Serial.println("CAN Data ON, Send X To Exit Service Tool");
  Serial.println(" ");
}
//**************************************************************************************
void StopCAN(){
ShowCANData = 0;
  Serial.println("CAN Data OFF, Send X To Exit Service Tool");
  Serial.println(" ");
}
 
