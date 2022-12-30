
// Danfoss PVED-CL Service Tool for use with AgOpenGPS
// The Danfoss PVED-CL parts are not in here yet, but will be copied over soon

//Ryan Claas addons
// 30-12-2022  Added PVED-CL parameter 64007 change code to CAN Service Tool page. Use to change CLAAS PVED-CL parameter 64007
//             Add CLAAS pre Mother reg tractor CSM button messages on main Autosteer_AOG_Teensy4.1 page line 156-160 Comment out lines not required to select pre MR or MR/Stage5

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

void Service_Tool (void) 
{
  Serial.println("\r\nAgOpenGPS CANBUS Service Tool Mode:");
  Help();
  
  while (Service == 1) 
  {
  
      if (Serial.available())   // Read Data From Serail Monitor 
      {    
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
        else if (b == 'Z') PVED64007();
        else if ( b == 'f') gpsModeOne();
        else if ( b == 'F') gpsModeTwo();
        else if ( b == 'p') gpsModeThree();
        else if ( b == 'P') gpsModeFour();

        else
        {
          Serial.println("No command, send ? for help");
          Serial.println(" ");
          delay(50);
        }

        while (Serial.available())
        {
        Serial.read();                //Clear the serial buffer
        }
      }

      if (tempChecker > 10000)
      {
          tempChecker = 0;
          float temp = tempmonGetTemp();
          Serial.print(temp, 2);
          Serial.println(" degC CPU Temp");
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
  Serial.println("Z = Change PVED parameter 64007 to 30");
  Serial.println("Forwarding Mode: f = 115200, F = 460800");
  Serial.println("Panda Mode: p = 115200, P = 460800");
  Serial.println(" ");
}

//**************************************************************************************
void Claas(){
  EEPROM.update(70,0); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Claas, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}

//**************************************************************************************
void Valtra(){
  EEPROM.update(70,1); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Valtra, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}

//**************************************************************************************
void CNH(){
  EEPROM.update(70,2); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set CaseIH / New Holland, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}

//**************************************************************************************
void Fendt(){
  EEPROM.update(70,3); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Fendt SCR/S4/Gen6, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void JCB(){
  EEPROM.update(70,4); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set JCB, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void FendtOne(){
  EEPROM.update(70,5); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set FendtOne, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void Lindner(){
  EEPROM.update(70,6); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set Lindner, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void AgOpenGPS(){
  EEPROM.update(70,7); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set AgOpenGPS, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
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
//**************************************************************************************
void gpsModeOne() {
    EEPROM.update(72, 1);
    Serial.println("GPS Forwarding @ 115200, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeTwo() {
    EEPROM.update(72, 2);
    Serial.println("GPS Forwarding @ 460800, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeThree() {
    EEPROM.update(72, 3);
    Serial.println("GPS Panda @ 115200, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeFour() {
    EEPROM.update(72, 4);
    Serial.println("GPS Panda @ 460800, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void PVED64007() {
    // Change parameter 64007 to 30(Dec) 1E(Hex)
    CAN_message_t msgP;
    msgP.id = 0x98EF13FD;
    msgP.flags.extended = true;
    msgP.len = 8;
    msgP.buf[0] = 0x0F;
    msgP.buf[1] = 0xA2;
    msgP.buf[2] = 0x07;
    msgP.buf[3] = 0xFA;
    msgP.buf[4] = 0x1E;
    msgP.buf[5] = 0x00;
    msgP.buf[6] = 0x00;
    msgP.buf[7] = 0x00;
    V_Bus.write(msgP);
    Serial.println("sent parameter change request");
    delay(100);

    // Commit parameters to PVED-CL  
    CAN_message_t msgC;
    msgC.id = 0x98EF13FD;
    msgC.flags.extended = true;
    msgC.len = 8;
    msgC.buf[0] = 0x0F;
    msgC.buf[1] = 0xA6;
    msgC.buf[2] = 0x5A;
    msgC.buf[3] = 0x00;
    msgC.buf[4] = 0x00;
    msgC.buf[5] = 0x00;
    msgC.buf[6] = 0x00;
    msgC.buf[7] = 0x00;
    V_Bus.write(msgC);

    delay(1000);
    Serial.println("parameters commited, reboot PVED");
}
