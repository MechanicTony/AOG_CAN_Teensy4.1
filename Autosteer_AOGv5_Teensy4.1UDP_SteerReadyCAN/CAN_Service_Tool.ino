
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
        else if ( b == 'Z') setupPVED();
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
  Serial.println("Z = Danfoss PVED parameter setup");
  Serial.println("    **GPS options**");
  Serial.println("Forwarding Mode: f = 115200, F = 460800");
  Serial.println("Panda Mode: p = 115200, P = 460800\r\n");
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
void setupPVED() {
    elapsedMillis parameterCheckTimer;
    static bool configPVED = true;
    static bool showMessage = false;
    static int ReadParameters = 0;
    int16_t S16Value = 9999;
    uint16_t U16Value = 9999;
    uint32_t U32Value = 9999;

    //Reset the CAN filters for config tool only
    V_Bus.setFIFOFilter(REJECT_ALL);
    V_Bus.setFIFOFilter(0, 0x18EFFD13, EXT);  
    V_Bus.setFIFOFilter(1, 0x18EFFC13, EXT);

    Serial.println("\r\nAgOpenGPS Danfoss PVED Config Mode:");
    Serial.println("Send X to exit and reset Teensy");
    Serial.println("Send S to print CAN message");
    Serial.println("Send s to stop printing message");
    Serial.println("Send R to read PVED parameters");
    Serial.println("Send W to change parameter 64007 to 30(Dec) 1E(Hex)");
    Serial.println("Send C to commit settings to PVED\r\n");

    Serial.println("Please power ON PVED valve...");

    while (Serial.available())
    {
        Serial.read();                //Clear the serial buffer
    }

    CAN_message_t ConfigData;
    while (V_Bus.read(ConfigData)) 
    {
        //Make sure theres no messages left
    }

    while (configPVED == true)
    {
        if (Serial.available())         // Read Data From Serail Monitor 
        {
            byte e = Serial.read();
            if (e == 'X')               //Exit Service Mode
            {
                Serial.println("Restarting Teensy...");
                delay(1000);
                SCB_AIRCR = 0x05FA0004; //Teensy Reset
            }
            else if (e == 'W') WriteParameters();   //Write Basic Parameters
            else if (e == 'C') Commit();            //Commit / Save Data
            else if (e == 'R') ReadParameters = 1;  //Get Basic Parameters
            else if (e == 'S')
            {
                showMessage = 1;
                Serial.println("Show CAN Data");
            }
            else if (e == 's')
            {
                showMessage = 0;
                Serial.println("Stop CAN Data");
            }
            else
            {
                Serial.println("No command, retry");
                Serial.println(" ");
                delay(50);
            }

            while (Serial.available())
            {
                Serial.read();                //Clear the serial buffer
            }
        }

        if (V_Bus.read(ConfigData))
        {
            if (showMessage == 1) 
            {
                Serial.print("V-Bus");
                Serial.print(", MB: "); Serial.print(ConfigData.mb);
                Serial.print(", ID: 0x"); Serial.print(ConfigData.id, HEX);
                Serial.print(", EXT: "); Serial.print(ConfigData.flags.extended);
                Serial.print(", LEN: "); Serial.print(ConfigData.len);
                Serial.print(", DATA: ");
                for (uint8_t i = 0; i < 8; i++) {
                    Serial.print(ConfigData.buf[i], DEC); Serial.print(", ");
                }
                Serial.println("");
            }

            //Valve current mode
            if ((ConfigData.buf[0] == 0x0F) && (ConfigData.buf[1] == 0xAA)) 
            {
                if ((ConfigData.buf[2] == 0x55)) Serial.println("PVED Awake In Calabration Mode");
                if ((ConfigData.buf[2] == 0xAA)) Serial.println("PVED Awake In Operation Mode");
            }

            //Config data reading
            if ((ConfigData.buf[0] == 0x0F) && ((ConfigData.buf[1] == 0xA1) || (ConfigData.buf[1] == 0xA3))) 
            {

                if ((ConfigData.buf[2] == 0xFC) && (ConfigData.buf[3] == 0x01)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("508 Kp Gain = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xC2) && (ConfigData.buf[3] == 0x02)) {
                    U16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("706 Vcap = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xC3) && (ConfigData.buf[3] == 0x02)) {
                    U16Value = (((uint8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("707 StrkVol (Cyd size) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xD9) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("729 Xspl_1000 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xE1) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("737 Xspl_0 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }


                else if ((ConfigData.buf[2] == 0xE2) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("738 Xspr_0 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xEB) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("747 Xspr_1000 = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xEC) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("748 Closed Loop Xsp Offset = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xF6) && (ConfigData.buf[3] == 0x02)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("758 Xsp Cal Offset = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0x03) && (ConfigData.buf[3] == 0x04)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("1027 (Qm) Max Port Flow Faststeer = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0xA3) && (ConfigData.buf[3] == 0x13)) {
                    S16Value = (((int8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("5027 (Qm) Max Port Flow Autosteer = ");
                    Serial.print(S16Value);
                    Serial.println(" ");
                }

                else if ((ConfigData.buf[2] == 0x07) && (ConfigData.buf[3] == 0xFA)) {
                    Serial.print("64007 (Setpoint Controller Address) = ");
                    Serial.print(ConfigData.buf[4], DEC);
                    Serial.print(",DEC ");
                    Serial.print(ConfigData.buf[4], HEX);
                    Serial.print(",HEX ");
                    Serial.println(" ");
                }

                //64022
                else if ((ConfigData.buf[2] == 0x16) && (ConfigData.buf[3] == 0xFA)) {
                    U16Value = (((uint8_t)ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("64022 SASA Cutout Speed = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //64023
                else if ((ConfigData.buf[2] == 0x17) && (ConfigData.buf[3] == 0xFA)) {
                    U32Value = (ConfigData.buf[7]);
                    U32Value = ((U32Value << 8) + (ConfigData.buf[6]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[5]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[4]));
                    Serial.print("64023 SASA Timeout = ");
                    Serial.print(U32Value);
                    Serial.println(" ");
                }

                //65080
                else if ((ConfigData.buf[2] == 0x38) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("AD1_1000_Left (WAS Left Raw Counts) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //65083
                else if ((ConfigData.buf[2] == 0x3B) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("AD1_1000_Right (WAS Right Raw Counts) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //65086
                else if ((ConfigData.buf[2] == 0x3E) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("AD1_Neutral (WAS Centre Raw Counts) = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                //65099
                else if ((ConfigData.buf[2] == 0x4B) && (ConfigData.buf[3] == 0xFE)) {
                    U32Value = (ConfigData.buf[7]);
                    U32Value = ((U32Value << 8) + (ConfigData.buf[6]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[5]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[4]));
                    Serial.print("True Angle Left = ");
                    Serial.print(U32Value);
                    Serial.println(" ");
                }

                //65100
                else if ((ConfigData.buf[2] == 0x4C) && (ConfigData.buf[3] == 0xFE)) {
                    U32Value = (ConfigData.buf[7]);
                    U32Value = ((U32Value << 8) + (ConfigData.buf[6]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[5]));
                    U32Value = ((U32Value << 8) + (ConfigData.buf[4]));
                    Serial.print("True Angle Right = ");
                    Serial.print(U32Value);
                    Serial.println(" ");
                }

                //65101
                else if ((ConfigData.buf[2] == 0x4D) && (ConfigData.buf[3] == 0xFE)) {
                    Serial.print("SASA Present = ");
                    Serial.print(ConfigData.buf[4],DEC);
                    Serial.println(" ");
                }

                //65104
                else if ((ConfigData.buf[2] == 0x50) && (ConfigData.buf[3] == 0xFE)) {
                    Serial.print("SASA Present = ");
                    Serial.print(ConfigData.buf[4], DEC);
                    Serial.println(" ");
                }

                //65112
                else if ((ConfigData.buf[2] == 0x58) && (ConfigData.buf[3] == 0xFE)) {
                    U16Value = ((ConfigData.buf[5] << 8) + ConfigData.buf[4]);
                    Serial.print("65112 Vehicle Length = ");
                    Serial.print(U16Value);
                    Serial.println(" ");
                }

                else  {
                    Serial.println("Im not sure ??");
                }

            }
            Serial.println(" ");

        }

        //Get Basic Parameters
        if (ReadParameters != 0) 
        {

            static CAN_message_t msgR;

            if (ReadParameters == 1) {
                Serial.println("Get Parameter 1/22 508");
                msgR.id = 0x98EF13FD;
                msgR.flags.extended = true;
                msgR.len = 8;
                msgR.buf[0] = 0x0F;
                msgR.buf[1] = 0xA0;
                msgR.buf[2] = 0xFC;
                msgR.buf[3] = 0x01;
                msgR.buf[4] = 0x00;
                msgR.buf[5] = 0x00;
                msgR.buf[6] = 0x00;
                msgR.buf[7] = 0x00;
                V_Bus.write(msgR);
                ReadParameters = 2;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 2 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 2/22 706");
                msgR.buf[2] = 0xC2;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 3;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 3 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 3/22 707");
                msgR.buf[2] = 0xC3;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 4;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 4 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 4/22 729");
                msgR.buf[2] = 0xD9;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 5;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 5 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 5/22 737");
                msgR.buf[2] = 0xE1;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 6;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 6 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 6/22 738");
                msgR.buf[2] = 0xE2;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 7;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 7 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 7/22 747");
                msgR.buf[2] = 0xEB;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 8;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 8 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 8/22 748");
                msgR.buf[2] = 0xEC;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 9;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 9 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 9/22 758");
                msgR.buf[2] = 0xF6;
                msgR.buf[3] = 0x02;
                V_Bus.write(msgR);
                ReadParameters = 10;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 10 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 10/22 1027");
                msgR.buf[2] = 0x03;
                msgR.buf[3] = 0x04;
                V_Bus.write(msgR);
                ReadParameters = 11;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 11 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 11/22 5027");
                msgR.buf[2] = 0xA3;
                msgR.buf[3] = 0x13;
                V_Bus.write(msgR);
                ReadParameters = 12;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 12 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 12/22 64007");
                msgR.buf[2] = 0x07;
                msgR.buf[3] = 0xFA;
                V_Bus.write(msgR);
                ReadParameters = 13;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 13 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 13/22 64022");
                msgR.buf[2] = 0x16;
                msgR.buf[3] = 0xFA;
                V_Bus.write(msgR);
                ReadParameters = 14;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 14 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 14/22 64023");
                msgR.buf[2] = 0x17;
                msgR.buf[3] = 0xFA;
                V_Bus.write(msgR);
                ReadParameters = 15;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 15 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 15/22 65080");
                msgR.buf[2] = 0x38;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 16;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 16 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 16/22 65083");
                msgR.buf[2] = 0x3B;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 17;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 17 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 17/22 65086");
                msgR.buf[2] = 0x3E;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 18;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 18 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 18/22 65099");
                msgR.buf[2] = 0x4B;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 19;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 19 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 19/22 65100");
                msgR.buf[2] = 0x4C;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 20;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 20 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 20/22 65101");
                msgR.buf[2] = 0x4D;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 21;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 21 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 21/22 65104");
                msgR.buf[2] = 0x50;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 22;
                parameterCheckTimer = 0;
            }

            else if (ReadParameters == 22 && parameterCheckTimer > 1000) {
                Serial.println("Get Parameter 22/22 65112");
                msgR.buf[2] = 0x58;
                msgR.buf[3] = 0xFE;
                V_Bus.write(msgR);
                ReadParameters = 0;
                parameterCheckTimer = 0;
            }
        }
    }
}

//**************************************************************************************

void WriteParameters()
{
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
    
    delay(100);
    Serial.println("Sent parameter change request, make sure you send commit command next");
}

//**************************************************************************************

void Commit()
{
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
    Serial.println("Commiting data wait 1min, then turn Off PVED valve & restart");
}

