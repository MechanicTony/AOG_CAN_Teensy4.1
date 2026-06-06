# AOG_CAN_Teensy4.1
Steering navagation controller files for Teensy based boards to use with AgOpenGPS 

If your new to the teensy this is what ive had to do:
1. You need to update the Arduino IDE
2. Install the Teensy program
3. The new updated Teensy program 1.55 now inclucdes the latest FlexCAN_T4 & Native Ethernet so you no longer have to update manually.

CaseIH Connections:  
CAN1-H = Pin C Diagnostic connector right rear pillar  
CAN1-L = Pin D Diagnostic connector right rear pillar  
CAN3-H = Pin 3 Nav controller 40 pin connector X-716 or Pin H Diagnostic connector behind seat    
CAN3-L = Pin 13 Nav controller 40 pin connector X-716 or Pin J Diagnostic connector behind seat  

[CaseIH_GPS.pdf](https://github.com/MechanicTony/AOG_CAN_Teensy4.1/files/10528432/CaseIH_GPS.pdf)

Deutz Warrior
CAN1 = Pin CD Diagnostic connector
CAN3 = Pin EF Diagnostic connector