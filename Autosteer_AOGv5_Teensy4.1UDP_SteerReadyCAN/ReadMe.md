*Version 03.02.2022*

-Any GPS data on Serial3 @ 115200 (Dual or Single) is forwarded to AgIO via UDP  
-Ntrip is forwarded from AgIO (Port 2233) to Serial3  
-BNO08x/CMPS14 Data sent as IMU message (Not in Steering Message), timmed from steering message from AgOpen  
-IMU sampled at 100hz (every 10msec)  

-This CAN setup is for factory CANBUS based steering controllers as below:  
-Danfoss PVED-CL & PVED-CLS (Claas, JCB, Massey Fergerson, CaseIH, New Holland, Valtra, Deutz, Lindner)  
-Fendt SCR, S4, Gen6, FendtOne Models need Part:ACP0595080 3rd Party Steering Unlock Installed  
-Late model Valtra & Massey with PVED-CC valve (Factory steering controller in main tractor ECU)  
-!!Model is selected via serial monitor service tool!! (One day we will will get a CANBUS setup page in AgOpen)  

-For engage & disengage via CAN or Button on PCB, select "Button" as switch option in AgOpen   
-For engage via AgOpen tablet & disengage via CAN, select "None" as switch option and make sure "Remote" is on the steering wheel icon  
-For engage & disengage via PCB switch only select "Switch" as switch option  

-PWM value drives set curve up & down, so you need to set the PWM settings in AgOpen  
-Normal starting settings P=15, Max=254, Low=5, Min=1  
-Some tractors have very fast valves, this smooths out the setpoint from AgOpen  

-Workswitch can be operated via PCB or CAN (May need to setup CAN Messages in ISOBUS section)  
-17.09.2021 - If Pressure Sensor selected, Work switch will be operated when hitch is less than pressure setting (0-250 x 0.4 = 0-100%)   
              Note: The above is temporary use of unused variable, as one day we will get hitch % added to AgOpen  

-Fendt K-Bus - (Not FendtOne models)  
-Big Go/End is operated via hitch control in AgOpen   
-Arduino Hitch settings must be enableded and sent to module  
-"Invert Relays" Uses section 1 to trigger hitch (Again temporary until we get button added to AgOpen)  
