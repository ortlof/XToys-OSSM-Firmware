# OSSM Firmware for XToys
Firmware for use with the OSSM (Open Source Sex Machine) by KinkyMakers.
[https://github.com/KinkyMakers/OSSM-hardware](https://github.com/KinkyMakers/OSSM-hardware)

# !!!!!!!!!!! Don't Use this. This is a stopped work in Progress its Dangerous. !!!!!!!!!!!!!!!!








## Setting Up
1. Clone Repo localy. 
2. Setup Platformio see [OSSM GIT] https://github.com/KinkyMakers/OSSM-hardware 
3. Setup Your Settings like for OSSM. maxStrokeLengthMm 
3. Flash Firmware. 
4. In [XToys](https://xtoys.app) click the + button, add the OSSM toy, and click the bluetooth connect button.
5. Local Control & Webcontrol is disabled as Bluetooth is Connected.
6. In the setup dialog connect to the OSSM. Don't Change Settings in the Xtoys app. 

## Bluetooth Info
This firmware uses the following Bluetooth info:  
Device Identifier: OSSM  
Service: e556ec25-6a2d-436f-a43d-82eab88dcefd  
Control Characteristic: c4bee434-ae8f-4e67-a741-0607141f185b (used for sending T-Code commands)  
Settings Characteristic: fe9a02ab-2713-40ef-a677-1716f2c03bad (used for adjusting OSSM settings)  

## Control Commands
Supported T-Code commands to send to Control characteristic:  
DSTOP - Stop OSSM immediately (leaves motor enabled)  
DENABLE - Enables motor (non-standard T-Code command)  
DDISABLE - Disable motor (non-standard T-Code command)  
L199I100 - Linear actuator move commands with a position + speed  
L199I100C - Appending 'C' to a command causes the OSSM to clear any pending movement commands (non-standard T-Code command)
