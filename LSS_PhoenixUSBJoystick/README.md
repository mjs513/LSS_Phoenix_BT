# LSS_Phoenix_BT

## Menu Commands

Command | Description
------------ | -------------
D | Toggle debug on or off
J | Show Joystick data
V | Voltage
M | Toggle Motors on or off
T | Test Servos
P | Servo Positions
A | Toggle LSS speed control
L | Toggle LSS Servo Debug output
F <FPS> | Set FPS for Interpolation mode
S | Track Servos
O | Enter Servo offset mode
C | clear Servo Offsets
  
## Servo Offset Mode
After 'O' command is enter you will see on the screen a brief description and the current settings for the servos and the sub menu:
  
 Update Servos Offsets and their rotation direction(Gyre)
 Current Servo Information
  
Servo |  Position | Offset | Gyre | EM | Filter | Ang. Stiffness | Ang Hold Stiffness | Ang Range
------------ | ------------- | ------------ | ------------- | ------------ | ------------- | ------------ | ------------- | ------------ 
Servo: RR Coxa(8) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RM Coxa(14) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RF Coxa(2) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LR Coxa(7) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LM Coxa(13) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LF Coxa(1) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RR Femur(10) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RM Femur(16) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RF Femur(4) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LR Femur(9) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LM Femur(15) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LF Femur(3) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RR Tibia(12) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RM Tibia(18) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: RF Tibia(6) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LR Tibia(11) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LM Tibia(17) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0
Servo: LF Tibia(5) | Pos:0 | O:0:0 | G:0:0 | EMC:0 | FPC:0:0 | AS:0:0 | AH:0:0 | AR:0:0

 The Goal is to align the top two servo pivots (Coxa and Femur) to be parallel to ground
 And the Tibia should be at a right angle to the ground

 Enter $-Exit, +- changes, *-change servo
    0-n Chooses a leg, C-Coxa, F-Femur, T-Tibia
    m - manually move mode to get close
 Servo: RR Coxa(8)
  
 NOTE: The numbering is based on the index for the coxa, femur, and tiberia.  For example if you want to change the RF Femur you would enter 'F 2' since its the 3 servo in the femur list and the indexing starts at 0
  
#### Joystick Commands

Command | Description
------------ | -------------
PS | Power On/Off
Cross | Joystick Debug
Up Arrow | Stand/Start Position
Dwn Arrow | Sit Position
Left Arrow | Slow Down Gait
Right Arrow | Speed Up Gait
Squate | Balance Mode On/Off
Triangle | Motion Select
... | Translate
... | Rotate
... | Single
... | Walk
L1 | Leg Controls
... | Ry - Up/Dwn
... | Rx - Speed Up/Dwn
--- | Lx/Ly  manually adjust the initial leg positions
R1 | Hold Single leg in place when Single leg selected

### Available Commands in Certain Motions Selected
  ### Single Leg Motion 
Command | Description
------------ | -------------
Select | Left or right front leg
  
  ### Walk Motion
Command | Description
------------ | -------------
Options | Gait Select
... | Tripod 8
... | Tripple 12
... | Tripple 16
... | Wave 24
... | Tripod 6
... |  Ripple 12
R2 | Double Leg Lift
Circle | Walkmode 1 (Auto)
... | Circle | Walkmode 0 (Manual Control)

  ### Rotate Motion
Command | Description
------------ | -------------
X axis | Use Ly stick
Y axis | Use Rx stick
Z axis | Use Lx stick
  
  ### Translate Motion
Command | Description
------------ | -------------
X axis | Use Lx stick
Y axis | Use Rx stick
Z axis | Use Ly stick

  
