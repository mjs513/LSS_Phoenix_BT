//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kare Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
//
//
// Phoenix.h - This is the first header file that is needed to build
//          a Phoenix program for a specific Hex Robot.
//
//
// This file assumes that the main source file either directly or through include
// file has defined all of the configuration information for the specific robot.
// Each robot will also need to include:
//  
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================
#ifndef _PHOENIX_DRIVER_BIOLOID_H_
#define _PHOENIX_DRIVER_BIOLOID_H_
#include "phoenix_float.h"
#include "Hex_Cfg.h"

class LSSServoDriver  : public ServoDriver {
public:
  void Init(void);

  word GetBatteryVoltage(void);

  void setGaitConfig();  //kludge MJS

  void            BeginServoUpdate(void);    // Start the update 
#ifdef c4DOF
  void            OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
#else
  void            OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
#endif    
#ifdef cTurretRotPin
  void            OutputServoInfoForTurret(short sRotateAngle1, short sTiltAngle1);
#endif
  void            CommitServoDriver(word wMoveTime);
  void            FreeServos(void);
  
  void            IdleTime(void);        // called when the main loop when the robot is not on
  void            showUserFeedback(int feedback_state); 

  // Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
  void            BackgroundProcess(void);
#endif    

#ifdef OPT_TERMINAL_MONITOR  
  void            ShowTerminalCommandList(void);
  boolean         ProcessTerminalCommand(byte *psz, byte bLen);
#endif

  enum {
    #ifdef c4DOF
      NUMSERVOSPERLEG = 4,
    #else
      NUMSERVOSPERLEG = 3,
    #endif
    #ifdef cTurretRotPin
      NUMSERVOS = NUMSERVOSPERLEG*CNT_LEGS +2
    #else
      NUMSERVOS = (NUMSERVOSPERLEG*CNT_LEGS)
    #endif
  };
  
  
  //=====================================================================
  // added stuff for bioloid like pose support to bypass servo
  //    firmware support for timed moves
  //=====================================================================
  enum {MAX_MOVE_SERVOS = 20, DEFAULT_FRAMES_PER_SECOND = 50, DEFAULT_MIN_NOT_WAIT_TIME = 1000};

  typedef struct {
    uint8_t id;   // id of servo
    int16_t target_pos; // our target position
    int16_t starting_pos; // our target position
    float   pos;        // our current working position
    float   cycle_delta;      // how much to change per cycle
  } tm_servo_t;


  // Again quick and dirty
  tm_servo_t tmServos[MAX_MOVE_SERVOS];
  uint32_t      tmCycleTime = 1000000l/DEFAULT_FRAMES_PER_SECOND;
  uint32_t      tmMinNotwaitTime = DEFAULT_MIN_NOT_WAIT_TIME; 
  elapsedMicros tmTimer;
  uint32_t      tmMovetime = 0;
  uint32_t      tmCyclesLeft = 0;
  uint8_t       tmServoCount = 0;
  bool          tmSetupServos = true;
  bool          use_servos_timed_moves = false;
  bool          servo_debug = false;

  // functions for
  void TMReset();
  uint8_t TMAddID(uint8_t id);
  void TMInitWithCurrentservoPositions();
  bool TMSetTargetByID(uint8_t id, int16_t target);
  void TMSetTargetByIndex(uint8_t index, int16_t target);
  void TMSetupMove(uint32_t move_time);
  int  TMStep(bool wait = true);
  void TMPrintDebugInfo();
  void TMConfigureServos();

private:
  void MakeSureServosAreOn(void);
  void TCSetServoID(byte *psz);
  void TCTrackServos();
  void TCServoPositions();
  void FindServoOffsets();
   void WakeUpRoutine(void);


  boolean _fServosFree;    // Are the servos in a free state?
 
} 
; 

#endif //_PHOENIX_DRIVER_BIOLOID_H_
