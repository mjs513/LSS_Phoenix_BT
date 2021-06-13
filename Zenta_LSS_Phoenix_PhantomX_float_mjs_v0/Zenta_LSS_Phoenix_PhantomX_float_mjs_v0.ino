
// Warning setup to build for standard hexapod or for quad.
//  #define QUADMODE
//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//   Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Arbotix Robocontroller board
//
//=============================================================================
// Warning:: This configuration does not check voltages, so you should be careful to
// not allow the lipo to discharge too far.
//
// This configuration should hopefully run on a stock PhantomX, without any
// of my changes.
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#include <Arduino.h>
#include <EEPROM.h>
#include <avr\pgmspace.h>
#include "Hex_Cfg.h"

#include "phoenix_float.h"
#include "USBPSXController.h"
//#include "phoenix_input_DIY_Commander.h"
#include "phoenix_driver_bioloid.h"

// We are using the commander.
//CommanderInputController commander;
USBPSXController usbControl;

// Using Bioloid:
//DynamixelServoDriver dxlServo;
LSSServoDriver lssServo;

#include "logo.h"
#ifdef USE_ST7789
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_RST);
#endif


void SketchSetup() {
#ifdef USE_ST7789
#ifndef TFT_MODE
    tft.init(TFT_WIDTH, TFT_HEIGHT);
#else
    tft.init(TFT_WIDTH, TFT_HEIGHT, TFT_MODE);
#endif
#ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWriteFast(TFT_BL, HIGH);
#endif
    // have some fun display logo
    tft.fillScreen(ST77XX_RED);
    tft.writeRect((tft.width() - LOGO_WIDTH) / 2, 0, LOGO_WIDTH, LOGO_HEIGHT, (uint16_t*)lynxmotion_logo);
#endif

    InputController::controller(usbControl);
    //g_InputController = &usbControl;
    LSSServoDriver::driver(lssServo);

}
