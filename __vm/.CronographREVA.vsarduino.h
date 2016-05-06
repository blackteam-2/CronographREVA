/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Uno, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 105
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
extern "C" void __cxa_pure_virtual() {;}

//
//
void programModeNormal();
void programModeAverage();
void programModeRPS();
void programModeAutotune();
void programModeSetup();
void programModeSelect();
void hardError(int errorCode);
void resetAverage();
float convertToFPS(float data);
void calculateAverage();
void updateDisplay(int mode, boolean data);
void saveSettingsToEEPROM();
void loadSettingsFromEEPROM();
void saveBBWeightToEEPROM();
void saveProgramModeToEEPROM();
void printSettings();
int getButton(int button);
float getBattVoltage();
int checkBattLimits();
int checkAllAutotuneLimits();
int checkAutoTuneALimits();
int checkAutoTuneBLimits();
int checkAutoTuneCLimits();
int checkAutoTuneDLimits();
int autotuneA(float target, int delta);
int autotuneB(float target, int delta);
int autotuneC(float target, int delta);
int autotuneD(float target, int delta);
void setupPCINT(int data);
void setupEXTINT(int data);
void setupCronoTimer(boolean i);
void setupTimer2(int data);

#include "E:\Dropbox\Projects\arduino-1.0.5\hardware\arduino\variants\standard\pins_arduino.h" 
#include "E:\Dropbox\Projects\arduino-1.0.5\hardware\arduino\cores\arduino\arduino.h"
#include <..\CronographREVA\CronographREVA.ino>
