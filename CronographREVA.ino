/*
==========================
========Cronograph========
==========================

James McKenna
Palmerston North Airsoft Club [PAC]


===Software Version: V0.5===

V0.1 - 30/09/2015 - Initial Program
V0.2 - 28/04/2016 - Test Version, Autotune and prototype limit test
V0.3 - 03/05/2016 - Prog flow rewritten, Normal mode, Average mode, select mode, edit setup; Adjustment variables saved and loaded from EEPROM
V0.4 - 06/05/2016 - Added in error handling + batt monitoring, Autotune check limits moved to allow automatic calibration, added manual autotune
V0.5 - 19/07/2016 - Added in Feild crono function with adjustable limits 

===Compatible PCB Revision: REV A ===
-


===NOTES:===

---TODO---
-Add RPS function
-Add energy average function with adjustable pass/fail limit (for marshals) ---DONE---(V0.5)


===Error Codes===
0 - NO Error
1 - Battery Low
10 - I2C Init fail
11 - Battery Empty


*/


//=======================================================================================
//===============================Includes and Class Definations==========================
//=======================================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "mcp4728.h"
#include <I2C.h>
#include <MCP4651.h>
#include "EEPROM/EEPROM.h"
#include <math.h>



mcp4728 dac = mcp4728(0);
LiquidCrystal_I2C	lcd(0x27,2,1,0,4,5,6,7);
MCP4651 BEAM1POT(7);
MCP4651 BEAM2POT(3);

//=======================================================================================
//===============================Definations and Tuning Variables========================
//=======================================================================================

#define I2CTIMEOUT 80
#define cronoDistance 0.101
#define MPStoFPSconversionFactor 3.28084
#define timer2Limit 245

#define autoTuneDeltaDefault 200
#define BeamGlobalSetDefault 3500

#define DacAChannel 0
#define DacBChannel 1
#define DacCChannel 3
#define DacDChannel 2

#define modeNormal 0
#define modeAverage 1
#define modeFeild 2
#define modeRPS 3
#define modeSetup 5
#define modeAutotune 6
#define modeSelect 7

#define buttPressed LOW
#define buttonReleased HIGH
#define buttHoldTimeDefault 20 //Time to hold butt in 0.1 sec increments

#define BBWeightDefault 0.2
#define BBWeightMax 0.80
#define BBWeightMin 0.12

#define errorCode_NoError 0
#define errorCode_LowBatt 1
#define errorCodeHardError 10
#define errorCode_I2CINITFail 10
#define errorCode_EmptyBatt 11

int batteryAlertArray[4] = {7000, 6800, 10500, 10200};//7.4 low, 7.4 empty, 11.1 low, 11.1 empty // 3.5V per cell Low -/- 3.4Vper cell Empty
//7000,6800,10500,10200
//====== Pin definitions ======
#define SIGA A0
#define SIGB A1
#define SIGC A7
#define SIGD A6

#define BattV A3

#define OUTA 3 //INT1
#define OUTB 2 //INT0
#define OUTC 4 //PCINT20
#define OUTD 5 //PCINT21

#define BUTT1 8
#define BUTT2 9
#define BUTT3 10


//=======================================================================================
//=======================================LCD CUSTOM CHAR=================================
//=======================================================================================

//byte BattChar1[8] = {
	//0b11111,
	//0b10000,
	//0b10111,
	//0b10111,
	//0b10111,
	//0b10000,
	//0b11111
//};
//
//byte BattChar2[8] = {
	//0b11111,
	//0b00000,
	//0b11111,
	//0b11111,
	//0b11111,
	//0b00000,
	//0b11111
//};
//
//byte BattChar3[8] = {
	//0b11110,
	//0b00010,
	//0b11011,
	//0b11011,
	//0b11011,
	//0b00010,
	//0b11110
//};
//
//byte BattChar4[8] = {
	//0b11111,
	//0b10000,
	//0b10000,
	//0b10000,
	//0b10000,
	//0b10000,
	//0b11111
//};
//
//byte BattChar5[8] = {
	//0b11111,
	//0b00000,
	//0b11000,
	//0b11000,
	//0b11000,
	//0b00000,
	//0b11111
//};
//
//byte BattChar6[8] = {
	//0b11111,
	//0b00000,
	//0b00000,
	//0b00000,
	//0b00000,
	//0b00000,
	//0b11111
//};
//
//byte BattChar7[8] = {
	//0b11110,
	//0b00010,
	//0b00011,
	//0b00011,
	//0b00011,
	//0b00010,
	//0b11110
//};

int LCDBattCharArray[5][3] = {{0,1,2},{0,1,6},{0,4,6},{0,5,6},{3,5,6}};


//=======================================================================================
//=====================================Global Variables==================================
//=======================================================================================

int programMode = 0;
int prevProgramMode = 0;

int buttonHoldTime = buttHoldTimeDefault;
float BBWeight = BBWeightDefault;
int lowVoltage = batteryAlertArray[0];
int emptyVoltage = batteryAlertArray[1];
int BeamDelta = autoTuneDeltaDefault;
int BeamASet = BeamGlobalSetDefault;
int BeamBSet = BeamGlobalSetDefault;
int BeamCSet = BeamGlobalSetDefault;
int BeamDSet = BeamGlobalSetDefault;
boolean metric = false;
uint8_t AverageNumOfShots = 5;
float feildLimCQB = 1.14;
float feildLimFeild = 1.64;
float feildLimSniper = 3.34;

int error = false;
volatile boolean flag1 = false;
volatile boolean flag2 = false;

int POTAVALUE = 0;
int POTBVALUE = 0;
int POTCVALUE = 0;
int POTDVALUE = 0;

volatile boolean shotInProgress = false;
volatile boolean shotSuccess = false;
volatile boolean shotFlag = false;
volatile boolean testing1 = false;
volatile unsigned int timerCount = 0;
volatile unsigned int tempTimerCount = 0;
float mpsAverage = 0;
float mpsLastShot = 0;
float statStandardDev = 0;
float statRange = 0;

volatile boolean timer2Flag = false;
volatile boolean timer2Count = false;

boolean Buttlock = false;
boolean Buttlock2 = false;
boolean Buttlock3 = false;

unsigned int shotTimeArray[10] = {0};
float shotFpsArray[10] = {0};
uint8_t shotCount = 0;
uint8_t shotCountSelect = 0;
boolean AverageSet = false;


//=======================================================================================
//======================================Setup Func=======================================
//=======================================================================================

void setup()
{
	pinMode(BUTT1, INPUT);
	pinMode(BUTT2, INPUT);
	pinMode(BUTT3, INPUT);
	
	pinMode(SIGA, INPUT);
	pinMode(SIGB, INPUT);
	pinMode(SIGC, INPUT);
	pinMode(SIGD, INPUT);
	
	Serial.begin(57600);//TESTING
	
	lcd.begin (16,2);
	//lcd.createChar(0, BattChar1);
	//lcd.createChar(1, BattChar2);
	//lcd.createChar(2, BattChar3);
	//lcd.createChar(3, BattChar4);
	//lcd.createChar(4, BattChar5);
	//lcd.createChar(5, BattChar6);
	//lcd.createChar(6, BattChar7);
	lcd.begin (16,2);
	lcd.setBacklightPin(3,POSITIVE);
	lcd.setBacklight(HIGH);
	lcd.home();
	lcd.print("CRONOGRAPH");
	lcd.setCursor(0, 1);
	lcd.print("J McKenna [PAC]");
	delay(3000);
	lcd.clear();
	
	dac.begin();  // initialize i2c interface
	dac.vdd(5000);//Set external Ref V level in mV
	
	dac.setVref(1,1,1,1);//Internal Ref
	dac.setGain(1, 1, 1, 1);// x2 gain
	dac.voutWrite(1000, 1000, 1000, 1000);//set Output in mV
	
	//Init I2c interface for I2C Pots
	BEAM1POT.begin(I2CTIMEOUT);
	BEAM2POT.begin(I2CTIMEOUT);
	
	int temp = 0;
	temp = BEAM1POT.setTCON(0xEE);
	if (temp != 0)
	{
		error = errorCode_I2CINITFail;
	}
	
	temp = BEAM2POT.setTCON(0xEE);
	if (temp != 0)
	{
		error = errorCode_I2CINITFail;
	}

	//Check to see of I2C is functional
	if (error >= errorCodeHardError)
	{
		hardError(error);
	}

	//Set all Pots to be midrange (Approx 5K ohm)
	BEAM1POT.setWiper(Wiper0, 0x80);
	BEAM1POT.setWiper(Wiper1, 0x80);
	BEAM2POT.setWiper(Wiper0, 0x80);
	BEAM2POT.setWiper(Wiper1, 0x80);
	
	int tempint = 0;
	
	//--------
	lcd.setCursor(0, 0);
	lcd.print("AUTOTUNE A");
	tempint = autotuneA(BeamASet, autoTuneDeltaDefault);
	lcd.setCursor(0, 1);
	lcd.print("REF:");
	lcd.print(dac.getVout(DacAChannel));
	lcd.setCursor(9, 1);
	lcd.print("DEL:");
	lcd.print(autoTuneDeltaDefault);
	delay(100);

	lcd.setCursor(0, 0);
	lcd.print("AUTOTUNE B");
	tempint = autotuneB(BeamBSet, autoTuneDeltaDefault);
	lcd.setCursor(0, 1);
	lcd.print("REF:");
	lcd.print(dac.getVout(DacBChannel));
	lcd.setCursor(9, 1);
	lcd.print("DEL:");
	lcd.print(autoTuneDeltaDefault);
	delay(100);
	
	lcd.setCursor(0, 0);
	lcd.print("AUTOTUNE C");
	tempint = autotuneC(BeamCSet, autoTuneDeltaDefault);
	lcd.setCursor(0, 1);
	lcd.print("REF:");
	lcd.print(dac.getVout(DacCChannel));
	lcd.setCursor(9, 1);
	lcd.print("DEL:");
	lcd.print(autoTuneDeltaDefault);
	delay(100);
	
	lcd.setCursor(0, 0);
	lcd.print("AUTOTUNE D");
	tempint = autotuneD(BeamDSet, autoTuneDeltaDefault);
	lcd.setCursor(0, 1);
	lcd.print("REF:");
	lcd.print(dac.getVout(DacDChannel));
	lcd.setCursor(9, 1);
	lcd.print("DEL:");
	lcd.print(autoTuneDeltaDefault);
	delay(100);
	//---------------------

	initalStartSetup();
}


//=======================================================================================
//============================================Main=======================================
//=======================================================================================

void loop()
{
	//
	if ((timer2Flag) && (!shotInProgress))
	{
		checkAllAutotuneLimits();
		
		int tempError = checkBattLimits();
		if ((error == errorCode_NoError) && (tempError != 0))
		{
			error = tempError;
			
			if (error >= errorCodeHardError)
			{
				hardError(error);
			}
			updateDisplay(programMode, true);
		}
		else if ((error == errorCode_LowBatt) && (tempError == 0))
		{
			error = tempError;
			updateDisplay(programMode, true);
		}
		
		
		timer2Flag = false;
	}
	
	//Serial.print(programMode);
	//delay(500);
	
	switch (programMode)
	{
		case modeNormal:
		programModeNormal();
		break;
		
		case modeAverage:
		programModeAverage();
		break;
		
		case modeFeild:
		programModeFeild();
		break;
		
		case modeRPS:
		programModeRPS();
		break;
		
		case modeAutotune:
		programModeAutotune();
		break;
		
		case modeSetup:
		programModeSetup();
		break;
		
		case modeSelect:
		programModeSelect();
		break;
	}
	
	
	
}


//=======================================================================================
//=====================================Program Modes=====================================
//=======================================================================================

void programModeNormal()
{
	//PRESS- Clear the reading / HOLD- Change program mode
	if (getButton(BUTT1) == buttPressed)
	{
		int i = 0;
		mpsLastShot = 0;
		updateDisplay(programMode, true);
		
		while(getButton(BUTT1) == buttPressed)
		{
			i++;
			
			if (i >= buttonHoldTime)
			{
				//Butt Held
				prevProgramMode = programMode;
				programMode = modeSelect;
				break;
			}
			delay(100);
		}
	}
	
	//---Increment BB Weight---
	if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
	{
		int i = 0;
		Buttlock3 = true;
		
		BBWeight += 0.01;
		if (BBWeight >= BBWeightMax)
		{
			BBWeight = BBWeightMax;
		}
		lcd.setCursor(11, 1);
		lcd.print((float)BBWeight, 2);
		lcd.print("g");
		
		while(getButton(BUTT3) == buttPressed)
		{
			i++;
			
			if (i >= buttonHoldTime)
			{
				BBWeight = BBWeightMax;
				lcd.setCursor(11, 1);
				lcd.print((float)BBWeight, 2);
				lcd.print("g");
				break;
			}
			delay(100);
		}
	}
	else if (getButton(BUTT3) == buttonReleased)
	{
		Buttlock3 = false;
	}
	
	//---Decrement BB Weight---
	if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
	{
		int i = 0;
		Buttlock2 = true;
		
		BBWeight -= 0.01;
		if (BBWeight < BBWeightMin)
		{
			BBWeight = BBWeightMin;
		}
		lcd.setCursor(11, 1);
		lcd.print((float)BBWeight, 2);
		lcd.print("g");
		
		while(getButton(BUTT2) == buttPressed)
		{
			i++;
			//lock = true;

			if (i >= buttonHoldTime)
			{
				BBWeight = BBWeightMin;
				lcd.setCursor(11, 1);
				lcd.print((float)BBWeight, 2);
				lcd.print("g");
				break;
			}
			delay(100);
		}
	}
	else if (getButton(BUTT2) == buttonReleased)
	{
		Buttlock2 = false;
	}
	
	//Check for flag to see if shot has been fired and picked up
	if (shotFlag)
	{
		if (shotSuccess)
		{
			//Calc fps here
			mpsLastShot = cronoDistance / ((float)timerCount * 0.0000005);
			
			//do average stuff ===BROKEN AS FUCK ATM===
			//mpsAverage = (mpsAverage + mpsLastShot) / 2.0;
		}
		
		if (shotSuccess)//debugging
		{
			updateDisplay(programMode, shotSuccess);
		}
		shotFlag = false;
	}
}

void programModeAverage()
{
	//---Increment BBWeight / Shot Num---
	if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
	{
		int i = 0;
		Buttlock3 = true;

		if (AverageSet)
		{
			shotCountSelect++;
			if (shotCountSelect >= (AverageNumOfShots))
			{
				shotCountSelect = AverageNumOfShots;
			}
			updateDisplay(programMode, false);
		}
		else
		{
			BBWeight += 0.01;
			if (BBWeight >= BBWeightMax)
			{
				BBWeight = BBWeightMax;
			}
			lcd.setCursor(11, 1);
			lcd.print((float)BBWeight, 2);
			lcd.print("g");
		}
		
		while(getButton(BUTT3) == buttPressed)
		{
			i++;

			if (i >= buttonHoldTime)
			{
				break;
			}
			delay(100);
		}
	}
	else if (getButton(BUTT3) == buttonReleased)
	{
		Buttlock3 = false;
	}
	
	//---Decrement BB Weight / Shot Num---
	if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
	{
		int i = 0;
		Buttlock2 = true;
		
		if (AverageSet)
		{
			if (shotCountSelect != 0)
			{
				shotCountSelect--;
			}
			updateDisplay(programMode, false);
		}
		else
		{
			BBWeight -= 0.01;
			if (BBWeight < BBWeightMin)
			{
				BBWeight = BBWeightMin;
			}
			lcd.setCursor(11, 1);
			lcd.print((float)BBWeight, 2);
			lcd.print("g");
		}
		
		while(getButton(BUTT2) == buttPressed)
		{
			i++;

			if (i >= buttonHoldTime)
			{
				break;
			}
			delay(100);
		}
	}
	else if (getButton(BUTT2) == buttonReleased)
	{
		Buttlock2 = false;
	}
	//
	if (getButton(BUTT1) == buttPressed)
	{
		int i = 0;
		
		resetAverage();
		updateDisplay(programMode, true);
		
		while(getButton(BUTT1) == buttPressed)
		{
			i++;
			
			if (i >= buttonHoldTime)
			{
				//Butt Held
				prevProgramMode = programMode;
				programMode = modeSelect;
				break;
			}
			delay(100);
		}
	}
	
	//Check for flag to see if shot has been fired and picked up
	if ((shotFlag) && (!AverageSet))
	{
		if (shotSuccess)
		{
			//Calc fps here
			mpsLastShot = cronoDistance / ((float)timerCount * 0.0000005);
			
			//
			shotTimeArray[shotCount] = timerCount;
			shotCount++;
		}
		
		if (shotSuccess)//debugging
		{
			if (shotCount >= AverageNumOfShots)
			{
				calculateAverage();
				shotCountSelect = shotCount; // - 1;
				updateDisplay(programMode, false);
				shotCount = 0;
				AverageSet = true;
			}
			else
			{
				updateDisplay(programMode, shotSuccess);
			}
		}
		shotFlag = false;
	}
	else if((shotFlag) && (AverageSet))
	{
		shotFlag = false;
	}
	
}

//
void programModeFeild()
{
	//---Increment BBWeight / Shot Num---
	if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
	{
		int i = 0;
		Buttlock3 = true;

		if (AverageSet)
		{
			shotCountSelect++;
			if (shotCountSelect >= (AverageNumOfShots - 1))
			{
				shotCountSelect = AverageNumOfShots - 1;
			}
			updateDisplay(programMode, false);
		}
		else
		{
			BBWeight += 0.01;
			if (BBWeight >= BBWeightMax)
			{
				BBWeight = BBWeightMax;
			}
			lcd.setCursor(11, 1);
			lcd.print((float)BBWeight, 2);
			lcd.print("g");
		}
		
		while(getButton(BUTT3) == buttPressed)
		{
			i++;

			if (i >= buttonHoldTime)
			{
				break;
			}
			delay(100);
		}
	}
	else if (getButton(BUTT3) == buttonReleased)
	{
		Buttlock3 = false;
	}
	
	//---Decrement BB Weight / Shot Num---
	if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
	{
		int i = 0;
		Buttlock2 = true;
		
		if (AverageSet)
		{
			if (shotCountSelect != 0)
			{
				shotCountSelect--;
			}
			updateDisplay(programMode, false);
		}
		else
		{
			BBWeight -= 0.01;
			if (BBWeight < BBWeightMin)
			{
				BBWeight = BBWeightMin;
			}
			lcd.setCursor(11, 1);
			lcd.print((float)BBWeight, 2);
			lcd.print("g");
		}
		
		while(getButton(BUTT2) == buttPressed)
		{
			i++;

			if (i >= buttonHoldTime)
			{
				break;
			}
			delay(100);
		}
	}
	else if (getButton(BUTT2) == buttonReleased)
	{
		Buttlock2 = false;
	}
	//
	if (getButton(BUTT1) == buttPressed)
	{
		int i = 0;
		
		resetAverage();
		updateDisplay(programMode, true);
		
		while(getButton(BUTT1) == buttPressed)
		{
			i++;
			
			if (i >= buttonHoldTime)
			{
				//Butt Held
				prevProgramMode = programMode;
				programMode = modeSelect;
				break;
			}
			delay(100);
		}
	}
	
	//Check for flag to see if shot has been fired and picked up
	if ((shotFlag) && (!AverageSet))
	{
		if (shotSuccess)
		{
			//Calc fps here
			mpsLastShot = cronoDistance / ((float)timerCount * 0.0000005);
			
			//
			shotTimeArray[shotCount] = timerCount;
			shotCount++;
		}
		
		if (shotSuccess)//debugging
		{
			if (shotCount >= AverageNumOfShots)
			{
				calculateAverage();
				shotCountSelect = shotCount - 1;
				updateDisplay(programMode, false);
				shotCount = 0;
				AverageSet = true;
			}
			else
			{
				updateDisplay(programMode, shotSuccess);
			}
		}
		shotFlag = false;
	}
	else if((shotFlag) && (AverageSet))
	{
		shotFlag = false;
	}
}


void programModeRPS()
{
	//
	
	
	
	if (getButton(BUTT1) == buttPressed)
	{
		int i = 0;
		
		while(getButton(BUTT1) == buttPressed)
		{
			i++;
			
			if (i >= buttonHoldTime)
			{
				prevProgramMode = programMode;
				programMode = modeSelect;
				break;
			}
			delay(100);
		}
	}
}

void programModeAutotune()
{
	updateDisplay(programMode, true);
	delay(1000);
	checkAllAutotuneLimits();
	delay(1000);
	programMode = prevProgramMode;
	updateDisplay(programMode, true);
}

#pragma region Program_Setup_Mode
void programModeSetup()
{
	int buttonHoldTimeTemp = buttonHoldTime;
	int tempMode = 0;
	Buttlock = false;
	Buttlock2 = false;
	Buttlock3 = false;
	boolean lock = true;
	int tempint = 0;
	
	lcd.clear();
	
	while(getButton(BUTT1) == buttPressed){}
	
	while(tempMode != 100)
	{
		switch(tempMode)
		{
			//============BUTT HOLD TIME================
			case 0:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Butt Hold Time");
				lcd.setCursor(0, 1);
				lcd.print((float)buttonHoldTimeTemp/10, 1);
				lcd.print("Sec");
				lock = false;
			}
			
			//
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				buttonHoldTimeTemp++;
				if (buttonHoldTimeTemp > 100)
				{
					buttonHoldTimeTemp = 100;
				}
				lcd.setCursor(0, 1);
				lcd.print((float)buttonHoldTimeTemp/10, 1);
				lcd.print("Sec");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					lock = true;

					if (i >= buttonHoldTime)
					{
						buttonHoldTimeTemp = 100;
						lcd.setCursor(0, 1);
						lcd.print((float)buttonHoldTimeTemp/10, 1);
						lcd.print("Sec");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				buttonHoldTimeTemp--;
				if (buttonHoldTimeTemp < 1)
				{
					buttonHoldTimeTemp = 1;
				}
				lcd.setCursor(0, 1);
				lcd.print((float)buttonHoldTimeTemp/10, 1);
				lcd.print("Sec");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					lock = true;

					if (i >= buttonHoldTime)
					{
						buttonHoldTimeTemp = 1;
						lcd.setCursor(0, 1);
						lcd.print((float)buttonHoldTimeTemp/10, 1);
						lcd.print("Sec");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					buttonHoldTime = buttonHoldTimeTemp;
					lock = true;
					tempMode = 1;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//===========Metric/Imperial Units============
			case 1:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Units");
				lcd.setCursor(0, 1);
				if (metric)
				lcd.print("Metric [MPS]");
				else
				lcd.print("Imperial [FPS]");
				
				lock = false;
			}
			
			//---Metric---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				metric = true;
				
				lcd.setCursor(0, 1);
				lcd.print("Metric [MPS]");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				metric = false;
				
				lcd.setCursor(0, 1);
				lcd.print("Imperial [FPS]");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 2;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//==========BB WEIGHT============
			case 2:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("BB Weight");
				lcd.setCursor(0, 1);
				lcd.print(BBWeight, 2);
				lcd.print("g               ");
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				BBWeight += 0.01;
				if (BBWeight >= BBWeightMax)
				{
					BBWeight = BBWeightMax;
				}
				lcd.setCursor(0, 1);
				lcd.print((float)BBWeight, 2);
				lcd.print("g");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						BBWeight = BBWeightMax;
						lcd.setCursor(0, 1);
						lcd.print((float)BBWeight, 2);
						lcd.print("g");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				BBWeight -= 0.01;
				if (BBWeight < BBWeightMin)
				{
					BBWeight = BBWeightMin;
				}
				lcd.setCursor(0, 1);
				lcd.print((float)BBWeight, 2);
				lcd.print("g");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						BBWeight = BBWeightMin;
						lcd.setCursor(0, 1);
						lcd.print((float)BBWeight, 2);
						lcd.print("g");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 3;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//============Num Of AVG============
			case 3:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("AVG Num Shots:  ");
				lcd.setCursor(0, 1);
				lcd.print("Shots:");
				lcd.print(AverageNumOfShots);
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				AverageNumOfShots ++;
				if (AverageNumOfShots >= 20)
				{
					AverageNumOfShots = 20;
				}
				lcd.setCursor(6, 1);
				lcd.print(AverageNumOfShots);
				lcd.print("    ");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						AverageNumOfShots = 20;
						lcd.setCursor(6, 1);
						lcd.print(AverageNumOfShots);
						lcd.print("    ");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				AverageNumOfShots --;
				if (AverageNumOfShots < 2)
				{
					AverageNumOfShots = 2;
				}
				lcd.setCursor(6, 1);
				lcd.print(AverageNumOfShots);
				lcd.print("    ");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						AverageNumOfShots = 2;
						lcd.setCursor(6, 1);
						lcd.print(AverageNumOfShots);
						lcd.print("    ");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 4;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//==========FEILD LIMIT CQB============
			case 4:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Field Lim:CQB");
				lcd.setCursor(0, 1);
				lcd.print("E:");
				lcd.print(feildLimCQB, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimCQB)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				lcd.print("@.2g");
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				feildLimCQB += 0.01;
				//if (feildLimCQB >= BBWeightMax)
				//{
					//feildLimCQB = BBWeightMax;
				//}
				lcd.setCursor(2, 1);
				lcd.print((float)feildLimCQB, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimCQB)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						//BBWeight = BBWeightMax;
						//lcd.setCursor(0, 1);
						//lcd.print((float)BBWeight, 2);
						//lcd.print("g");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				feildLimCQB -= 0.01;
				if (feildLimCQB <= 0)
				{
					feildLimCQB = 0.1;
				}
				lcd.setCursor(2, 1);
				lcd.print((float)feildLimCQB, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimCQB)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						//BBWeight = BBWeightMin;
						//lcd.setCursor(0, 1);
						//lcd.print((float)BBWeight, 2);
						//lcd.print("g");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 5;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//==========FEILD LIMIT FEILD============
			case 5:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Field Lim:FIELD");
				lcd.setCursor(0, 1);
				lcd.print("E:");
				lcd.print(feildLimFeild, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimFeild)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				lcd.print("@.2g");
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				feildLimFeild += 0.01;
				//if (feildLimCQB >= BBWeightMax)
				//{
					//feildLimCQB = BBWeightMax;
				//}
				lcd.setCursor(2, 1);
				lcd.print((float)feildLimFeild, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimFeild)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					lock = true;

					if (i >= buttonHoldTime)
					{
						//BBWeight = BBWeightMax;
						//lcd.setCursor(0, 1);
						//lcd.print((float)BBWeight, 2);
						//lcd.print("g");
						//break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				feildLimFeild -= 0.01;
				if (feildLimFeild <= (feildLimCQB + 0.1))
				{
					feildLimFeild = 0.1;
				}
				lcd.setCursor(2, 1);
				lcd.print((float)feildLimFeild, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimFeild)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						//BBWeight = BBWeightMin;
						//lcd.setCursor(0, 1);
						//lcd.print((float)BBWeight, 2);
						//lcd.print("g");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 6;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//==========FEILD LIMIT SNIPER============
			case 6:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Field Lim:SNIPER");
				lcd.setCursor(0, 1);
				lcd.print("E:");
				lcd.print(feildLimSniper, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimSniper)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				lcd.print("@.2g");
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				feildLimSniper += 0.01;
				//if (feildLimCQB >= BBWeightMax)
				//{
					//feildLimCQB = BBWeightMax;
				//}
				lcd.setCursor(2, 1);
				lcd.print((float)feildLimSniper, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimSniper)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						//BBWeight = BBWeightMax;
						//lcd.setCursor(0, 1);
						//lcd.print((float)BBWeight, 2);
						//lcd.print("g");
						//break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				feildLimSniper -= 0.01;
				if (feildLimSniper <= (feildLimFeild + 0.1))
				{
					feildLimSniper = 0.1;
				}
				lcd.setCursor(2, 1);
				lcd.print((float)feildLimSniper, 2);
				lcd.print("J");
				lcd.setCursor(9, 1);
				float tempFloat = sqrt((2 * feildLimSniper)/ (0.20 / 1000));
				if (metric)
				{
					lcd.print(tempFloat, 0);
				}
				else
				{
					lcd.print(convertToFPS(tempFloat), 0);
				}
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						//BBWeight = BBWeightMin;
						//lcd.setCursor(0, 1);
						//lcd.print((float)BBWeight, 2);
						//lcd.print("g");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 7;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//=====LIPO V SET===== //used for low V alert
			case 7:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Lipo V Set");
				lcd.setCursor(0, 1);
				if (lowVoltage == batteryAlertArray[0])
				lcd.print("7.4V Lipo       ");
				else
				lcd.print("11.1V Lipo      ");
				
				lock = false;
			}
			
			//---7.4V---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				lowVoltage = batteryAlertArray[0];
				emptyVoltage = batteryAlertArray[1];
				
				lcd.setCursor(0, 1);
				lcd.print("7.4V Lipo       ");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---11.1V---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				lowVoltage = batteryAlertArray[2];
				emptyVoltage = batteryAlertArray[3];
				
				lcd.setCursor(0, 1);
				lcd.print("11.1V Lipo      ");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 8;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//==========BEAM DELTA SET===========-------------------------------------
			case 8:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Beam Delta");
				lcd.setCursor(0, 1);
				lcd.print(BeamDelta);
				lcd.print("mV          ");
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				BeamDelta += 10;
				if (BeamDelta >= 1000)
				{
					BeamDelta = 1000;
				}
				lcd.setCursor(0, 1);
				lcd.print(BeamDelta);
				lcd.print("mV");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						BeamDelta = 1000;
						lcd.setCursor(0, 1);
						lcd.print(BeamDelta);
						lcd.print("mV");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				BeamDelta -= 10;
				BeamBSet = BeamASet;
				BeamCSet = BeamASet;
				BeamDSet = BeamASet;
				if (BeamDelta < 200)
				{
					BeamDelta = 200;
				}
				lcd.setCursor(0, 1);
				lcd.print(BeamDelta);
				lcd.print("mV");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						BeamDelta = 200;
						lcd.setCursor(0, 1);
						lcd.print(BeamDelta);
						lcd.print("mV");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 9;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//============BEAM TARGET SET=============
			case 9:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Beam set point");
				lcd.setCursor(0, 1);
				lcd.print(BeamASet);
				lcd.print("mV       ");
				lock = false;
			}
			
			//---Increment---
			if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
			{
				int i = 0;
				Buttlock3 = true;
				
				BeamASet += 10;
				BeamBSet = BeamASet;
				BeamCSet = BeamASet;
				BeamDSet = BeamASet;
				if (BeamASet >= 4200)
				{
					BeamASet = 4200;
				}
				lcd.setCursor(0, 1);
				lcd.print(BeamASet);
				lcd.print("mV");
				
				while(getButton(BUTT3) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						BeamASet = 4200;
						lcd.setCursor(0, 1);
						lcd.print(BeamASet);
						lcd.print("mV");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT3) == buttonReleased)
			{
				Buttlock3 = false;
			}
			//---Decrement---
			if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
			{
				int i = 0;
				Buttlock2 = true;
				
				BeamASet -= 10;
				BeamBSet = BeamASet;
				BeamCSet = BeamASet;
				BeamDSet = BeamASet;
				if (BeamASet < 500)
				{
					BeamASet = 500;
				}
				lcd.setCursor(0, 1);
				lcd.print(BeamASet);
				lcd.print("mV");
				
				while(getButton(BUTT2) == buttPressed)
				{
					i++;
					//lock = true;

					if (i >= buttonHoldTime)
					{
						BeamASet = 500;
						lcd.setCursor(0, 1);
						lcd.print(BeamASet);
						lcd.print("mV");
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT2) == buttonReleased)
			{
				Buttlock2 = false;
			}
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 10;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			//==============Error Code==============//This mode is usefull for testing / debugging
			case 10:
			if (lock == true)
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
				lcd.setCursor(0, 0);
				lcd.print("Error Code:");
				lcd.print(error);
				lcd.setCursor(0, 1);
				lcd.print("                ");
				lock = false;
			}
			
			
			if (timer2Flag == true)
			{
				lcd.setCursor(11, 0);
				lcd.print(error);
				timer2Flag = false;
			}
			
			//---Confirm/Switch mode---
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					lock = true;
					tempMode = 0;
					if (i >= buttonHoldTime)
					{
						programMode = prevProgramMode;
						tempMode = 100;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
		}
	}
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Saving Settings");
	
	saveSettingsToEEPROM();
	
	lcd.clear();
	updateDisplay(programMode, true);
	
}
#pragma endregion


#pragma region Select_program_mode
void programModeSelect()
{
	int tempMode = 0;
	boolean Buttlock = false;
	boolean lock = true;
	
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Sel Mode:");
	lcd.setCursor(0, 1);
	lcd.print("Batt V:");
	
	while(getButton(BUTT1) == buttPressed){}
	
	while(tempMode != 10)
	{
		//---Increment Mode---
		if ((getButton(BUTT3) == buttPressed) && (Buttlock3 == false))
		{
			int i = 0;
			Buttlock3 = true;

			tempMode++;
			if (tempMode > 5)
			{
				tempMode = 0;
			}
			lock = true;

			while(getButton(BUTT3) == buttPressed)
			{
				i++;

				if (i >= buttonHoldTime)
				{
					break;
				}
				delay(100);
			}
		}
		else if (getButton(BUTT3) == buttonReleased)
		{
			Buttlock3 = false;
		}
		
		//---Decrement Shot Num---
		if ((getButton(BUTT2) == buttPressed) && (Buttlock2 == false))
		{
			int i = 0;
			Buttlock2 = true;
			
			tempMode--;
			if (tempMode < 0)
			{
				tempMode = 5;
			}
			lock = true;

			while(getButton(BUTT2) == buttPressed)
			{
				i++;

				if (i >= buttonHoldTime)
				{
					break;
				}
				delay(100);
			}
		}
		else if (getButton(BUTT2) == buttonReleased)
		{
			Buttlock2 = false;
		}
		//
		
		switch(tempMode)
		{
			case 0:
			if (lock == true)
			{
				lcd.setCursor(9, 0);
				lcd.print("       ");
				lcd.setCursor(9, 0);
				lcd.print("Normal");
				lock = false;
			}
			
			if (timer2Flag)
			{
				lcd.setCursor(7, 1);
				lcd.print("       ");
				lcd.setCursor(7, 1);
				lcd.print(getBattVoltage(), 2);
				lcd.print("V");
				timer2Flag = false;
			}
			
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					//lock = true;
					//tempMode = 1;
					if (i >= buttonHoldTime)
					{
						programMode = modeNormal;
						tempMode = 10;
						saveProgramModeToEEPROM();
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			case 1:
			if (lock == true)
			{
				lcd.setCursor(9, 0);
				lcd.print("       ");
				lcd.setCursor(9, 0);
				lcd.print("Average");
				lock = false;
			}
			
			if (timer2Flag)
			{
				lcd.setCursor(7, 1);
				lcd.print("       ");
				lcd.setCursor(7, 1);
				lcd.print(getBattVoltage(), 2);
				lcd.print("V");
				timer2Flag = false;
			}
			
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					//tempMode = 2;
					//lock = true;
					if (i >= buttonHoldTime)
					{
						programMode = modeAverage;
						tempMode = 10;
						saveProgramModeToEEPROM();
						resetAverage();
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			case 2:
			if (lock == true)
			{
				lcd.setCursor(9, 0);
				lcd.print("       ");
				lcd.setCursor(9, 0);
				lcd.print("Field");
				lock = false;
			}
			
			if (timer2Flag)
			{
				lcd.setCursor(7, 1);
				lcd.print("       ");
				lcd.setCursor(7, 1);
				lcd.print(getBattVoltage(), 2);
				lcd.print("V");
				timer2Flag = false;
			}
			
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					//tempMode = 2;
					//lock = true;
					if (i >= buttonHoldTime)
					{
						programMode = modeFeild;
						tempMode = 10;
						saveProgramModeToEEPROM();
						resetAverage();
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			case 3:
			if (lock == true)
			{
				lcd.setCursor(9, 0);
				lcd.print("       ");
				lcd.setCursor(9, 0);
				lcd.print("RPS");
				lock = false;
			}
			
			if (timer2Flag)
			{
				lcd.setCursor(7, 1);
				lcd.print("       ");
				lcd.setCursor(7, 1);
				lcd.print(getBattVoltage(), 2);
				lcd.print("V");
				timer2Flag = false;
			}
			
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					//tempMode = 3;
					//lock = true;
					if (i >= buttonHoldTime)
					{
						programMode = modeRPS;
						tempMode = 10;
						saveProgramModeToEEPROM();
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			case 4:
			if (lock == true)
			{
				lcd.setCursor(9, 0);
				lcd.print("       ");
				lcd.setCursor(9, 0);
				lcd.print("Setup");
				lock = false;
			}
			
			if (timer2Flag)
			{
				lcd.setCursor(7, 1);
				lcd.print("       ");
				lcd.setCursor(7, 1);
				lcd.print(getBattVoltage(), 2);
				lcd.print("V");
				timer2Flag = false;
			}
			
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					//tempMode = 4;
					//lock = true;
					if (i >= buttonHoldTime)
					{
						programMode = modeSetup;
						tempMode = 10;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
			
			case 5:
			if (lock == true)
			{
				lcd.setCursor(9, 0);
				lcd.print("       ");
				lcd.setCursor(9, 0);
				lcd.print("ATune");
				lock = false;
			}
			
			if (timer2Flag)
			{
				lcd.setCursor(7, 1);
				lcd.print("       ");
				lcd.setCursor(7, 1);
				lcd.print(getBattVoltage(), 2);
				lcd.print("V");
				timer2Flag = false;
			}
			
			if ((getButton(BUTT1) == buttPressed) && (Buttlock == false))
			{
				int i = 0;
				Buttlock = true;
				while(getButton(BUTT1) == buttPressed)
				{
					i++;
					//tempMode = 0;
					//lock = true;
					if (i >= buttonHoldTime)
					{
						programMode = modeAutotune;
						tempMode = 10;
						break;
					}
					delay(100);
				}
			}
			else if (getButton(BUTT1) == buttonReleased)
			{
				Buttlock = false;
			}
			break;
		}
	}
	
	lcd.clear();
	updateDisplay(programMode, true);
}
#pragma endregion

//=======================================================================================
//==================================Background Prog Func=================================
//=======================================================================================

//
void initalStartSetup()
{
	loadSettingsFromEEPROM();

	setupPCINT(1);
	setupEXTINT(1);
	setupTimer2(1);
	
	if (programMode == modeRPS)
	{
		setupCronoTimer(2);
	}
	else
	{
		setupCronoTimer(1);
	}

	
	lcd.clear();
	updateDisplay(programMode, true);
	
	resetAverage();
	shotInProgress = false;
}


//
void hardError(int errorCode)
{
	lcd.clear();
	lcd.print("Fatal Error");
	lcd.setCursor(0, 1);
	if (errorCode == errorCode_EmptyBatt)
	{
		lcd.print("BATT EMPTY");
	}
	else
	{
		lcd.print("CODE:");
		lcd.print(errorCode);
	}
	
	setupCronoTimer(false);
	setupTimer2(0);
	setupPCINT(0);
	setupEXTINT(0);
	
	while(1){};
}


//
void resetAverage()
{
	shotCount = 0;
	AverageSet = false;
	mpsLastShot = 0;
}


//
float convertToFPS(float data)
{
	float tempFloat = data * MPStoFPSconversionFactor;//3.28084
	
	return tempFloat;
}

//
void calculateAverage()
{
	unsigned long tempint = 0;
	unsigned long tempintsq = 0;
	float tempMean = 0;
	float tempMeanSq = 0;
	
	for (int i = 0 ; i < AverageNumOfShots ; i++)
	{
		tempint += shotTimeArray[i];
		shotFpsArray[i] = cronoDistance / ((float)tempint * 0.0000005);
		tempMean = shotFpsArray[i];
		tempMeanSq = (shotFpsArray[i] * shotFpsArray[i]);
	}
	
	tempMean = tempMean / (float)AverageNumOfShots;
	statStandardDev = (tempMeanSq - ((tempMean * tempMean) / AverageNumOfShots)) / ((float)AverageNumOfShots - 1.0);
	
	tempint = (float)tempint / (float)AverageNumOfShots;
	
	mpsAverage = cronoDistance / ((float)tempint * 0.0000005);
}


//=======================================================================================
//====================================Display Functions==================================
//=======================================================================================

void updateDisplay(int mode, boolean data)
{
	lcd.clear();
	if ((error == errorCode_LowBatt) && (mode != modeAutotune))
	{
		lcd.setCursor(13, 0);
		lcd.write(LCDBattCharArray[3][0]);
		lcd.write(LCDBattCharArray[3][1]);
		lcd.write(LCDBattCharArray[3][2]);
	}
	lcd.setCursor(0, 0);
	
	switch(mode)
	{
		case modeNormal:
		if (data)
		{
			if (metric)
			{
				lcd.print("MPS:");
				lcd.print(mpsLastShot, 1);
			}
			else
			{
				lcd.print("FPS:");
				lcd.print(convertToFPS(mpsLastShot), 1);
			}
			lcd.setCursor(0, 1);
			lcd.print("E:");
			float energy = 0.5 * ((float)BBWeight / 1000.0) * ((float)mpsLastShot * (float)mpsLastShot);
			lcd.print(energy, 2);
			lcd.setCursor(11, 1);
			lcd.print(BBWeight, 2);
			lcd.print("g");
		}
		else
		{
			lcd.print("FPS:");
			lcd.print("FAIL");
			lcd.setCursor(0, 1);
			lcd.print("AVG:");
			lcd.print(timerCount);
		}
		break;
		
		case modeAverage:
		if (data)
		{
			if (metric)
			{
				lcd.print("MPS:");
				lcd.print(mpsLastShot, 1);
			}
			else
			{
				lcd.print("FPS:");
				lcd.print(convertToFPS(mpsLastShot), 1);
			}
			
			if (error == errorCode_LowBatt)
			{
				lcd.setCursor(10, 0);
				lcd.print(shotCount);
			}
			else
			{
				lcd.setCursor(10, 0);
				lcd.print("SC:");
				lcd.print(shotCount);
			}
			
			lcd.setCursor(0, 1);
			lcd.print("E:");
			float energy = 0.5 * ((float)BBWeight / 1000.0) * ((float)mpsLastShot * (float)mpsLastShot);
			lcd.print(energy, 2);
			lcd.setCursor(11, 1);
			lcd.print(BBWeight, 2);
			lcd.print("g");
		}
		else
		{
			if (metric)
			{
				lcd.print("MPS:");
				lcd.print(mpsAverage, 1);
			}
			else
			{
				lcd.print("FPS:");
				lcd.print(convertToFPS(mpsAverage), 1);
			}
			lcd.setCursor(10, 0);
			lcd.print("E:");
			float energy = 0.5 * ((float)BBWeight / 1000.0) * ((float)mpsAverage * (float)mpsAverage);
			lcd.print(energy, 2);
			lcd.setCursor(0, 1);
			if (shotCountSelect == AverageNumOfShots)
			{
				lcd.print("SC:");
				lcd.print(shotCountSelect);
				lcd.setCursor(7, 1);
				lcd.print("SD:");
				lcd.print(statStandardDev, 2);
			}
			else
			{
				lcd.print("SC:");
				lcd.print(shotCountSelect + 1);
				lcd.setCursor(7, 1);
				float tempFloat = cronoDistance / ((float)shotTimeArray[shotCountSelect] * 0.0000005);
				if (metric)
				{
					lcd.print("MPS:");
					lcd.print(tempFloat, 1);
				}
				else
				{
					lcd.print("FPS:");
					lcd.print(convertToFPS(tempFloat), 1);
				}
			}
		}
		break;
		
		case modeRPS:
		//
		break;
		
		case modeFeild:
		if (data)
		{
			if (metric)
			{
				lcd.print("MPS:");
				lcd.print(mpsLastShot, 1);
			}
			else
			{
				lcd.print("FPS:");
				lcd.print(convertToFPS(mpsLastShot), 1);
			}
			
			if (error == errorCode_LowBatt)
			{
				lcd.setCursor(10, 0);
				lcd.print(shotCount);
			}
			else
			{
				lcd.setCursor(10, 0);
				lcd.print("SC:");
				lcd.print(shotCount);
			}
			
			lcd.setCursor(0, 1);
			lcd.print("E:");
			float energy = 0.5 * ((float)BBWeight / 1000.0) * ((float)mpsLastShot * (float)mpsLastShot);
			lcd.print(energy, 2);
			lcd.setCursor(11, 1);
			lcd.print(BBWeight, 2);
			lcd.print("g");
		}
		else
		{
			if (metric)
			{
				lcd.print("MPS:");
				lcd.print(mpsAverage, 1);
			}
			else
			{
				lcd.print("FPS:");
				lcd.print(convertToFPS(mpsAverage), 1);
			}
			lcd.setCursor(10, 0);
			lcd.print("E:");
			float energy = 0.5 * ((float)BBWeight / 1000.0) * ((float)mpsAverage * (float)mpsAverage);
			lcd.print(energy, 2);
			lcd.setCursor(0, 1);
			if (energy <= feildLimCQB)
			{
				lcd.print("------CQB-------");
			}
			else if ((energy > feildLimCQB) && (energy <= feildLimFeild))
			{
				lcd.print("-----Field------");
			}
			else if ((energy > feildLimFeild) && (energy <= feildLimSniper))
			{
				lcd.print("-----Sniper-----");
			}
			else
			{
				lcd.print("--OVER LIMITS--");
			}
		}
		break;
		
		case modeAutotune:
		lcd.print("Auto Tuning All");
		lcd.setCursor(0, 1);
		lcd.print("Channels");
		break;
	}
	
	
}


//=======================================================================================
//====================================EEPROM Functions===================================
//=======================================================================================

//int programMode = 0;
//
//int buttonHoldTime = buttHoldTimeDefault;
//float BBWeight = BBWeightDefault;
//int lowVoltage = 0;
//int emptyVoltage = 0;
//int BeamDelta = autoTuneDeltaDefault;
//int BeamASet = BeamGlobalSetDefault;
//int BeamBSet = BeamGlobalSetDefault;
//int BeamCSet = BeamGlobalSetDefault;
//int BeamDSet = BeamGlobalSetDefault;
//boolean metric = true;
//uint8_t AverageNumOfShots = 5;
//float feildLimCQB = 1.14;
//float feildLimFeild = 1.64;
//float feildLimSniper = 3.34;

//
void saveSettingsToEEPROM()
{
	EEPROM.writeWord(0, programMode);
	EEPROM.writeWord(2, buttonHoldTime);
	EEPROM.writeFloat(4, BBWeight);
	EEPROM.writeWord(8, lowVoltage);
	EEPROM.writeWord(10, emptyVoltage);
	EEPROM.writeWord(12, BeamDelta);
	EEPROM.writeWord(14, BeamASet);
	EEPROM.writeWord(16, BeamBSet);
	EEPROM.writeWord(18, BeamCSet);
	EEPROM.writeWord(20, BeamDSet);
	EEPROM.write(22, metric);
	EEPROM.writeByte(23, AverageNumOfShots);
	EEPROM.writeFloat(24, feildLimCQB);
	EEPROM.writeFloat(28, feildLimFeild);
	EEPROM.writeFloat(32, feildLimSniper);
}


//
void loadSettingsFromEEPROM()
{
	programMode = EEPROM.readWord(0);
	buttonHoldTime = EEPROM.readWord(2);
	BBWeight = EEPROM.readFloat(4);
	lowVoltage = EEPROM.readWord(8);
	emptyVoltage = EEPROM.readWord(10);
	BeamDelta = EEPROM.readWord(12);
	BeamASet = EEPROM.readWord(14);
	BeamBSet = EEPROM.readWord(16);
	BeamCSet = EEPROM.readWord(18);
	BeamDSet = EEPROM.readWord(20);
	metric = EEPROM.read(22);
	AverageNumOfShots = EEPROM.read(23);
	feildLimCQB = EEPROM.readFloat(24);
	feildLimFeild = EEPROM.readFloat(28);
	feildLimSniper = EEPROM.readFloat(32);
}

void saveBBWeightToEEPROM()
{
	EEPROM.writeFloat(4, BBWeight);
}

void saveProgramModeToEEPROM()
{
	EEPROM.writeWord(0, programMode);
}

//Debugging
void printSettings()
{
	Serial.println(programMode);
	Serial.println(buttonHoldTime);
	Serial.println(BBWeight);
	Serial.println(lowVoltage);
	Serial.println(emptyVoltage);
	Serial.println(BeamDelta);
	Serial.println(BeamASet);
	Serial.println(BeamBSet);
	Serial.println(BeamCSet);
	Serial.println(BeamDSet);
	Serial.println(metric);
	Serial.println(AverageNumOfShots);
	Serial.println(feildLimCQB);
	Serial.println(feildLimFeild);
	Serial.println(feildLimSniper);
	Serial.println("===");
}

//=======================================================================================
//===================================Button Functions====================================
//=======================================================================================

int getButton(int button)
{
	return digitalRead(button);
}


//=======================================================================================
//===================================Battery Functions===================================
//=======================================================================================

float getBattVoltage()
{
	float returnVal = 0.0;
	int tempint = 0;
	
	tempint = analogRead(BattV);
	returnVal = ((5.0 / 1024.0) * (float)tempint) * 3.0;
	
	return returnVal;
}

int checkBattLimits()
{
	int tempint = (float)(getBattVoltage() * 1000);
	
	if ((tempint <= lowVoltage) && (tempint > emptyVoltage))
	{
		return errorCode_LowBatt;
	}
	else if (tempint <= emptyVoltage)
	{
		return errorCode_EmptyBatt;
	}
	
	return 0;
}

//=======================================================================================
//========================================AutoTune=======================================
//=======================================================================================

//
int checkAllAutotuneLimits()
{
	int tempint = 0;
	
	tempint = checkAutoTuneALimits();
	tempint = checkAutoTuneBLimits();
	tempint = checkAutoTuneCLimits();
	tempint = checkAutoTuneDLimits();
	
	return tempint;
}


//
int checkAutoTuneALimits()
{
	int tempint = 0;
	
	//check setpoints, autotune if necessary
	float tempfloat = (5000.0 / 1024.0) * (float)analogRead(SIGA);
	if ((dac.getVout(DacAChannel) + BeamDelta - 5) > (int)tempfloat)
	{
		//Turn interrupts off
		setupEXTINT(false);
		setupPCINT(false);
		
		tempint = autotuneA(BeamASet, BeamDelta);
		
		//Turn interrupts back on
		setupEXTINT(true);
		setupPCINT(true);
	}
	
	return tempint;
}

//
int checkAutoTuneBLimits()
{
	int tempint = 0;
	
	//check setpoints, autotune if necessary
	float tempfloat = (5000.0 / 1024.0) * (float)analogRead(SIGB);
	if ((dac.getVout(DacBChannel) + BeamDelta - 5) > (int)tempfloat)
	{
		//Turn interrupts off
		setupEXTINT(false);
		setupPCINT(false);
		
		tempint = autotuneB(BeamBSet, BeamDelta);
		
		//Turn interrupts back on
		setupEXTINT(true);
		setupPCINT(true);
	}
	
	return tempint;
}


//
int checkAutoTuneCLimits()
{
	int tempint = 0;
	
	//check setpoints, autotune if necessary
	float tempfloat = (5000.0 / 1024.0) * (float)analogRead(SIGC);
	if ((dac.getVout(DacCChannel) + BeamDelta - 5) > (int)tempfloat)
	{
		//Turn interrupts off
		setupEXTINT(false);
		setupPCINT(false);
		
		tempint = autotuneC(BeamCSet, BeamDelta);
		
		//Turn interrupts back on
		setupEXTINT(true);
		setupPCINT(true);
	}
	
	return tempint;
}

//
int checkAutoTuneDLimits()
{
	int tempint = 0;
	
	//check setpoints, autotune if necessary
	float tempfloat = (5000.0 / 1024.0) * (float)analogRead(SIGD);
	if ((dac.getVout(DacDChannel) + BeamDelta - 5) > (int)tempfloat)
	{
		//Turn interrupts off
		setupEXTINT(false);
		setupPCINT(false);
		
		tempint = autotuneD(BeamDSet, BeamDelta);
		
		//Turn interrupts back on
		setupEXTINT(true);
		setupPCINT(true);
	}
	
	return tempint;
}

int autotuneA(float target, int delta)
{
	float tempFloat = 0.0;
	int tempV = 0;
	int result = 0;
	
	//Set gain
	for (int i = 255 ; i >= 0 ; i--)
	{
		POTAVALUE = i;
		BEAM1POT.setWiper(Wiper0, POTAVALUE);
		delay(1);//Allow loop to stabilize after a change
		tempV = analogRead(SIGA);
		tempFloat = (5000.0 / 1024.0) * (float)tempV;
		
		if (tempFloat >= target)//Sig has reached the maximum allowed level(going higher will adversely affect responsiveness and speed)
		{
			result = 1;
			break;
		}
	}
	
	if (POTAVALUE <= 1)
	{
		result = 2;
	}
	
	delay(10);
	
	//Set DAC Referance voltage to a voltage level of the measured sig - delta voltage
	tempV = analogRead(SIGA);
	tempFloat = (5.0 / 1024.0) * (float)tempV;
	tempV = (tempFloat * 1000) - delta;
	dac.voutWrite(0, tempV);
	
	return  result;
	
}

int autotuneB(float target, int delta)
{
	float tempFloat = 0.0;
	int tempV = 0;
	int result = 0;
	
	//Set gain
	for (int i = 255 ; i >= 0 ; i--)
	{
		POTBVALUE = i;
		BEAM1POT.setWiper(Wiper1, POTBVALUE);
		delay(1);
		tempV = analogRead(SIGB);
		tempFloat = (5000 / 1024.0) * (float)tempV;
		
		if (tempFloat >= target)
		{
			result = 1;
			break;
		}
	}
	
	if (POTBVALUE <= 1)
	{
		result = 2;
	}
	
	delay(10);

	tempV = analogRead(SIGB);
	tempFloat = (5.0 / 1024.0) * (float)tempV;
	tempV = (tempFloat * 1000) - delta;
	dac.voutWrite(1, tempV);
	
	return  result;
	
}

int autotuneC(float target, int delta)
{
	float tempFloat = 0.0;
	int tempV = 0;
	int result = 0;
	
	//Set gain
	for (int i = 255 ; i >= 0 ; i--)
	{
		POTCVALUE = i;
		BEAM2POT.setWiper(Wiper0, POTCVALUE);
		delay(1);
		tempV = analogRead(SIGC);
		tempFloat = (5000 / 1024.0) * (float)tempV;
		
		if (tempFloat >= target)
		{
			result = 1;
			break;
		}
	}
	
	if (POTCVALUE <= 1)
	{
		result = 2;
	}
	
	delay(10);
	
	tempV = analogRead(SIGC);
	tempFloat = (5.0 / 1024.0) * (float)tempV;
	tempV = (tempFloat * 1000) - delta;
	dac.voutWrite(3, tempV);
	
	return  result;
	
}

int autotuneD(float target, int delta)
{
	float tempFloat = 0.0;
	int tempV = 0;
	int result = 0;
	
	//Set gain
	for (int i = 255 ; i >= 0 ; i--)
	{
		POTDVALUE = i;
		BEAM2POT.setWiper(Wiper1, POTDVALUE);
		delay(1);
		tempV = analogRead(SIGD);
		tempFloat = (5000 / 1024.0) * (float)tempV;
		
		if (tempFloat >= target)
		{
			result = 1;
			break;
		}
	}
	
	if (POTDVALUE <= 1)
	{
		result = 2;
	}
	
	delay(10);

	tempV = analogRead(SIGD);
	tempFloat = (5.0 / 1024.0) * (float)tempV;
	tempV = (tempFloat * 1000) - delta;
	dac.voutWrite(2, tempV);
	
	return  result;
	
}


//=======================================================================================
//=================================Pin Change Interupts==================================
//=======================================================================================

void setupPCINT(int data)
{
	if (data == 0)
	{
		PCICR = 0x00;
	}
	
	PCMSK2 = 0x30;
	PCICR = 0x04;
}

ISR(PCINT2_vect)//Beam 2
{
	tempTimerCount = TCNT1;
	TIMSK1 = 0x00;
	
	if (shotInProgress)//Beam 1 + 2 trigg
	{
		shotSuccess = true;
		shotFlag = true;
		timerCount = tempTimerCount;
		shotInProgress = false;
	}
	//else//Beam 1 did not trigg //IGNORE BEAM 1 FAILS
	//{
	//shotSuccess = false;
	//shotFlag = true;
	//}
}


//=======================================================================================
//==================================External Interupts===================================
//=======================================================================================

//
void setupEXTINT(int data)
{
	if (data == 0)
	{
		EIMSK = 0x00;
	}
	
	EICRA = 0x0A;
	EIMSK = 0x03;
}

ISR(INT1_vect)//Beam 1
{
	if (shotInProgress == false)
	{
		TIMSK1 = 0x03;
		TCNT1 = 0;
		shotInProgress = true;
		testing1 = true;
	}
}

ISR(INT0_vect)//Beam 1
{
	if (shotInProgress == false)
	{
		TIMSK1 = 0x03;
		TCNT1 = 0;
		shotInProgress = true;
		testing1 = true;
	}
}


//=======================================================================================
//========================================TIMER 1========================================
//=======================================================================================

// 0     - Disable Timer
// 1     - Crono Timer setup
// other - RPS crono setup
void setupCronoTimer(int i)
{
	if (i == 0)
	{
		TCCR1A = 0x00;
		TCCR2A = 0x00;
		return;
	}
	
	if (i == 1)
	{
		TCCR1A = 0x00;
		TCCR1B = 0x02; //clk/8
	}
	else
	{
		TCCR1A = 0x00;
		TCCR1B = 0x03; //clk/64
	}
	
	TCNT1 = 0;
	OCR1A = 400;
	TIMSK1 = 0x00;
}

//
volatile void resetTimer1Count()
{
	TCNT1 = 0;
}


//
ISR(TIMER1_COMPA_vect)
{
	if (shotInProgress == true)
	{
		if (testing1 == false)
		{
			testing1 = true;
		}
		else
		{
			testing1 = false;
		}
	}
}

//Beam 1 trigg but beam 2 failed and timer overflowed
ISR(TIMER1_OVF_vect)
{
	if ((shotInProgress == true) && (testing1 == true))
	{
		shotSuccess = false;
		shotFlag = true;
		
		shotInProgress = false;
		testing1 = false;
		
		TIMSK1 = 0x00;
	}
	TIFR1 = 0x01;
}


//=======================================================================================
//========================================TIMER 2========================================
//=======================================================================================

void setupTimer2(int data)
{
	if (data == 0)
	{
		TCCR2B = 0x00;
	}
	
	TCCR2A = 0x00;
	TCCR2B = 0x00;
	TCCR2B = 0x07; // clk/1024
	TIMSK2 = 0x01;//  OVR FLAG
}

ISR(TIMER2_OVF_vect)
{
	timer2Count++;
	if (timer2Count >= timer2Limit)
	{
		timer2Flag = true;
		timer2Count = 0;
	}
}