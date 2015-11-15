/******************************************************************************
* File Name          : UF_uArm.h
* Author             : Evan
* Updated            : Evan
* Version            : V0.0.1 (BATE)
* Created Date       : 12 Dec, 2014
* Modified Date      : 12 Dec, 2014
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "VarSpeedServo.h"

#ifndef UF_uArm_Metal_new_h
#define UF_uArm_Metal_new_h

/****************  Macro definitions  ****************/
#define ARM_A                   148    // upper arm
#define ARM_B                   160    // lower arm
#define ARM_2AB                 47360  // 2*A*B
#define ARM_A2                  21904  // A^2
#define ARM_B2                  25600  // B^2
#define ARM_A2B2                47504  // A^2 + B^2
#define ARM_STRETCH_MIN         0
#define ARM_STRETCH_MAX         195
#define ARM_HEIGHT_MIN          -150
#define ARM_HEIGHT_MAX          160
#define ARM_ROTATION_MIN        -90
#define ARM_ROTATION_MAX        90
#define HAND_ROTATION_MIN       -90
#define HAND_ROTATION_MAX       90
#define HAND_ANGLE_OPEN         25
#define HAND_ANGLE_CLOSE        70
#define FIXED_OFFSET_L          25
#define FIXED_OFFSET_R          42
#define D090M_SERVO_MIN_PUL     500
#define D090M_SERVO_MAX_PUL     2500
#define D009A_SERVO_MIN_PUL     600
#define D009A_SERVO_MAX_PUL     2550
#define SAMPLING_DEADZONE       2
#define INIT_POS_L              139//37   // the angle of calibration position (initial angle)
#define INIT_POS_R              26//25   // the angle of calibration position (initial angle)
#define BTN_TIMEOUT_1000        1000
#define BTN_TIMEOUT_3000        3000
#define CATCH						0x01
#define RELEASE						0x02
#define CALIBRATION_FLAG				0xEE
#define SERVO_MAX					605
#define SERVO_MIN					80
#define MEMORY_SERVO_PER				32760//335   //  eeprom: (1024 - 3 - 14)/3=335
#define DATA_FLAG					255
#define BUFFER_OUTPUT					555
/*****************  Port definitions  *****************/
#define BTN_D4                  4     //
#define BTN_D7                  7     //
#define BUZZER                  3     //
#define LIMIT_SW                2     // Limit Switch
#define PUMP_EN                 6     //
#define VALVE_EN                5     //
#define SERVO_HAND              9     //
#define SERVO_HAND_ROT          10    //
#define SERVO_ROT               11    //
#define SERVO_R                 12    //
#define SERVO_L                 13    //

#define old_data                1
#define new_data                2

#define have_done_record_since_power      1
#define have_not_done_record_since_power  0

class UF_uArm
{
public:
	UF_uArm();
	void init();    // initialize the uArm position
    void calibration();  //
	void recordingMode(unsigned char _sampleDelay = 50);
	void setPosition(double _stretch, double _height, int _armRot, int _handRot);    // 
	void setServoSpeed(char _servoNum, unsigned char _servoSpeed); // 0=full speed, 1-255 slower to faster
	int readAngle(char _servoNum);
	void gripperCatch();    //
	void gripperRelease();  //
	void gripperDetach();   //
    void gripperDirectDetach(); //
    void pumpOn();          // pump enable
    void pumpOff();         // pump disnable
    void valveOn();         // valve enable, decompression
    void valveOff();        // valve disnable
    void detachServo(char _servoNum);
	void sendData(byte _dataAdd, int _dataIn); //
	void alert(int _times, int _runTime, int _stopTime);
	void writeEEPROM();
        void writeExternalEeprom(unsigned int address,unsigned char * data_array,int num);
	void readEEPROM();
        void readExternalEeprom(unsigned int address, unsigned char * data_array,int num);
	void play(unsigned char buttonPin);
	void record(unsigned char buttonPin, unsigned char buttonPinC);
	void servoBufOutL(unsigned char _lastDt, unsigned char _dt);
	void servoBufOutR(unsigned char _lastDt, unsigned char _dt);
	void servoBufOutRot(unsigned char _lastDt, unsigned char _dt);
        void servoBufOutHandRot(unsigned char _lastDt, unsigned char _dt);
        void run(int x, int y, int z);

private:
	/*******************  Servo offset  *******************/
	char offsetL;
	char offsetR;
	/*****************  Define variables  *****************/
        unsigned int record_status=0;
        unsigned int addr;
	int heightLst;
	int height;
	int stretch; 
	int rotation; 
	int handRot;
	int handrotLast;
	boolean playFlag;
    boolean recordFlag;
    boolean firstFlag;
	boolean gripperRst;
	unsigned char sampleDelay;
	unsigned char servoSpdR;
	unsigned char servoSpdL;
	unsigned char servoSpdRot;
	unsigned char servoSpdHand;
	unsigned char servoSpdHandRot;
	unsigned char leftServoLast;
    unsigned char rightServoLast;
    unsigned char rotServoLast;
	unsigned char griperState[14];
	unsigned char data[5];  // 0: L  1: R  2: Rotation 
    unsigned long delay_loop;
    unsigned long lstTime;  //limit: 50days
	/***************  Create servo objects  ***************/
	VarSpeedServo servoR;
	VarSpeedServo servoL;
	VarSpeedServo servoRot;
	VarSpeedServo servoHand;
	VarSpeedServo servoHandRot;
};

#endif

