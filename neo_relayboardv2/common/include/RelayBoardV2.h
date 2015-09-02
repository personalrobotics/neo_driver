/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef SerRelayBoard_INCLUDEDEF_H
#define SerRelayBoard_INCLUDEDEF_H

//-----------------------------------------------


#include <SerialIO.h>
#include <Mutex.h>
#include <StrUtil.h>
#include <DriveParam.h>


#include <vector>
#include <boost/array.hpp>

//-----------------------------------------------

/**
 * Message handler class for communication with a Neobotix RelayBoardV2.
 * 
 */
class RelayBoardV2
{
public:
	//Constructor
	RelayBoardV2();
	//Deconstructor
	~RelayBoardV2();

//------------------------------Initialisation-------------------------------------------------------------------------
	int openSerial(const char* device,int baudrate); //TODO
	int init(const char* device_name, int iactive_motors, int ihoming_motors, int iext_hardware, long lModulo0, long lModulo1, long lModulo2, long lModulo3, long lModulo4, long lModulo5, long lModulo6, long lModulo7);

	bool shutdownPltf();
//-------------------------------Msg Handling--------------------------------------------------------------------------
	int evalRxBuffer();

	int sendDataToRelayBoard();

	int convDataToSendMsg(unsigned char cMsg[]);

	void convRecMsgToData(unsigned char cMsg[]);
//------------------------------Set Data for next Message--------------------------------------------------------------

	//RelayBoard
	void startCharging();
	void stopCharging();
	void setRelayBoardDigOut(int iChannel, bool bOn);	

	//Motors
	void setMotorDesiredEncS(int motor_nr, long dVel);
	
	//EM-Stop
	void setEMStop();	
	void resetEMStop();

	//LCD
	void writeLCD(const std::string& sText);

	//USBoard
	void startUSBoard(int iChannelActive);
	void stopUSBoard();

	//IOBoard
	void setIOBoardDigOut(int iChannel, bool bVal);
//------------------------------Get Data for Topics--------------------------------------------------------------------
	//EM-Stop
	bool isEMStop();
	bool isScannerStop();

	//RelayBoard Data
	void getRelayBoardState(int* State);
	void getTemperature(int* temp);
	void getBattVoltage(int* battvolt);
	void getChargingCurrent(int* current);
	void getChargingState(int* state);
	void getRelayBoardDigOut(int* DigOut);
	void getKeyPad(int* keypad);

	//Motors
	void getMotorEnc(int imotor_nr,long* lmotorEnc);
	void getMotorEncS(int imotor_nr,long* lmotorEncS);
	void getMotorState(int imotor_nr,int* lmotorStatus);
	
	//USBoard
	void getUSBoardData1To8(int* piUSDistMM);
	void getUSBoardData9To16(int* piUSDistMM);
	void getUSBoardAnalogIn(int* piAnalogIn);
	
	//IOBoard
	void getIOBoardDigIn(int* DigIn);
	void getIOBoardDigOut(int* DigOut);
	void getIOBoardAnalogIn(int* iAnalogIn);
//------------------------------Logging--------------------------------------------------------------------------------
	void enable_logging();
	void disable_logging();
	void log_to_file(int direction, unsigned char cMsg[]);	

	enum RelBoardReturns
	{
		NO_ERROR = 0,
		NOT_INITIALIZED = 1,
		GENERAL_SENDING_ERROR = 2,
		TOO_LESS_BYTES_IN_QUEUE = 3,
		NO_MESSAGES = 4, //for a long time, no message have been received, check com port!
		CHECKSUM_ERROR = 5,
		MSG_CONFIG = 6,
		MSG_DATA = 7
	};
private:

	SerialIO m_SerIO;

	Mutex m_Mutex;

	//Config Bits
	int m_iFoundMotors;
	int m_iHomedMotors;
	int m_iFoundExtHardware;
	int m_iConfigured;
	int m_iNumBytesRec;
	int m_iNumBytesSend;

	//data indicators
	int m_ihasRelayData;
	int m_ihas_LCD_DATA;
	int m_iHasIOBoard;
	int m_iHasUSBoard;
	int m_iHasSpeakerData; 
	int m_iChargeState;

	struct Motor
	{
                int iMotorActive;
		long lEnc;
		long lEncS;
		int iStatus;
		long ldesiredEncS;
	};
	Motor m_Motor[8];

	struct Send_MSG
	{
		int iSoftEM;
		int iCmdRelayBoard;
		char LCD_Txt[20];
		int IOBoard_Cmd;
		int USBoard_Cmd;
		int Speaker;
		int SpeakerLoud;
	};
	Send_MSG m_S_MSG;	

	struct Recived_MSG
	{
		//normal
		int iRelayBoard_Status;
		int iCharging_Current;
		int iCharging_State;
		int iBattery_Voltage;
		int iKey_Pad;
		int iTemp_Sensor;
		
		//IOBoard 
		int iIODig_In;
		int iIOAnalog_In[8];
		int iIOBoard_State;

		//USBoard
		int iUSSensor_Dist[16];
		int iUSAnalog_In[4];
		int iUSBoard_State;	
	};

	Recived_MSG m_REC_MSG;

	bool m_blogging;
};


//-----------------------------------------------
#endif
