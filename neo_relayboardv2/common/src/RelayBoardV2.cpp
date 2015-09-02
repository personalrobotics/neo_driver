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


#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include "../include/RelayBoardV2.h"


//-----------------------------------------------



#define RS422_BAUDRATE 420000

#define RS422_TIMEOUT 0.025


RelayBoardV2::RelayBoardV2()
{
	m_iNumBytesSend = 0;
	m_iNumBytesRec = 0;
	m_ihasRelayData = 0;
	m_ihas_LCD_DATA = 0;
	m_iHasIOBoard = 0;
	m_iHasUSBoard = 0;
	m_iHasSpeakerData = 0;
	m_iChargeState = 0;

	

	m_REC_MSG.iRelayBoard_Status = 0;
	m_REC_MSG.iCharging_Current = 0;
	m_REC_MSG.iCharging_State = 0;
	m_REC_MSG.iBattery_Voltage = 0;
	m_REC_MSG.iKey_Pad = 0;
	m_REC_MSG.iTemp_Sensor = 0;
	m_REC_MSG.iIODig_In = 0;
	m_REC_MSG.iIOBoard_State = 0;
	m_REC_MSG.iUSBoard_State = 0;

	for(int a = 0; a<8;a++)
	{
		m_Motor[a].lEnc = 0;
		m_Motor[a].lEncS = 0;
		m_Motor[a].iStatus = 0;
		m_Motor[a].ldesiredEncS = 0;
	}

	m_S_MSG.iSoftEM = 0;
	m_S_MSG.iCmdRelayBoard = 0;
	m_S_MSG.IOBoard_Cmd = 0;
	m_S_MSG.USBoard_Cmd = 0;
	m_S_MSG.Speaker = 0;
	m_S_MSG.SpeakerLoud = 0;

	m_blogging = false;

	ROS_INFO("Starting RelayBoardV2 Node\n");

}
//-----------------------------------------------
RelayBoardV2::~RelayBoardV2()
{
	m_SerIO.closeIO();
}
//----------------------------------------------
int RelayBoardV2::evalRxBuffer()
{
	int errorFlag = NO_ERROR;

	bool found_header = false;
	int waiting_cnt = 0;
	int waiting_cnt2 = 0;
	int msg_type = 100;
	int error_cnt = 0;
	int no_data_cnt = 0;
	int BytesToRead = 0;
	int received_checksum = 0;
	int my_checksum = 0;
	const int c_iSizeBuffer = 130;//4096;
	int iNrBytesRead = 0;
	unsigned char cDat[c_iSizeBuffer];
	unsigned char cHeader[4] = {0x00, 0x00, 0x00, 0x00};
	//ROS_INFO("While....");
	while(found_header == false)
	{
		if(m_SerIO.getSizeRXQueue() >= 1)
		{
			waiting_cnt = 0;
			//Read Header
			cHeader[3] = cHeader[2];
			cHeader[2] = cHeader[1];
			cHeader[1] = cHeader[0];
			iNrBytesRead = m_SerIO.readBlocking((char*)&cHeader[0], 1);

			if((cHeader[3] == 0x08) && (cHeader[2] == 0xFE) && (cHeader[1] == 0xEF) && (cHeader[0] == 0x08))
			{
				//Update Msg
				msg_type = 1;
				found_header = true;
			}
			else if((cHeader[3] == 0x02) && (cHeader[2] == 0x80) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
			{
				//Config Msg
				msg_type = 2;
				found_header = true;
			}
			else if((cHeader[3] == 0x02) && (cHeader[2] == 0xFF) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
			{
				//Error Msg
				ROS_INFO("Error Msg");
				msg_type = 3;
				found_header = true;
			}
			if(++error_cnt > 20)
			{
				//No Header in first 20 Bytes -> Error
				//flush input
				return 99;
			}

		}
		else
		{
			waiting_cnt++;
			if(waiting_cnt > 60000)
			{
				waiting_cnt = 0;
				waiting_cnt2++;
				if(waiting_cnt2 > 10)
				{
					//resend
					return 101;
				} 
			}
		}
	}
	switch(msg_type)
	{
		case 1: BytesToRead = m_iNumBytesRec - 4;
				break;
		case 2: BytesToRead = 6;
				break;
		case 3: BytesToRead = 3;
				break;
		default: return 98;
	}
	error_cnt = 0;
	while(m_SerIO.getSizeRXQueue() < BytesToRead)
	{
		usleep(2000);
	}
	iNrBytesRead = m_SerIO.readBlocking((char*)&cDat[0], BytesToRead);

	if(m_blogging == true)
	{
		log_to_file(2, cDat); //direction 1 = transmitted; 2 = recived
	}
	//Calc Checksum
	my_checksum += cHeader[3];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[2];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[1];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[0];
	for(int e = 0; e < iNrBytesRead - 2; e++)
	{
		my_checksum %= 0xFF00;
		my_checksum += cDat[e];
	}
	//received checksum
	received_checksum = (cDat[BytesToRead - 1] << 8);
	received_checksum += cDat[BytesToRead - 2];
	if(received_checksum != my_checksum)
	{
		//Wrong Checksum
		return 97;
	}
	if(msg_type == 1)
	{
		convRecMsgToData(&cDat[0]);
		return 0;
	}
	else if(msg_type == 2)
	{
		m_iFoundMotors = cDat[0];
		m_iHomedMotors = cDat[1];
		m_iFoundExtHardware = cDat[2];
		m_iConfigured = cDat[3];
		//ROS_INFO("Found Motors: %d",cDat[0]);
		//ROS_INFO("Homed Motors: %d",cDat[1]);
		//ROS_INFO("Ext Hardware: %d",cDat[2]);
		//ROS_INFO("Configuriert: %d",cDat[3]);
		return 1;
	}
	else if(msg_type == 3)
	{
		
		return 2;
	}
	else
	{
		return 100;
	}

	return 0;
}
//----------------------------------Configuration--------------------------------------
int RelayBoardV2::init(const char* cdevice_name, int iactive_motors, int ihoming_motors, int iext_hardware, long lModulo0, long lModulo1, long lModulo2, long lModulo3, long lModulo4, long lModulo5, long lModulo6, long lModulo7)
{
	m_SerIO.setDeviceName(cdevice_name);
	m_SerIO.setBaudRate(RS422_BAUDRATE);
	m_SerIO.openIO();
	//ROS_INFO("Homing Motors: %d",ihoming_motors);
	unsigned char cConfig_Data[33]; //4 Byte Header 3 Config Bytes 24 Byte Modulo 2 Byte Checksum
	int iChkSum = 0;
	int byteswritten = 0;
	int answertype = 0;
	int iNumDrives = 0;
	//Header
	cConfig_Data[0] = 0x02;
	cConfig_Data[1] = 0x80;
	cConfig_Data[2] = 0xD6;
	cConfig_Data[3] = 0x02;
	//-------------configuration Bytes-----------------------------------
	cConfig_Data[4] = iactive_motors;
	cConfig_Data[5] = ihoming_motors;
	cConfig_Data[6] = iext_hardware;

	//-------------set motors active-------------------------------------
	if((iactive_motors & 0x80) == 0x80) m_Motor[7].iMotorActive = 1;
	else m_Motor[7].iMotorActive = 0;
	if((iactive_motors & 0x40) == 0x40) m_Motor[6].iMotorActive = 1;
	else m_Motor[6].iMotorActive = 0;
	if((iactive_motors & 0x20) == 0x20) m_Motor[5].iMotorActive = 1;
	else m_Motor[5].iMotorActive = 0;
	if((iactive_motors & 0x10) == 0x10) m_Motor[4].iMotorActive = 1;
	else m_Motor[4].iMotorActive = 0;
	if((iactive_motors & 0x8) == 0x8) m_Motor[3].iMotorActive = 1;
	else m_Motor[3].iMotorActive = 0;
	if((iactive_motors & 0x4) == 0x4) m_Motor[2].iMotorActive = 1;
	else m_Motor[2].iMotorActive = 0;
	if((iactive_motors & 0x2) == 0x2) m_Motor[1].iMotorActive = 1;
	else m_Motor[1].iMotorActive = 0;
	if((iactive_motors & 0x1) == 0x1) m_Motor[0].iMotorActive = 1;
	else m_Motor[0].iMotorActive = 0;	
	//---------------Modulo for all Motors-------------------------------
	//Motor1
	cConfig_Data[7] = lModulo0 >> 16;
	cConfig_Data[8] = lModulo0 >> 8;
	cConfig_Data[9] = lModulo0;
	//Motor2
	cConfig_Data[10] = lModulo1 >> 16;
	cConfig_Data[11] = lModulo1 >> 8;
	cConfig_Data[12] = lModulo1;
	//Motor3
	cConfig_Data[13] = lModulo2 >> 16;
	cConfig_Data[14] = lModulo2 >> 8;
	cConfig_Data[15] = lModulo2;
	//Motor4
	cConfig_Data[16] = lModulo3 >> 16;
	cConfig_Data[17] = lModulo3 >> 8;
	cConfig_Data[18] = lModulo3;
	//Motor5
	cConfig_Data[19] = lModulo4 >> 16;
	cConfig_Data[20] = lModulo4 >> 8;
	cConfig_Data[21] = lModulo4;
	//Motor6
	cConfig_Data[22] = lModulo5 >> 16;
	cConfig_Data[23] = lModulo5 >> 8;
	cConfig_Data[24] = lModulo5;
	//Motor7
	cConfig_Data[25] = lModulo6 >> 16;
	cConfig_Data[26] = lModulo6 >> 8;
	cConfig_Data[27] = lModulo6;
	//Motor8
	cConfig_Data[28] = lModulo7 >> 16;
	cConfig_Data[29] = lModulo7 >> 8;
	cConfig_Data[30] = lModulo7;

	//-----------------Calc m_iNumBytesRec-----------------------------
	m_iNumBytesRec = 4; //4Byte Header
	m_iNumBytesRec += 13; //11Byte ActRelayboardConfig, State, Charging Current, Charging State, Batt Voltage, Keypad, Temp
	for(int g = 0; g < 8;g++)
	{
		if(m_Motor[g].iMotorActive == 1)
		{
			iNumDrives++;
		}
	}
	m_iNumBytesRec += (iNumDrives*10); //10 Byte pro active Motor

	if(m_iHasIOBoard == 1)
	{
		m_iNumBytesRec += 20;
	}
	if(m_iHasUSBoard == 1)
	{
		m_iNumBytesRec += 26;
	}

	m_iNumBytesRec += 2; //2Byte Checksum
	//ROS_INFO("iNumBytesRec: %d",m_iNumBytesRec);
	//----------------Calc Checksum-------------------------------------
	for(int i=4;i<=30;i++)    //i=4 => Header not added to Checksum
	{
		iChkSum %= 0xFF00;
		iChkSum += cConfig_Data[i];
	}
	//----------------END Calc Checksum---------------------------------

	//----------------Add Checksum to Data------------------------------
	cConfig_Data[31] = iChkSum >> 8;
	cConfig_Data[32] = iChkSum;
	//------------END Add Checksum to Data------------------------------
	if(m_blogging == true)
	{
		log_to_file(3, cConfig_Data); //direction 1 = transmitted; 2 = recived ; 3 = config
	}
	byteswritten = m_SerIO.writeIO((char*)cConfig_Data,33);
	if(byteswritten != 33)
	{
		//ROS_ERROR("FAILED: Could not write data to serial port!");
		return 60;
	}
	else
	{
		//ROS_INFO("Waiting for RelayBoard....");
	}

	answertype = evalRxBuffer();
	if(answertype == 1)
	{
		//Check received Data
		if(m_iConfigured == 1)
		{
			//ROS_INFO("Configured");
			return 1;
		}
		else
		{
			//ROS_ERROR("FAILED: Invalid configuration");
			return 59;
		}

	}
	
	return 50;
	

}
//-----------------------------------------------
bool RelayBoardV2::shutdownPltf()
{
	m_SerIO.closeIO();
	return true;
}
//-----------------------------------------------
bool RelayBoardV2::isEMStop()
{
	if( (m_REC_MSG.iRelayBoard_Status & 0x0001) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
//-----------------------------------------------
void RelayBoardV2::setEMStop()
{		
	m_S_MSG.iSoftEM |= 0x01;
	ROS_ERROR("Software Emergency Stop AKTIVE");
}

//-----------------------------------------------
void RelayBoardV2::resetEMStop()
{	
	m_S_MSG.iSoftEM |= 0x02;
	ROS_ERROR("Software Emergency Stop INAKTIVE");
}
//-----------------------------------------------
bool RelayBoardV2::isScannerStop()
{
	if( (m_REC_MSG.iRelayBoard_Status & 0x0002) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
//-------------------------------------------------
int RelayBoardV2::sendDataToRelayBoard()
{

	int errorFlag = NO_ERROR;
	int iNrBytesWritten;

	unsigned char cMsg[80];

	m_Mutex.lock();
	//ROS_INFO("Converting...");
	m_iNumBytesSend = convDataToSendMsg(cMsg);
	//ROS_INFO("Fin");	
	m_SerIO.purgeTx();
	iNrBytesWritten = m_SerIO.writeIO((char*)cMsg,m_iNumBytesSend);

	if(iNrBytesWritten < m_iNumBytesSend) {
		//errorFlag = GENERAL_SENDING_ERROR;
		//ROS_ERROR("ZU wenig gesendet!");
	}
	//log
	if(m_blogging == true)
	{
		log_to_file(1, cMsg); //direction 1 = transmitted; 2 = recived
	}
	//m_LastTime = ros::Time::now();
	m_Mutex.unlock();
	return errorFlag;

};
//---------------------------------------------------------------------------------------------------------------------
// MotorCtrl
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::setMotorDesiredEncS(int imotor_nr, long lEncS)
{		
	m_Mutex.lock();
	
	m_Motor[imotor_nr].ldesiredEncS = lEncS;

	m_Mutex.unlock();
}
//-------------------------------------------------------------
void RelayBoardV2::getMotorEnc(int imotor_nr,long* lmotorEnc)
{
	m_Mutex.lock();
	
	*lmotorEnc = m_Motor[imotor_nr].lEnc;
	
	m_Mutex.unlock();

}
//-------------------------------------------------------------
void RelayBoardV2::getMotorEncS(int imotor_nr,long* lmotorEncS)
{
	m_Mutex.lock();
	
	*lmotorEncS = m_Motor[imotor_nr].lEncS;

	m_Mutex.unlock();
}
//-------------------------------------------------------------
void RelayBoardV2::getMotorState(int imotor_nr,int* imotorStatus)
{
	m_Mutex.lock();
	
	*imotorStatus = m_Motor[imotor_nr].iStatus;

	m_Mutex.unlock();
}
//---------------------------------------------------------------------------------------------------------------------
// RelayBoard
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::getRelayBoardState(int* State)
{
	m_Mutex.lock();
	*State = m_REC_MSG.iRelayBoard_Status;
	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getRelayBoardDigOut(int* DigOut)
{
	m_Mutex.lock();
	*DigOut = m_S_MSG.iCmdRelayBoard;
	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::setRelayBoardDigOut(int iChannel, bool bOn)
{

      /*Bit 4: Relais 1 schalten
	Bit 5: Relais 2 schalten
	Bit 6: Relais 3 schalten
	Bit 7: Relais 4 schalten*/
	m_ihasRelayData = 1;
	switch( iChannel)
	{
	case 1:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 8; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 8; }
		
		break;

	case 2:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 16; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 16; }

		break;

	case 3:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 32; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 32; }

		break;

	case 4:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 64; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 64; }

		break;

	default:

		m_S_MSG.iCmdRelayBoard = 0;
	}
}
//-------------------------------------------------------
void RelayBoardV2::getTemperature(int* temp)
{
	m_Mutex.lock();

	*temp = m_REC_MSG.iTemp_Sensor;

	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getBattVoltage(int* battvolt)
{
	m_Mutex.lock();

	*battvolt = m_REC_MSG.iBattery_Voltage;

	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getChargingCurrent(int* current)
{
	m_Mutex.lock();

	*current = m_REC_MSG.iCharging_Current;

	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getChargingState(int* state)
{
	m_Mutex.lock();

	*state = m_REC_MSG.iCharging_State;

	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::writeLCD(const std::string& sText)
{
	int iSize;
	m_ihas_LCD_DATA = 1;
	m_Mutex.lock();

	iSize = sText.size();
	
	for(int i = 0; i < 20; i++)
	{
		if(i < iSize)
		{
			m_S_MSG.LCD_Txt[i] = sText[i];
		}
		else
		{
			m_S_MSG.LCD_Txt[i] = ' ';
		}
	}
	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getKeyPad(int* keypad)
{
	m_Mutex.lock();
	*keypad = m_REC_MSG.iKey_Pad;
	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::startCharging()
{
	m_Mutex.lock();
	m_ihasRelayData = 1;
	m_S_MSG.iCmdRelayBoard |= 1;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::stopCharging()
{
	m_Mutex.lock();
	m_ihasRelayData = 1;
	m_S_MSG.iCmdRelayBoard &= ~ 1;
	m_Mutex.unlock();
}
//-----------------------------------------------
// IOBoard
//-----------------------------------------------
void RelayBoardV2::getIOBoardDigIn(int* DigIn)
{
	m_Mutex.lock();
	*DigIn = m_REC_MSG.iIODig_In;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getIOBoardDigOut(int* DigOut)
{
	m_Mutex.lock();
	*DigOut = m_S_MSG.IOBoard_Cmd;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::setIOBoardDigOut(int iChannel, bool bVal)
{
	m_iHasIOBoard = 1;
	int iMask;

	iMask = (1 << iChannel);
	
	if(bVal)
	{
		m_S_MSG.IOBoard_Cmd |= iMask;
	}
	else
	{
		m_S_MSG.IOBoard_Cmd &= ~iMask;
	}	
}
//-----------------------------------------------
void RelayBoardV2::getIOBoardAnalogIn(int* iAnalogIn)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		iAnalogIn[i] = m_REC_MSG.iIOAnalog_In[i];
	}

	m_Mutex.unlock();
}
//---------------------------------------------------------------------------------------------------------------------
// USBoard
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::startUSBoard(int iChannelActive)
{
	m_Mutex.lock();
	m_iHasUSBoard = 1;
	m_S_MSG.USBoard_Cmd = iChannelActive;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::stopUSBoard()
{
	m_Mutex.lock();
	m_iHasUSBoard = 1;
	m_S_MSG.USBoard_Cmd = 0x00;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getUSBoardData1To8(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = 10 * m_REC_MSG.iUSSensor_Dist[i];
	}

	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getUSBoardData9To16(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = 10 * m_REC_MSG.iUSSensor_Dist[i + 8];
	}

	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getUSBoardAnalogIn(int* piAnalogIn)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 4; i++)
	{
		piAnalogIn[i] = m_REC_MSG.iUSAnalog_In[i];
	}

	m_Mutex.unlock();
}
//---------------------------------------------------------------------------------------------------------------------
//Logging
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::enable_logging()
{
	m_blogging = true;
}
//-----------------------------------------------
void RelayBoardV2::disable_logging()
{
	m_blogging = false;
}
//-----------------------------------------------
void RelayBoardV2::log_to_file(int direction, unsigned char cMsg[])
{
	FILE * pFile;
	//Open Logfile
	pFile = fopen ("neo_relayboard_RX_TX_log.log","a");
	//Write Data to Logfile
	if(direction == 1)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<m_iNumBytesSend; i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	if(direction == 2)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<(m_iNumBytesRec + 2); i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	if(direction == 3)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<(33); i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	//Close Logfile
	fclose (pFile);
}
//---------------------------------------------------------------------------------------------------------------------
//Data Converter
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::convRecMsgToData(unsigned char cMsg[])
{
	int data_in_message = 0;

	m_Mutex.lock();

	// convert data
	int iCnt = 0;

	//Has Data
	data_in_message = cMsg[iCnt];
	iCnt++;
	//Relayboard Status
	m_REC_MSG.iRelayBoard_Status = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Charging Current
	m_REC_MSG.iCharging_Current = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Charging Current: " << m_iChargeCurrent << std::endl; //JNN
	iCnt += 2;
	//Charging State
	m_REC_MSG.iCharging_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Charging State: " << m_iChargeState << std::endl; //JNN
	iCnt += 2;
	//Battery Voltage
	m_REC_MSG.iBattery_Voltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Battery Voltage: " << m_iRelBoardBattVoltage << std::endl; //JNN
	iCnt += 2;
	//Keypad
	m_REC_MSG.iKey_Pad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Temp Sensor
	m_REC_MSG.iTemp_Sensor = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	//std::cout << "Relayboard Temp: " << m_iRelBoardTempSensor << std::endl; //JNN
	iCnt += 2;

	// IOBoard
	if(m_iHasIOBoard)
	{
		m_REC_MSG.iIODig_In = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;

		for(int i = 0; i < 8; i++)
		{
			m_REC_MSG.iIOAnalog_In[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
		}

		m_REC_MSG.iIOBoard_State =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}
	// USBoard
	if(m_iHasUSBoard)
	{
		for(int i = 0; i < 16; i++)
		{
			m_REC_MSG.iUSSensor_Dist[i] = (cMsg[iCnt++]);
		}
		for(int i = 0; i < 4; i++)
		{
			m_REC_MSG.iUSAnalog_In[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
		}

		m_REC_MSG.iUSBoard_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}
	//Motor Data
	if(m_Motor[0].iMotorActive)
	{
		//Enc Data
		m_Motor[0].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M2 Enc: " << m_Motor[0].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[0].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M2 EncS: " << m_Motor[0].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[0].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M2 Status: " << m_Motor[0].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[1].iMotorActive)
	{
		//Enc Data
		m_Motor[1].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M3 Enc: " << m_Motor[1].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[1].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M3 EncS: " << m_Motor[1].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[1].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M3 Status: " << m_Motor[1].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[2].iMotorActive)
	{
		//Enc Data
		m_Motor[2].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M4 Enc: " << m_Motor[2].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[2].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M4 EncS: " << m_Motor[2].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[2].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M4 Status: " << m_Motor[2].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[3].iMotorActive)
	{
		//Enc Data
		m_Motor[3].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M5 Enc: " << m_Motor[3].lEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[3].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M5 EncS: " << m_Motor[3].lEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[3].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M5 Status: " << m_Motor[3].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[4].iMotorActive)
	{
		//Enc Data
		m_Motor[4].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M6 Enc: " << m_Motor[4].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[4].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M6 EncS: " << m_Motor[4].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[4].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M6 Status: " << m_Motor[4].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[5].iMotorActive)
	{
		//Enc Data
		m_Motor[5].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M7 Enc: " << m_Motor[5].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[5].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M7 EncS: " << m_Motor[5].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[5].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M7 Status: " << m_Motor[5].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[6].iMotorActive)
	{
		//Enc Data
		m_Motor[6].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M8 Enc: " << m_Motor[6].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[6].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M8 EncS: " << m_Motor[6].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[6].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M8 Status: " << m_Motor[6].iStatus << std::endl; //JNN
		iCnt += 2;
	}
	if(m_Motor[7].iMotorActive)
	{
		//Enc Data
		m_Motor[7].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M9 Enc: " << m_Motor[7].iPosMeasEnc << std::endl; //JNN
		iCnt += 4;
		//EncS Data
		m_Motor[7].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M9 EncS: " << m_Motor[7].iPosMeasEncS << std::endl; //JNN
		iCnt += 4;
		//Motor Status
		m_Motor[7].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		//std::cout << "M9 Status: " << m_Motor[7].iStatus << std::endl; //JNN
		iCnt += 2;
	}

	m_Mutex.unlock();
}

//------------------------------------------------------------
int RelayBoardV2::convDataToSendMsg(unsigned char cMsg[])
{
	//m_Mutex.lock();
	int iCnt = 0;
	int iChkSum = 0;

	int has_data = 0;
	int has_motor_data8 = 0;
	int has_motor_data4 = 0;

	if((m_Motor[0].iMotorActive) || (m_Motor[1].iMotorActive) || (m_Motor[2].iMotorActive) || (m_Motor[3].iMotorActive))
	{
		has_motor_data4 = 1;
		if((m_Motor[4].iMotorActive) || (m_Motor[5].iMotorActive) || (m_Motor[6].iMotorActive) || (m_Motor[7].iMotorActive))
		{
			has_motor_data8 = 1;
		}
	}
	//First Bit not in use yet
	has_data = (has_motor_data8 << 6) + (has_motor_data4 << 5) + (m_ihasRelayData << 4) + (m_ihas_LCD_DATA << 3) + (m_iHasIOBoard << 2) +(m_iHasUSBoard << 1) + (m_iHasSpeakerData);
	//Data in Message:
	//Header
	cMsg[iCnt++] = 0x02;
	cMsg[iCnt++] = 0xD6;
	cMsg[iCnt++] = 0x80;
	cMsg[iCnt++] = 0x02;
	//has_data
	//soft_em
	//Motor9-6
	//Motor5-2
	//Relaystates
	//LCD Data
	//IO Data
	//US Data
	//Speaker Data
	//Checksum
	cMsg[iCnt++] = has_data;
	cMsg[iCnt++] = m_S_MSG.iSoftEM; //SoftEM

	if(has_motor_data8)
	{
		//Motor 9
		cMsg[iCnt++] = ((m_Motor[7].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[7].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[7].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[7].ldesiredEncS & 0xFF);
		//Motor 8
		cMsg[iCnt++] = ((m_Motor[6].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[6].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[6].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[6].ldesiredEncS & 0xFF);
		//Motor 7
		cMsg[iCnt++] = ((m_Motor[5].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[5].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[5].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[5].ldesiredEncS & 0xFF);
		//Motor 6
		cMsg[iCnt++] = ((m_Motor[4].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[4].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[4].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[4].ldesiredEncS & 0xFF);
	}
	if(has_motor_data4)
	{
		//Motor 5
		cMsg[iCnt++] = ((m_Motor[3].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[3].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[3].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[3].ldesiredEncS & 0xFF);
		//Motor 4
		cMsg[iCnt++] = ((m_Motor[2].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[2].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[2].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[2].ldesiredEncS & 0xFF);
		//Motor 3
		cMsg[iCnt++] = ((m_Motor[1].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[1].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[1].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[1].ldesiredEncS & 0xFF);
		//Motor 2
		cMsg[iCnt++] = ((m_Motor[0].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[0].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[0].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[0].ldesiredEncS & 0xFF);
	}
	//Relaystates
	if(m_ihasRelayData)
	{
		//ROS_INFO("Relay Data: %d",m_S_MSG.iCmdRelayBoard);
		cMsg[iCnt++] = m_S_MSG.iCmdRelayBoard >> 8;
		cMsg[iCnt++] = m_S_MSG.iCmdRelayBoard;
	}
	//LCD Data
	if(m_ihas_LCD_DATA)
	{
		//ROS_INFO("Display: %s",m_S_MSG.LCD_Txt);
		for(int u = 0; u < 20; u++)
		{
			cMsg[iCnt++] = m_S_MSG.LCD_Txt[u];
		}
	}
	//IO Data
	if(m_iHasIOBoard)
	{
		cMsg[iCnt++] = m_S_MSG.IOBoard_Cmd >> 8;
		cMsg[iCnt++] = m_S_MSG.IOBoard_Cmd;
	}
	//US Data
	if(m_iHasUSBoard)
	{
		cMsg[iCnt++] = m_S_MSG.USBoard_Cmd >> 8;
		cMsg[iCnt++] = m_S_MSG.USBoard_Cmd;
	}
	//Speaker Data
	if(m_iHasSpeakerData)
	{
		cMsg[iCnt++] = m_S_MSG.Speaker >> 8;
		cMsg[iCnt++] = m_S_MSG.SpeakerLoud;
	}
	m_iNumBytesSend = iCnt;
	// calc checksum
	for(int i = 4; i < m_iNumBytesSend; i++)
	{
		iChkSum %= 0xFF00;
		iChkSum += cMsg[i];
	}
	cMsg[m_iNumBytesSend] = iChkSum >> 8;
	cMsg[m_iNumBytesSend + 1] = iChkSum;
	
	//resett data indicators
	m_iHasIOBoard = 0;
	m_ihasRelayData = 0;
	m_ihas_LCD_DATA = 0;
	m_iHasIOBoard = 0;
	m_iHasUSBoard = 0;
	m_iHasSpeakerData = 0;

	//m_Mutex.unlock();

	return m_iNumBytesSend + 2;
}
