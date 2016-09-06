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

#include <ros/ros.h>
#include <../include/neo_relayboardV2_node.h>
#include <math.h>



int neo_relayboardV2_node::init() 
{

//----------------------------------------GET PARAMS-----------------------------------------------------------
	//Relayboard Config Parameter
	if (n.hasParam("ComPort"))
	{
		n.getParam("ComPort", m_sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",m_sComPort.c_str());
	}

	n.getParam("hasIOBoard", m_ihasIOBoard);
	n.getParam("hasUSBoard", m_ihasUSBoard);

	//Node Parameter
	n.param("message_timeout", m_dRelayBoard_timeout, 2.0);
	n.param("requestRate", m_dRequestRate, 25.0);
	
	//Logging
	n.getParam("log", m_ilog);

	//Motor Parameter
	//Drive2
	n.param("drive2/motor_active", m_Drives[0].imotor_active, 0);
	n.param("drive2/homing_active", m_Drives[0].ihoming_active, 0);
	n.param("drive2/EncIncrPerRevMot", m_Drives[0].iEncIncrPerRevMot, 0);
	n.param("drive2/VelMeasFrqHz", m_Drives[0].dVelMeasFrqHz, 0.0);
	n.param("drive2/GearRatio", m_Drives[0].dGearRatio, 0.0);
	n.param("drive2/BeltRatio", m_Drives[0].dBeltRatio, 0.0);
	n.param("drive2/Sign", m_Drives[0].iSign, 0);
	n.param("drive2/VelMaxEncIncrS", m_Drives[0].dVelMaxEncIncrS, 0.0);
	n.param("drive2/VelPModeEncIncrS", m_Drives[0].dVelPModeEncIncrS, 0.0);
	n.param("drive2/AccIncrS2", m_Drives[0].dAccIncrS2, 0.0);
	n.param("drive2/DecIncrS2", m_Drives[0].dDecIncrS2, 0.0);
	n.param("drive2/Modulo", m_Drives[0].dModulo, 0.0);
	m_Drives[0].calcRadToIncr();
	//Drive3
	n.param("drive3/motor_active", m_Drives[1].imotor_active, 0);
	n.param("drive3/homing_active", m_Drives[1].ihoming_active, 0);
	n.param("drive3/EncIncrPerRevMot", m_Drives[1].iEncIncrPerRevMot, 0);
	n.param("drive3/VelMeasFrqHz", m_Drives[1].dVelMeasFrqHz, 0.0);
	n.param("drive3/GearRatio", m_Drives[1].dGearRatio, 0.0);
	n.param("drive3/BeltRatio", m_Drives[1].dBeltRatio, 0.0);
	n.param("drive3/Sign", m_Drives[1].iSign, 0);
	n.param("drive3/VelMaxEncIncrS", m_Drives[1].dVelMaxEncIncrS, 0.0);
	n.param("drive3/VelPModeEncIncrS", m_Drives[1].dVelPModeEncIncrS, 0.0);
	n.param("drive3/AccIncrS2", m_Drives[1].dAccIncrS2, 0.0);
	n.param("drive3/DecIncrS2", m_Drives[1].dDecIncrS2, 0.0);
	n.param("drive3/Modulo", m_Drives[1].dModulo, 0.0);
	m_Drives[1].calcRadToIncr();
	//Drive4
	n.param("drive4/motor_active", m_Drives[2].imotor_active, 0);
	n.param("drive4/homing_active", m_Drives[2].ihoming_active, 0);
	n.param("drive4/EncIncrPerRevMot", m_Drives[2].iEncIncrPerRevMot, 0);
	n.param("drive4/VelMeasFrqHz", m_Drives[2].dVelMeasFrqHz, 0.0);
	n.param("drive4/GearRatio", m_Drives[2].dGearRatio, 0.0);
	n.param("drive4/BeltRatio", m_Drives[2].dBeltRatio, 0.0);
	n.param("drive4/Sign", m_Drives[2].iSign, 0);
	n.param("drive4/VelMaxEncIncrS", m_Drives[2].dVelMaxEncIncrS, 0.0);
	n.param("drive4/VelPModeEncIncrS", m_Drives[2].dVelPModeEncIncrS, 0.0);
	n.param("drive4/AccIncrS2", m_Drives[2].dAccIncrS2, 0.0);
	n.param("drive4/DecIncrS2", m_Drives[2].dDecIncrS2, 0.0);
	n.param("drive4/Modulo", m_Drives[2].dModulo, 0.0);
	m_Drives[2].calcRadToIncr();
	//Drive5
	n.param("drive5/motor_active", m_Drives[3].imotor_active, 0);
	n.param("drive5/homing_active", m_Drives[3].ihoming_active, 0);
	n.param("drive5/EncIncrPerRevMot", m_Drives[3].iEncIncrPerRevMot, 0);
	n.param("drive5/VelMeasFrqHz", m_Drives[3].dVelMeasFrqHz, 0.0);
	n.param("drive5/GearRatio", m_Drives[3].dGearRatio, 0.0);
	n.param("drive5/BeltRatio", m_Drives[3].dBeltRatio, 0.0);
	n.param("drive5/Sign", m_Drives[3].iSign, 0);
	n.param("drive5/VelMaxEncIncrS", m_Drives[3].dVelMaxEncIncrS, 0.0);
	n.param("drive5/VelPModeEncIncrS", m_Drives[3].dVelPModeEncIncrS, 0.0);
	n.param("drive5/AccIncrS2", m_Drives[3].dAccIncrS2, 0.0);
	n.param("drive5/DecIncrS2", m_Drives[3].dDecIncrS2, 0.0);
	n.param("drive5/Modulo", m_Drives[3].dModulo, 0.0);
	m_Drives[3].calcRadToIncr();
	//Drive6
	n.param("drive6/motor_active", m_Drives[4].imotor_active, 0);
	n.param("drive6/homing_active", m_Drives[4].ihoming_active, 0);
	n.param("drive6/EncIncrPerRevMot", m_Drives[4].iEncIncrPerRevMot, 0);
	n.param("drive6/VelMeasFrqHz", m_Drives[4].dVelMeasFrqHz, 0.0);
	n.param("drive6/GearRatio", m_Drives[4].dGearRatio, 0.0);
	n.param("drive6/BeltRatio", m_Drives[4].dBeltRatio, 0.0);
	n.param("drive6/Sign", m_Drives[4].iSign, 0);
	n.param("drive6/VelMaxEncIncrS", m_Drives[4].dVelMaxEncIncrS, 0.0);
	n.param("drive6/VelPModeEncIncrS", m_Drives[4].dVelPModeEncIncrS, 0.0);
	n.param("drive6/AccIncrS2", m_Drives[4].dAccIncrS2, 0.0);
	n.param("drive6/DecIncrS2", m_Drives[4].dDecIncrS2, 0.0);
	n.param("drive6/Modulo", m_Drives[4].dModulo, 0.0);
	m_Drives[4].calcRadToIncr();
	//Drive7
	n.param("drive7/motor_active", m_Drives[5].imotor_active, 0);
	n.param("drive7/homing_active", m_Drives[5].ihoming_active, 0);
	n.param("drive7/EncIncrPerRevMot", m_Drives[5].iEncIncrPerRevMot, 0);
	n.param("drive7/VelMeasFrqHz", m_Drives[5].dVelMeasFrqHz, 0.0);
	n.param("drive7/GearRatio", m_Drives[5].dGearRatio, 0.0);
	n.param("drive7/BeltRatio", m_Drives[5].dBeltRatio, 0.0);
	n.param("drive7/Sign", m_Drives[5].iSign, 0);
	n.param("drive7/VelMaxEncIncrS", m_Drives[5].dVelMaxEncIncrS, 0.0);
	n.param("drive7/VelPModeEncIncrS", m_Drives[5].dVelPModeEncIncrS, 0.0);
	n.param("drive7/AccIncrS2", m_Drives[5].dAccIncrS2, 0.0);
	n.param("drive7/DecIncrS2", m_Drives[5].dDecIncrS2, 0.0);
	n.param("drive7/Modulo", m_Drives[5].dModulo, 0.0);
	m_Drives[5].calcRadToIncr();
	//Drive8
	n.param("drive8/motor_active", m_Drives[6].imotor_active, 0);
	n.param("drive8/homing_active", m_Drives[6].ihoming_active, 0);
	n.param("drive8/EncIncrPerRevMot", m_Drives[6].iEncIncrPerRevMot, 0);
	n.param("drive8/VelMeasFrqHz", m_Drives[6].dVelMeasFrqHz, 0.0);
	n.param("drive8/GearRatio", m_Drives[6].dGearRatio, 0.0);
	n.param("drive8/BeltRatio", m_Drives[6].dBeltRatio, 0.0);
	n.param("drive8/Sign", m_Drives[6].iSign, 0);
	n.param("drive8/VelMaxEncIncrS", m_Drives[6].dVelMaxEncIncrS, 0.0);
	n.param("drive8/VelPModeEncIncrS", m_Drives[6].dVelPModeEncIncrS, 0.0);
	n.param("drive8/AccIncrS2", m_Drives[6].dAccIncrS2, 0.0);
	n.param("drive8/DecIncrS2", m_Drives[6].dDecIncrS2, 0.0);
	n.param("drive8/Modulo", m_Drives[6].dModulo, 0.0);
	m_Drives[6].calcRadToIncr();
	//Drive9
	n.param("drive9/motor_active", m_Drives[7].imotor_active, 0);
	n.param("drive9/homing_active", m_Drives[7].ihoming_active, 0);
	n.param("drive9/EncIncrPerRevMot", m_Drives[7].iEncIncrPerRevMot, 0);
	n.param("drive9/VelMeasFrqHz", m_Drives[7].dVelMeasFrqHz, 0.0);
	n.param("drive9/GearRatio", m_Drives[7].dGearRatio, 0.0);
	n.param("drive9/BeltRatio", m_Drives[7].dBeltRatio, 0.0);
	n.param("drive9/Sign", m_Drives[7].iSign, 0);
	n.param("drive9/VelMaxEncIncrS", m_Drives[7].dVelMaxEncIncrS, 0.0);
	n.param("drive9/VelPModeEncIncrS", m_Drives[7].dVelPModeEncIncrS, 0.0);
	n.param("drive9/AccIncrS2", m_Drives[7].dAccIncrS2, 0.0);
	n.param("drive9/DecIncrS2", m_Drives[7].dDecIncrS2, 0.0);
	n.param("drive9/Modulo", m_Drives[7].dModulo, 0.0);
	m_Drives[7].calcRadToIncr();
//----------------------------------------END GET PARAMS-------------------------------------------------------
//----------------------------------------OPEN COMPORT---------------------------------------------------------
	//Check which motors are active
	if(m_Drives[0].imotor_active == 1) {m_iactive_motors += 1;m_imotor_count++;}
	if(m_Drives[1].imotor_active == 1) {m_iactive_motors += 2;m_imotor_count++;}
	if(m_Drives[2].imotor_active == 1) {m_iactive_motors += 4;m_imotor_count++;}
	if(m_Drives[3].imotor_active == 1) {m_iactive_motors += 8;m_imotor_count++;}
	if(m_Drives[4].imotor_active == 1) {m_iactive_motors += 16;m_imotor_count++;}
	if(m_Drives[5].imotor_active == 1) {m_iactive_motors += 32;m_imotor_count++;}
	if(m_Drives[6].imotor_active == 1) {m_iactive_motors += 64;m_imotor_count++;}
	if(m_Drives[7].imotor_active == 1) {m_iactive_motors += 128;m_imotor_count++;}
	//Check if homing is active
	if(m_Drives[0].ihoming_active == 1) m_ihoming_motors += 1;
	if(m_Drives[1].ihoming_active == 1) m_ihoming_motors += 2;
	if(m_Drives[2].ihoming_active == 1) m_ihoming_motors += 4;
	if(m_Drives[3].ihoming_active == 1) m_ihoming_motors += 8;
	if(m_Drives[4].ihoming_active == 1) m_ihoming_motors += 16;
	if(m_Drives[5].ihoming_active == 1) m_ihoming_motors += 32;
	if(m_Drives[6].ihoming_active == 1) m_ihoming_motors += 64;
	if(m_Drives[7].ihoming_active == 1) m_ihoming_motors += 128;
	//Check external hardware
	if(m_ihasIOBoard == 1) m_iext_hardware += 1;
	if(m_ihasUSBoard == 1) m_iext_hardware += 2;
	ROS_INFO("Parameters loaded");
	m_SerRelayBoard = new RelayBoardV2();
	int ret = m_SerRelayBoard->init(m_sComPort.c_str(),m_iactive_motors,m_ihoming_motors,m_iext_hardware,(long)m_Drives[0].dModulo,(long)m_Drives[1].dModulo,(long)m_Drives[2].dModulo,(long)m_Drives[3].dModulo,(long)m_Drives[4].dModulo,(long)m_Drives[5].dModulo,(long)m_Drives[6].dModulo,(long)m_Drives[7].dModulo);
	if(ret == 1) 
	{
		m_iRelayBoard_available = true;
		ROS_INFO("Opened RelayboardV2 at ComPort = %s", m_sComPort.c_str());
	}
	else	
	{
		ROS_ERROR("FAILED to open RelayboardV2 at ComPort = %s", m_sComPort.c_str());
		//eval Error
		return(1);
	}
//----------------------------------------END OPEN COMPORT-----------------------------------------------------
//----------------------------------------Init Publisher/Subscriber--------------------------------------------
	//topics and subscriber which will allways get published
	topicPub_isEmergencyStop = n.advertise<neo_msgs::EmergencyStopState>("/Emergency_Stop_State", 1);
	topicPub_batVoltage = n.advertise<std_msgs::Int16>("/RelayBoardV2/Battery_Voltage", 1);
	topicPub_chargeCurrent = n.advertise<std_msgs::Int16>("/RelayBoardV2/Charging/Charging_Current", 1);
	topicPub_chargeState = n.advertise<std_msgs::Int16>("/RelayBoardV2/Charging/Charging_State", 1);
	topicPub_temperatur = n.advertise<std_msgs::Int16>("/RelayBoardV2/Temperature", 1);
	topicSub_SetRelayStates = n.subscribe("/RelayBoardV2/Set_Relay_States",1,&neo_relayboardV2_node::getRelayBoardDigOut, this);
	topicPub_SendRelayStates = n.advertise<std_msgs::Int16>("/RelayBoardV2/Relay_States",1);
	topicPub_RelayBoardState = n.advertise<std_msgs::Int16>("/RelayBoardV2/State",1);
	topicSub_startCharging = n.subscribe("/RelayBoardV2/Charging/Start",1,&neo_relayboardV2_node::startCharging, this);
	topicSub_stopCharging = n.subscribe("/RelayBoardV2/Charging/Stop",1,&neo_relayboardV2_node::stopCharging, this);

	if(m_iactive_motors != 0)
	{
		topicPub_drives = n.advertise<sensor_msgs::JointState>("/Drives/JointStates",1);
		topicSub_drives = n.subscribe("/Drives/Set_Velocities",1,&neo_relayboardV2_node::getNewVelocitiesFomTopic, this);
	}

	topicSub_lcdDisplay = n.subscribe("/RelayBoardV2/Set_LCD_Message",1,&neo_relayboardV2_node::getNewLCDMsg, this);
	if(m_ihasIOBoard == 1)
	{
		topicSub_setDigOut = n.subscribe("/IOBoard/Set_Dig_Out",1,&neo_relayboardV2_node::getIOBoardDigOut, this);
		topicPub_ioDigIn = n.advertise<std_msgs::Int16>("/IOBoard/Dig_In",1);
		topicPub_ioDigOut = n.advertise<std_msgs::Int16>("/IOBoard/Dig_Out",1);
		topicPub_analogIn = n.advertise<neo_msgs::IOAnalogIn>("/IOBoard/Analog_in",1);

	}
	if(m_ihasUSBoard == 1)
	{
		topicPub_usBoard = n.advertise<neo_msgs::USBoard>("/USBoard/Measurements",1);
		topicSub_startUSBoard = n.subscribe("/USBoard/Start",1,&neo_relayboardV2_node::startUSBoard, this);
		topicSub_stopUSBoard = n.subscribe("/USBoard/Stop",1,&neo_relayboardV2_node::stopUSBoard, this);

	}
	topicPub_keypad = n.advertise<neo_msgs::Keypad>("/RelayBoardV2/Keypad",1);

	//logging
	if(m_ilog == 1)
	{
		ROS_INFO("Log enabled");
		m_SerRelayBoard->enable_logging();
	}
	else
	{
		ROS_INFO("Log disabled");
		m_SerRelayBoard->disable_logging();
	}
//----------------------------------------END Init Publisher/Subscriber----------------------------------------
	return 0;
}

//--------------------------RelayBoardV2-----------------------------------------------------------------------
void neo_relayboardV2_node::HandleCommunication() 
{
	if(!m_iRelayBoard_available) return;
	int send_return = 0;
	int receive_return = 101;
	static int last_error = 0;	
	while(receive_return == 101)
	{ 
		//ROS_INFO("Sending");
		send_return = m_SerRelayBoard->sendDataToRelayBoard();
		receive_return = m_SerRelayBoard->evalRxBuffer();
//ROS_INFO("Ret: %i", ret);
		if(receive_return == last_error)
		{
			//Do not show message again
		}
		else if(receive_return == 0) //ok
		{
			ROS_INFO("communicating with RelayBoard");
		}		
		else if(receive_return == 101) //No Answer => resend
		{
			ROS_ERROR("No answer from RelayBoard ... ");
		}
		else if(receive_return == 97)
		{
			ROS_ERROR("Wrong Checksum");
		}
		else if(receive_return == 99)
		{
			ROS_ERROR("No valid Message header found");
		}
		else
		{
			//Unknown error
			ROS_ERROR("Unknown error");
		}
		last_error = receive_return;
		
	}
}

double neo_relayboardV2_node::getRequestRate()
{
	return m_dRequestRate;
}
//-------Publisher------
void neo_relayboardV2_node::PublishRelayBoardState()
{
	if(!m_iRelayBoard_available) return;
	int value = 0;
	std_msgs::Int16 state;
	m_SerRelayBoard->getRelayBoardState(&value);
	state.data = value;
	topicPub_RelayBoardState.publish(state);
	
	//handle RelayBoardV2 shutdown
	//RelayBoardV2 will power off in < 30 sec
	if((value & 0x400) != 0)
	{
		ROS_INFO("-----------SHUTDOWN Signal from RelayBoardV2----------");
		ros::shutdown();
		usleep(2000);
		system("dbus-send --system --print-reply --dest=org.freedesktop.login1 /org/freedesktop/login1 \"org.freedesktop.login1.Manager.PowerOff\" boolean:true");
	}
}
void neo_relayboardV2_node::PublishTemperature()
{
	if(!m_iRelayBoard_available) return;
	int temperature = 0;
	std_msgs::Int16 temp;
	m_SerRelayBoard->getTemperature(&temperature);
	temp.data = temperature;
	topicPub_temperatur.publish(temp);
}
void neo_relayboardV2_node::PublishBattVoltage()
{
	if(!m_iRelayBoard_available) return;
	int battvolt = 0;
	std_msgs::Int16 bat;
	m_SerRelayBoard->getBattVoltage(&battvolt);
	bat.data = battvolt/1000;	
	topicPub_batVoltage.publish(bat);
}
void neo_relayboardV2_node::PublishChargingCurrent()
{
	if(!m_iRelayBoard_available) return;
	int value = 0;
	std_msgs::Int16 current;
	m_SerRelayBoard->getChargingCurrent(&value);
	current.data = value;	
	topicPub_chargeCurrent.publish(current);
}
void neo_relayboardV2_node::PublishChargingState()
{
	if(!m_iRelayBoard_available) return;
	int value = 0;
	std_msgs::Int16 state;
	m_SerRelayBoard->getChargingState(&value);
	state.data = value;	
	topicPub_chargeState.publish(state);
}
void neo_relayboardV2_node::PublishKeyPad()
{
	if(!m_iRelayBoard_available) return;
	int value = 0;
	neo_msgs::Keypad pad;
	m_SerRelayBoard->getKeyPad(&value);
	pad.button[0] = !(value & 1); //Info Taste	
	pad.button[1] = !(value & 2); //Home Taste	
	pad.button[2] = !(value & 4); //Start Taste	
	pad.button[3] = !(value & 8); //Stop Taste
	pad.button[4] = !(value & 16); //Bremsen lösen Taste	
	topicPub_keypad.publish(pad);
}
void neo_relayboardV2_node::PublishEmergencyStopStates()
{
	if(!m_iRelayBoard_available) return;
	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;
	neo_msgs::EmergencyStopState EM_msg;

	// assign input (laser, button) specific EM state
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (m_iEM_stop_state)
	{
		case ST_EM_FREE:
		{
			if (EM_signal == true)
			{
				ROS_ERROR("Emergency stop was issued");
				m_iEM_stop_state = EM_msg.EMSTOP;
			}
			break;
		}
		case ST_EM_ACTIVE:
		{
			if (EM_signal == false)
			{
				ROS_INFO("Emergency stop was confirmed");
				m_iEM_stop_state = EM_msg.EMCONFIRMED;
				m_time_of_EM_confirmed = ros::Time::now();
			}
			break;
		}
		case ST_EM_CONFIRMED:
		{
			if (EM_signal == true)
			{
				ROS_ERROR("Emergency stop was issued");
				m_iEM_stop_state = EM_msg.EMSTOP;
			}
			else
			{
				duration_since_EM_confirmed = ros::Time::now() - m_time_of_EM_confirmed;
				if( duration_since_EM_confirmed.toSec() > m_duration_for_EM_free.toSec() )
				{
					ROS_INFO("Emergency stop released");
					m_iEM_stop_state = EM_msg.EMFREE;
				}
			}
			break;
		}
	};

	EM_msg.emergency_state = m_iEM_stop_state;

	
	topicPub_isEmergencyStop.publish(EM_msg);
}
void neo_relayboardV2_node::PublishRelayBoardDigOut()
{
	if(!m_iRelayBoard_available) return;
	int data = 0;
 	std_msgs::Int16 i;
	m_SerRelayBoard->getRelayBoardDigOut(&data);
	i.data = data;
	topicPub_SendRelayStates.publish(i);

}
//-------Subscriber------
void neo_relayboardV2_node::getNewLCDMsg(const neo_msgs::LCDOutput& msg) 
{
	if(!m_iRelayBoard_available) return;
	m_SerRelayBoard->writeLCD(msg.msg_line);

}
void neo_relayboardV2_node::getRelayBoardDigOut(const neo_msgs::IOOut& setOut)
{
	if(!m_iRelayBoard_available) return;
	m_SerRelayBoard->setRelayBoardDigOut(setOut.channel, setOut.active);
}
void neo_relayboardV2_node::startCharging(const std_msgs::Empty& empty)
{
	if(!m_iRelayBoard_available) return;
	m_SerRelayBoard->startCharging();
}

void neo_relayboardV2_node::stopCharging(const std_msgs::Empty& empty)
{
	if(!m_iRelayBoard_available) return;
	m_SerRelayBoard->stopCharging();
}
//----------------------END RelayBoardV2-----------------------------------------------------------------------
//--------------------------Motor Ctrl-------------------------------------------------------------------------
void neo_relayboardV2_node::PublishJointStates()
{
	ROS_INFO("PUB_JOINT_STATES");
	if(!m_iRelayBoard_available) return;
	long lEnc[8] = {0,0,0,0,0,0,0,0};
	long lEncS[8] = {0,0,0,0,0,0,0,0};
	int iStatus[8] = {0,0,0,0,0,0,0,0};
	int iMotor_Nr = 0;
	static float sfLastPos[8] = {0,0,0,0,0,0,0,0};
	
	sensor_msgs::JointState state;

	//Publish Data for all possible Motors
	state.name.resize(8);
	state.position.resize(8);
	state.velocity.resize(8);

	//TODO Joint Names einfügen
	//for(int i = 0; i<anz_drives; i++)  state.name[i] = joint_names[i];

	//Motor Data from MSG Handler for each Motor
	//Enc (4 Byte), EncS (4 Byte) and Status (2 Byte) for each Motor 
	for(int i = 0; i<8; i++)
	{
		m_SerRelayBoard->getMotorEnc(i,&lEnc[i]);
		m_SerRelayBoard->getMotorEncS(i,&lEncS[i]);
		m_SerRelayBoard->getMotorState(i,&iStatus[i]);
		//m_SerRelayBoard->setMotorDesiredEncS(i, 20000);
		state.position[i] = (float)lEnc[i] - sfLastPos[i];
		sfLastPos[i] = (float)lEnc[i];
		//ROS_INFO("Motor %d: Enc: %f",i,(float)lEnc[i]);
		state.velocity[i] = m_Drives[i].iSign * m_Drives[i].convIncrPerPeriodToRadS((float)lEncS[i]);	 
	}
	topicPub_drives.publish(state);
	ROS_INFO("ENDE PUB_JOINT_STATES");
}

void neo_relayboardV2_node::getNewVelocitiesFomTopic(const trajectory_msgs::JointTrajectory jt)
{
	if(!m_iRelayBoard_available) return;
	ROS_INFO("SUB NEW_VEL");
	double dvelocity = 0.0;
	trajectory_msgs::JointTrajectoryPoint point = jt.points[0];
	//Check if Data for all Motors are avaliable
	//ROS_INFO("Data in Topic: %d ; Motor Nr: %d",sizeof(point.velocities),m_imotor_count);
	//if(sizeof(point.velocities) != m_imotor_count)
	//{
	//	ROS_ERROR("TOO LESS DATA FOR ALL MOTORS IN TOPIC!");
	//}
	//else
	//{
		//set new velocities
		ROS_INFO("1");
		for(int i=0; i<m_imotor_count; i++)
		{
			ROS_INFO("2");
			//convert velocities [rad/s] -> [incr/period]
			//ROS_INFO("Motor: %d ; Vel: %d [rad]; Vel: %d [incr/period]",i,point.velocities[i],dvelocity);
			dvelocity = m_Drives[i].iSign * m_Drives[i].convRadSToIncrPerPeriod(point.velocities[i]);
			//check if velocity is too high -> limit velocity
			/*if(MathSup::limit((int)&dvelocity, (int)Drives[i].getVelMax()) != 0)
			{
				ROS_ERROR("Velocity for motor %d limited",i+2);
			}*/
			//send Data to MSG Handler
			m_SerRelayBoard->setMotorDesiredEncS(i, (long)dvelocity);
		}
	//}
	ROS_INFO("END_SUB_NEW_VEL");
}
//----------------------END Motor Ctrl-------------------------------------------------------------------------
//-----------------------------USBoard-------------------------------------------------------------------------
void neo_relayboardV2_node::PublishUSBoardData()
{
	if(!m_iRelayBoard_available || !m_ihasUSBoard) return;
	int usSensors[8];
	int usAnalog[4];
	neo_msgs::USBoard usBoard;
	m_SerRelayBoard->getUSBoardData1To8(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i] = usSensors[i];
	m_SerRelayBoard->getUSBoardData9To16(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i+8] = usSensors[i];
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);
	for(int i=0; i<4; i++) usBoard.analog[i] = usAnalog[i];	
	topicPub_usBoard.publish(usBoard);
}
void neo_relayboardV2_node::startUSBoard(const std_msgs::Int16& configuration)
{
	if(!m_iRelayBoard_available || !m_ihasUSBoard) return;
	m_SerRelayBoard->startUSBoard(configuration.data);
}

void neo_relayboardV2_node::stopUSBoard(const std_msgs::Empty& empty)
{
	if(!m_iRelayBoard_available || !m_ihasUSBoard) return;
	m_SerRelayBoard->stopUSBoard();
}
//-----------------------------END USBoard---------------------------------------------------------------------
//---------------------------------IOBoard---------------------------------------------------------------------
void neo_relayboardV2_node::getIOBoardDigOut(const neo_msgs::IOOut& setOut)
{
	if(!m_iRelayBoard_available || !m_ihasIOBoard) return;
	m_SerRelayBoard->setIOBoardDigOut(setOut.channel, setOut.active);
}

void neo_relayboardV2_node::PublishIOBoardDigIn()
{
	if(!m_iRelayBoard_available || !m_ihasIOBoard) return;
	int data = 0;
 	std_msgs::Int16 i;
	m_SerRelayBoard->getIOBoardDigIn(&data);
	i.data = data;
	topicPub_ioDigIn.publish(i);

}

void neo_relayboardV2_node::PublishIOBoardDigOut()
{
	if(!m_iRelayBoard_available || !m_ihasIOBoard) return;
	int data = 0;
 	std_msgs::Int16 i;
	m_SerRelayBoard->getIOBoardDigOut(&data);
	i.data = data;
	topicPub_ioDigOut.publish(i);
}

void neo_relayboardV2_node::PublishIOBoardAnalogIn()
{
	if(!m_iRelayBoard_available || !m_ihasIOBoard) return;
	int *pointer = 0;
	int analogin[8];
	neo_msgs::IOAnalogIn in;
	pointer = analogin;
	m_SerRelayBoard->getIOBoardAnalogIn(pointer);
	for(int i=0;i <8; i++) in.input[i] = pointer[i];
	topicPub_analogIn.publish(in);
}
//-----------------------------END IOBoard---------------------------------------------------------------------
