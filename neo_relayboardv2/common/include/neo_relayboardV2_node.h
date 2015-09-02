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

// ROS includes
#include <ros/ros.h>
#include <iostream>
#include <RelayBoardV2.h>

// ROS message includes

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <neo_msgs/EmergencyStopState.h>
#include <neo_msgs/Keypad.h>
#include <neo_msgs/LCDOutput.h>
#include <neo_msgs/USBoard.h>
#include <neo_msgs/IOOut.h>
#include <neo_msgs/IOAnalogIn.h>
#include <neo_msgs/PowerState.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class neo_relayboardV2_node
{
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n;
                
		// topics:
		//basic topics:
		ros::Publisher topicPub_RelayBoardState;
		ros::Publisher topicPub_isEmergencyStop;
		ros::Publisher topicPub_batVoltage;
		ros::Publisher topicPub_chargeCurrent;
		ros::Publisher topicPub_chargeState;
		ros::Publisher topicPub_temperatur;
		ros::Publisher topicPub_keypad;
		ros::Subscriber topicSub_startCharging;
		ros::Subscriber topicSub_stopCharging;

		ros::Publisher topicPub_SendRelayStates;
		ros::Subscriber topicSub_SetRelayStates;
		//drives:
		ros::Publisher topicPub_drives;			//
		ros::Subscriber topicSub_drives;		//
		//ultrasonic board:
		ros::Publisher topicPub_usBoard;
		ros::Subscriber topicSub_startUSBoard;
		ros::Subscriber topicSub_stopUSBoard;		
		//io board:
		ros::Publisher topicPub_ioDigIn;
		ros::Publisher topicPub_ioDigOut;
		ros::Publisher topicPub_analogIn;
		ros::Subscriber topicSub_lcdDisplay;
		ros::Subscriber topicSub_setDigOut;


		// Constructor
		neo_relayboardV2_node()
		{
			// Make sure member variables have a defined state at the beginning
			m_iEM_stop_state = ST_EM_FREE;
			m_iRelayBoard_available = 0;
			m_dRelayBoard_timeout = 2.0;
			m_duration_for_EM_free = ros::Duration(1);
			m_iactive_motors = 0;
			m_imotor_count = 0;
			m_ihoming_motors = 0;
			m_iext_hardware = 0;
			m_ihasIOBoard = 0;
			m_ihasUSBoard = 0;
		}
        
		// Destructor
		~neo_relayboardV2_node() 
		{
			delete m_SerRelayBoard;
		}

		//Comm Handler
		int init();
		void HandleCommunication();

		// Topic Callbacks
		void getNewVelocitiesFomTopic(const trajectory_msgs::JointTrajectory jt);
		void startUSBoard(const std_msgs::Int16& configuration);
		void stopUSBoard(const std_msgs::Empty& empty);
		void getRelayBoardDigOut(const neo_msgs::IOOut& setOut);	
		void getNewLCDMsg(const neo_msgs::LCDOutput& msg);
		void getIOBoardDigOut(const neo_msgs::IOOut& setOut);

		void startCharging(const std_msgs::Empty& empty);
		void stopCharging(const std_msgs::Empty& empty);	
		
		// Pubisher functions
		//RelayBoard
		void PublishRelayBoardState();
		void PublishTemperature();
		void PublishBattVoltage();
		void PublishChargingCurrent();
		void PublishChargingState();
		void PublishEmergencyStopStates();
		void PublishRelayBoardDigOut();
		void PublishKeyPad();
		//Motors
		void PublishJointStates();
		//USBoard		
		void PublishUSBoardData();
		//IOBoard
		void PublishIOBoardAnalogIn();
		void PublishIOBoardDigIn();
		void PublishIOBoardDigOut();
		
		double getRequestRate();
	private:

		//----Configuration--------
		int m_iactive_motors;
		int m_imotor_count;
		int m_ihoming_motors;
		int m_iext_hardware;
		int m_ihasIOBoard;
		int m_ihasUSBoard;
		int m_iRelayBoard_available; //the RelayBoard has confirmed the configuration and is ready
		std::string m_sComPort;
		RelayBoardV2 * m_SerRelayBoard;
		
		DriveParam m_Drives[8];

		//----EM Stop Handling------
		int m_iEM_stop_state;
		ros::Duration m_duration_for_EM_free;
		ros::Time m_time_of_EM_confirmed;
		// possible states of emergency stop
		enum
		{
			ST_EM_FREE = 0,
			ST_EM_ACTIVE = 1,
			ST_EM_CONFIRMED = 2
		};
		//----Msg Handling------------
		double m_dRelayBoard_timeout;
		double m_dRequestRate;
		ros::Time m_time_last_message_received;
		

		//log
		int m_ilog;  //enables or disables the log for neo_relayboard
};

