/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2022 National Council of Research of Italy (CNR)
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#ifndef __MQTT_TO_JOINT_POSITION_CONTROLLER_H__
#define __MQTT_TO_JOINT_POSITION_CONTROLLER_H__

#include <sensor_msgs/JointState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <position_controllers/joint_group_position_controller.h>
#include <drapebot_mqtt_client/drapebot_mqtt_client.h>
#include <realtime_tools/realtime_publisher.h>


namespace drapebot_controller 
{
    class MQTTToPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
    public:
        MQTTToPositionController();
        ~MQTTToPositionController();

        virtual bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n) final;
        void update   (const ros::Time& time, const ros::Duration& period) final;
        void starting (const ros::Time& time) final;
        void stopping (const ros::Time& time) final;
        void waiting  (const ros::Time& time) final;
        void aborting (const ros::Time& time) final;

    private:

        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> command_pub_;
      
        bool first_cycle_;
        bool topics_subscribed_;
        bool first_ros_cmd_msg_rec_;
        bool use_ros_command_;

        unsigned long int counter_, loss_packages_;

        std::string mqtt_command_topic_;
        std::string ros_command_topic_;
        cnr::drapebot::drapebot_msg command_from_mqtt_;
        cnr::drapebot::MQTTDrapebotClient* mqtt_drapebot_client_;

        std::vector<std::string> joint_names_;
        std::vector<double> j_pos_command_;   
        std::vector<double> j_pos_command_ros_;            
        position_controllers::JointGroupPositionController ctrl_;

        ros::Subscriber robot_cmd_ros_sub_;    
        void CmdJointStatejCallback(const sensor_msgs::JointState::ConstPtr& msg);
    };

} //end drapebot_controller

#endif
