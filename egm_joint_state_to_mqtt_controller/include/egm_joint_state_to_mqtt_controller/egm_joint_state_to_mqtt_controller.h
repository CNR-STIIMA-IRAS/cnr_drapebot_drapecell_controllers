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

#ifndef __EGM_JOINT_STATE_TO_MQTT_CONTROLLER_H__
#define __EGM_JOINT_STATE_TO_MQTT_CONTROLLER_H__

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>

namespace drapebot_controller
{
  class EgmJointStateToMQTTController: public controller_interface::Controller<hardware_interface::JointStateInterface>
  {
  public:
    EgmJointStateToMQTTController();
    ~EgmJointStateToMQTTController();

    virtual bool init(hardware_interface::JointStateInterface* hw,
                      ros::NodeHandle&                         root_nh,
                      ros::NodeHandle&                         controller_nh);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
    virtual void stopping(const ros::Time& /*time*/);

  private:
    std::vector<hardware_interface::JointStateHandle> joint_state_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
    ros::Time last_publish_time_;
    double publish_rate_;
    size_t num_hw_joints_; ///< Number of joints present in the JointStateInterface, excluding extra joints
    unsigned long int counter_; //number of packages sent through MQTT
    
    cnr::drapebot::MQTTDrapebotClient* mqtt_drapebot_client_;
    std::string mqtt_feedback_topic_;

    void addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg);
  };

} //end drapebot_controller

#endif