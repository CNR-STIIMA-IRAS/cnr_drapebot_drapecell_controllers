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

// #ifdef WIN32
//   #include <json.h>
// #else
//   #include <jsoncpp/json/json.h>
// #endif

#include <chrono>
#include <algorithm>
#include <cstddef>
#include <pluginlib/class_list_macros.hpp>
#include <joint_state_controller/joint_state_controller.h>

#include <drapebot_mqtt_client/json.hpp>
#include <egm_joint_state_to_mqtt_controller/egm_joint_state_to_mqtt_controller.h>


namespace drapebot_controller
{
  EgmJointStateToMQTTController::EgmJointStateToMQTTController() :  publish_rate_(0.0),
                                                                    counter_(0) 
  {

  }


  EgmJointStateToMQTTController::~EgmJointStateToMQTTController() 
  {
    delete mqtt_drapebot_client_;
  }


  bool EgmJointStateToMQTTController::init( hardware_interface::JointStateInterface* hw,
                                            ros::NodeHandle&                         root_nh,
                                            ros::NodeHandle&                         controller_nh)
  { 
    try
    {
      ROS_WARN_STREAM("EgmJointStateToMQTTController::init");
      // List of joints to be published
      std::vector<std::string> joint_names;

      // Get list of joints: This allows specifying a desired order, or
      // alternatively, only publish states for a subset of joints. If the
      // parameter is not set, all joint states will be published in the order
      // specified by the hardware interface.
      if (controller_nh.getParam("joints", joint_names)) 
      {
        ROS_INFO_STREAM("Joints parameter specified, publishing specified joints in desired order.");
      } 
      else 
      {
        // get all joint names from the hardware interface
        joint_names = hw->getNames();
      }

      num_hw_joints_ = joint_names.size();
      for (unsigned i=0; i<num_hw_joints_; i++)
        ROS_DEBUG("Got joint %s", joint_names[i].c_str());

      // get publishing period
      if (!controller_nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
      }

      // realtime publisher
      realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

      // get joints and allocate message
      for (unsigned i=0; i<num_hw_joints_; i++)
      {
        joint_state_.push_back(hw->getHandle(joint_names[i]));
        realtime_pub_->msg_.name.push_back(joint_names[i]);
        realtime_pub_->msg_.position.push_back(0.0);
        realtime_pub_->msg_.velocity.push_back(0.0);
        realtime_pub_->msg_.effort.push_back(0.0);
      }

      addExtraJoints(controller_nh, realtime_pub_->msg_);
    
      // ---- MQTT params ----
      std::string client_id;
      if (!controller_nh.getParam("client_id",client_id))
      {
        client_id = "Client_ID";
        ROS_WARN_STREAM("client id not found under " + controller_nh.getNamespace() + "/client_id . Using defalut client ID: " + client_id);
      }

      std::string host_str;
      if (!controller_nh.getParam("broker_address",host_str))
      {
        host_str = "localhost";
        ROS_WARN_STREAM("broker_address not found under " + controller_nh.getNamespace() + "/broker_address . Using defalut broker address: "+ host_str);
      }

      int port;
      if (!controller_nh.getParam("port",port))
      {
        port = 1883;
        ROS_WARN_STREAM("port not found under " + controller_nh.getNamespace() + "/port. Using defalut broker address: "+ std::to_string( port));      
      }    

      if (!controller_nh.getParam("use_json",use_json_))
      {
        use_json_ = true;
        ROS_WARN_STREAM("use json flag not found " + controller_nh.getNamespace() + "/use_json. Using defalut json flag: true " );      
      }   

      ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);
      mqtt_drapebot_client_ = new cnr::drapebot::MQTTDrapebotClient(client_id.c_str(), host_str.c_str(), port, use_json_);
      ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);
      
      if (!controller_nh.getParam("mqtt_feedback_topic", mqtt_feedback_topic_))
      {
        mqtt_feedback_topic_ = "mqtt_feedback_topic";
        ROS_WARN_STREAM("mqtt_feedback_topic not found under " + controller_nh.getNamespace() + "/mqtt_feedback_topic . Using defalut broker address: "+ mqtt_feedback_topic_);  
      }

    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM( "Thrown exception: " << e.what() );
      return false;
    }

    ROS_INFO_STREAM("Controller EgmJointStateToMQTTController Initialized ! ");

    return true;
  }

  void EgmJointStateToMQTTController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
    ROS_INFO_STREAM("Starting controller: EgmJointStateToMQTTController " );
  }

  void EgmJointStateToMQTTController::stopping(const ros::Time& /*time*/)
  {
    ROS_INFO_STREAM("Stopping controller: EgmJointStateToMQTTController " );
  }

  void EgmJointStateToMQTTController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
    {
      // try to publish
      if (realtime_pub_->trylock())
      {
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // populate joint state message:
        // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
        // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
        realtime_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<num_hw_joints_; i++)
        {
          realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
          realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
          realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
        }
        realtime_pub_->unlockAndPublish();
      }
    }
    
    cnr::drapebot::drapebot_msg j_pos_feedback;

    for (unsigned i=0; i<num_hw_joints_; i++)
      j_pos_feedback.joints_values_[i] = joint_state_[i].getPosition();

    j_pos_feedback.counter_ = counter_;

    int rc;

    if (use_json_)
    {
      nlohmann::json data;
      
      data["J0"] =  j_pos_feedback.joints_values_[0];
      data["J1"] =  j_pos_feedback.joints_values_[1];
      data["J2"] =  j_pos_feedback.joints_values_[2];
      data["J3"] =  j_pos_feedback.joints_values_[3];
      data["J4"] =  j_pos_feedback.joints_values_[4];
      data["J5"] =  j_pos_feedback.joints_values_[5];
      data["E0"] =  j_pos_feedback.joints_values_[6];
      data["count"] = j_pos_feedback.counter_;
      
      const std::string json_file = data.dump();

      int payload_len = json_file.length() + 1;
      char* payload = new char[ payload_len ];
      strcpy(payload, json_file.c_str());

      rc = mqtt_drapebot_client_->publish(payload, payload_len, mqtt_feedback_topic_.c_str());
      delete payload;

    }
    else
    {
      int payload_len = sizeof(j_pos_feedback);
      void* payload = malloc( payload_len );
      memcpy(payload, &j_pos_feedback, payload_len);  
      rc = mqtt_drapebot_client_->publish(payload, payload_len, mqtt_feedback_topic_.c_str());
      delete payload;
    }

    
    if ( rc != 0)
      ROS_ERROR_STREAM("MQTT publish function returned: " << rc);

    counter_++;
    
  }

  void EgmJointStateToMQTTController::addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg)
  {
    // Preconditions
    XmlRpc::XmlRpcValue list;
    if (!nh.getParam("extra_joints", list))
    {
      ROS_DEBUG("No extra joints specification found.");
      return;
    }

    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Extra joints specification is not an array. Ignoring.");
      return;
    }

    for(std::size_t i = 0; i < list.size(); ++i)
    {
      XmlRpc::XmlRpcValue& elem = list[i];

      if (elem.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << elem.getType() <<
                         "'. Ignoring.");
        continue;
      }

      if (!elem.hasMember("name"))
      {
        ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
        continue;
      }

      const std::string name = elem["name"];
      if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
      {
        ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
        continue;
      }

      const bool has_pos = elem.hasMember("position");
      const bool has_vel = elem.hasMember("velocity");
      const bool has_eff = elem.hasMember("effort");

      const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
      if (has_pos && elem["position"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default position. Ignoring.");
        continue;
      }
      if (has_vel && elem["velocity"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default velocity. Ignoring.");
        continue;
      }
      if (has_eff && elem["effort"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default effort. Ignoring.");
        continue;
      }

      // State of extra joint
      const double pos = has_pos ? static_cast<double>(elem["position"]) : 0.0;
      const double vel = has_vel ? static_cast<double>(elem["velocity"]) : 0.0;
      const double eff = has_eff ? static_cast<double>(elem["effort"])   : 0.0;

      // Add extra joints to message
      msg.name.push_back(name);
      msg.position.push_back(pos);
      msg.velocity.push_back(vel);
      msg.effort.push_back(eff);
    }
  }

}

PLUGINLIB_EXPORT_CLASS( drapebot_controller::EgmJointStateToMQTTController, controller_interface::ControllerBase)
