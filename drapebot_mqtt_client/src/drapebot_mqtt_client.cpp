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

#ifdef WIN32
  #include <json.h>
#else
  #include <jsoncpp/json/json.h>
#endif

#include <ros/ros.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>

namespace  cnr
{
  namespace drapebot
  {
    void tic(int mode) 
    {
      static std::chrono::high_resolution_clock::time_point t_start;
    
      if (mode==0)
          t_start = std::chrono::high_resolution_clock::now();
      else 
      {
        auto t_end = std::chrono::high_resolution_clock::now();
        ROS_WARN_STREAM_THROTTLE(1.0, "Elapsed time is " << (t_end-t_start).count()*1E-9 << "  seconds" );
      }
    }

    void toc() 
    { 
      tic(1); 
    }

    void DrapebotMsgDecoder::on_message(const struct mosquitto_message *msg)
    {
      char* buffer = new char[msg->payloadlen];
      memcpy(buffer, msg->payload, msg->payloadlen);
    
      Json::Reader reader;
      Json::Value root;
    
      reader.parse(buffer,root);
      
      if (mtx_.try_lock_for(std::chrono::milliseconds(4))) //try to lock mutex for 4ms
      {
        mqtt_msg_->joints_values_[0] = root["J0"].asDouble();
        mqtt_msg_->joints_values_[1] = root["J1"].asDouble();
        mqtt_msg_->joints_values_[2] = root["J2"].asDouble();
        mqtt_msg_->joints_values_[3] = root["J3"].asDouble();
        mqtt_msg_->joints_values_[4] = root["J4"].asDouble();
        mqtt_msg_->joints_values_[5] = root["J5"].asDouble();
        mqtt_msg_->joints_values_[6] = root["E0"].asDouble();
        mqtt_msg_->counter_ = root["count"].asInt();
      
        //setNewMessageAvailable(true);
        //setDataValid(true);   // Should be checked the length of the received data, but the length is not constant

        mtx_.unlock();

        if (!first_message_rec_)
          first_message_rec_ = true;
      }
      else
      {
        ROS_WARN_STREAM("Can't lock mutex in DrapebotMsgDecoder::on_message timeout reached, last MQTT message not recovered." );
      }
    
      delete buffer;
    }
    
    void DrapebotMsgEncoder::on_publish(int mid)
    {
      // Nothing to do here
    }

    MQTTDrapebotClient::MQTTDrapebotClient(const char *id, const char *host, int port, int keepalive)
    {
      try
      {
        mqtt_msg_enc_ = new cnr::drapebot::drapebot_msg;
        mqtt_msg_dec_ = new cnr::drapebot::drapebot_msg;
        
        drapebot_msg_encoder_ = new cnr::drapebot::DrapebotMsgEncoder(mqtt_msg_enc_);
        drapebot_msg_decoder_ = new cnr::drapebot::DrapebotMsgDecoder(mqtt_msg_dec_);

        mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, drapebot_msg_encoder_, drapebot_msg_decoder_);
      }
      catch(const std::exception& e)
      {
        ROS_ERROR_STREAM("Exception thrown in MQTTDrapebotClient constructor: " <<  e.what() );
      }
    }

    MQTTDrapebotClient::~MQTTDrapebotClient()
    {  
      delete mqtt_msg_dec_;
      delete mqtt_msg_enc_;
      delete drapebot_msg_decoder_;
      delete drapebot_msg_encoder_;
      delete mqtt_client_;
    }

    int MQTTDrapebotClient::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      

      return -1;
    }

    int MQTTDrapebotClient::loop(int timeout)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop(timeout);
      
      return -1;
    }

    int MQTTDrapebotClient::reconnect()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->reconnect();
      
      return -1;
    }

    int MQTTDrapebotClient::subscribe(int *mid, const char *sub, int qos)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->subscribe(mid, sub, qos);
      
      return -1;
    }
    
    int MQTTDrapebotClient::unsubscribe(int *mid, const char *sub)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->unsubscribe(mid, sub);
      
      return -1;
    }

    int MQTTDrapebotClient::publish(const void* payload, int& payload_len, const char* topic_name)
    {        
      if (mqtt_client_ != NULL)
        return mqtt_client_->publish(payload, payload_len, topic_name);
      
      return -1;
    }

    bool MQTTDrapebotClient::getLastReceivedMessage(cnr::drapebot::drapebot_msg& last_msg)
    {
      if (drapebot_msg_decoder_ != NULL)
      {
        if (!drapebot_msg_decoder_->isFirstMsgRec())
        {
          ROS_WARN_THROTTLE(2.0,"First message not received yet." );
          return false;
        }

        //if (drapebot_msg_decoder_->isNewMessageAvailable() && drapebot_msg_decoder_->isDataValid() )
        //{
          if (drapebot_msg_decoder_->mtx_.try_lock_for(std::chrono::milliseconds(4))) //try to lock mutex for 4ms
          {
            for (size_t id=0; id<MSG_AXES_LENGTH; id++)
              last_msg.joints_values_[id] = mqtt_msg_dec_->joints_values_[id];

            last_msg.counter_ = mqtt_msg_dec_->counter_;

            //drapebot_msg_decoder_->setNewMessageAvailable(false);

            drapebot_msg_decoder_->mtx_.unlock();

            return true;
          }
          else
          {
             ROS_WARN_STREAM("Can't lock mutex MQTTDrapebotClient::getLastReceivedMessage. Last message received from MQTT not recovered." );
             return false;
          }          
        //}
      
      }

      return false;
    }
  }
}

