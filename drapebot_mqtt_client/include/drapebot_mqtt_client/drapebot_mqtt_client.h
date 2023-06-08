
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


#ifndef __DRAPEBOT_MQTT_CLIENT__
#define __DRAPEBOT_MQTT_CLIENT__

#include <mutex>
#include <ctime>
#include <chrono>
#include <fstream>

#include <cnr_mqtt_client/cnr_mqtt_client.h>

#define MAX_PAYLOAD_SIZE 1024
#define MSG_AXES_LENGTH 7 // The length is given by 6 axes robot + linear axis
#define DEFAULT_KEEP_ALIVE 60

namespace cnr
{
  namespace drapebot
  {

    // void tic(int mode=0);
    // void toc();

    struct drapebot_msg 
    {
      double joints_values_[MSG_AXES_LENGTH] = {0};    
      unsigned long long int counter_ = 0;
    }; 


    class DrapebotMsgDecoder: public cnr::mqtt::MsgDecoder
    {
    public:
      DrapebotMsgDecoder(cnr::drapebot::drapebot_msg* mqtt_msg): mqtt_msg_(mqtt_msg), first_message_rec_(false) {};
      
      // The method should be reimplemented on the base of the application
      void on_message(const struct mosquitto_message *msg) override;
      bool isFirstMsgRec(){return first_message_rec_;};

    private:
      cnr::drapebot::drapebot_msg* mqtt_msg_;
      bool first_message_rec_;
    };

    class DrapebotMsgEncoder: public cnr::mqtt::MsgEncoder
    {
    public:
      DrapebotMsgEncoder(cnr::drapebot::drapebot_msg* mqtt_msg): mqtt_msg_(mqtt_msg) {};
      
      // The method should be reimplemented on the base of the application
      void on_publish(int mid) override;

    private:
      cnr::drapebot::drapebot_msg* mqtt_msg_;
    };

    class MQTTDrapebotClient
    {
    public:
      MQTTDrapebotClient (const char *id, const char *host, int port, int keepalive = 60);
      ~MQTTDrapebotClient();

      int stop();
      int loop(int timeout=2000);
      int reconnect();  
      int subscribe(int *mid, const char *sub, int qos);
      int unsubscribe(int *mid, const char *sub);
      int publish(const void* payload, int& payload_len, const char* topic_name);
     
      bool isFirstMsgRec(){ return drapebot_msg_decoder_->isFirstMsgRec(); };
      bool getLastReceivedMessage(cnr::drapebot::drapebot_msg& last_msg);  
    
    private:
      cnr::drapebot::DrapebotMsgDecoder* drapebot_msg_decoder_;
      cnr::drapebot::DrapebotMsgEncoder* drapebot_msg_encoder_;

      cnr::drapebot::drapebot_msg* mqtt_msg_enc_;
      cnr::drapebot::drapebot_msg* mqtt_msg_dec_;            

      cnr::mqtt::MQTTClient* mqtt_client_;

    };
  }

}
#endif
