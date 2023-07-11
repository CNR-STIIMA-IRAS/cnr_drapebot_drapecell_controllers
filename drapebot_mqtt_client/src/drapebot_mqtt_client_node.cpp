#include <boost/shared_ptr.hpp>

namespace boost {
#ifdef BOOST_NO_EXCEPTIONS
void throw_exception( std::exception const & e ) { throw 11; };
#endif
}

#include <ros/console.h>
#include <ros/node_handle.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_mqtt_publisher");
    ros::NodeHandle nh;
    ros::Rate r(10); // 10 hz

    // ---- MQTT params ----
    std::string client_id = "egm_joint_state_to_mqtt_controller";
    std::string client_id2 = "egm_joint_state_to_mqtt_controller2";

    //std::string host_str = "192.168.125.9";
    std::string host_str = "127.0.0.1";
    int port = 1883;
    std::string mqtt_command_topic = "/robot_1/command";
    std::string mqtt_feedback_topic = "/robot_1/feedback";
    std::string mqtt_feedback_topic2 = "/robot_2/feedback";
    bool use_json = true;

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);
    cnr::drapebot::MQTTDrapebotClient mqtt_drapebot_client_(client_id.c_str(), host_str.c_str(), port, use_json);
    ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);
    cnr::drapebot::MQTTDrapebotClient mqtt_drapebot_client2_(client_id2.c_str(), host_str.c_str(), port, use_json);
    ROS_INFO_STREAM("Connencted to: "<< client_id << ": " << host_str);

    if (mqtt_drapebot_client_.subscribe(NULL, mqtt_command_topic.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_command_topic );
      return -1;
    }

    if (mqtt_drapebot_client2_.subscribe(NULL, mqtt_command_topic.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_command_topic );
      return -1;
    }

    char data[] = "a";
    int payload_len_ = sizeof(data);
    void* payload_ = malloc(payload_len_);
    memcpy(payload_,data,payload_len_);

    char data2[] = "b";
    int payload_len2_ = sizeof(data2);
    void* payload2_ = malloc(payload_len2_);
    memcpy(payload2_,data2,payload_len2_);

    while (ros::ok())
    {
        int rc = mqtt_drapebot_client_.publish(payload_, payload_len_, mqtt_feedback_topic.c_str() );
        if ( rc != 0)
            ROS_ERROR_STREAM("returned " << rc);

        rc = mqtt_drapebot_client2_.publish(payload2_, payload_len2_, mqtt_feedback_topic2.c_str() );
        if ( rc != 0)
            ROS_ERROR_STREAM("returned " << rc);

        if (mqtt_drapebot_client_.loop() != 0 )
        {
            ROS_ERROR_STREAM("Error on Mosquitto loop function");
            return -1;
        }
        
        if (mqtt_drapebot_client2_.loop() != 0 )
        {
            ROS_ERROR_STREAM("Error on Mosquitto loop function");
            return -1;
        }
        r.sleep();
    }

    free(payload_);

    return 0;
}
