#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

#include <drapebot_mqtt_client/drapebot_mqtt_client.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"mqtt_to_ros_abb_command_repeater");
    ros::NodeHandle nh;
    ros::Rate r(250); // 250 hz
    
    bool use_ros_command;
    if (!nh.getParam("/egm/mqtt_to_joint_position_controller/use_ros_command",use_ros_command))
    {
      use_ros_command = true;
      ROS_WARN_STREAM("use_ros_command not found under " + nh.getNamespace() + "/use_ros_command . Using defalut use_ros_command: TRUE");
    }

    if (!use_ros_command)
    {
      ROS_WARN_STREAM("The use of MQTT TO ROS repeater is disabled, this node is not necessary.");
      return 0;
    } 

    std::string ros_command_topic;
    if (!nh.getParam("/egm/mqtt_to_joint_position_controller/ros_command_topic",ros_command_topic))
    {
      ros_command_topic = "/egm/mqtt_to_ros_command/joint_states";
      ROS_WARN_STREAM("ROS ABB command topic " + nh.getNamespace() + "/mqtt_to_joint_position_controller/ros_command_topic . Using default topic name: " + ros_command_topic);
    }


    ros::Publisher jointStatePublisher = nh.advertise<sensor_msgs::JointState>(ros_command_topic, 1);

    //---- MQTT params ----
    std::string client_id = "mqtt_robot_commad_repeater_node";

    std::string host_str;
    if (!nh.getParam("/egm/mqtt_to_joint_position_controller/broker_address",host_str))
    {
      host_str = "localhost";
      ROS_WARN_STREAM("broker_address not found under " + nh.getNamespace() + "mqtt_to_joint_position_controller/broker_address . Using defalut broker address: "+ host_str);
    }

    int port;
    if (!nh.getParam("/egm/mqtt_to_joint_position_controller/port",port))
    {
      port = 1883;
      ROS_WARN_STREAM("port not found under " + nh.getNamespace() + "mqtt_to_joint_position_controller/port. Using defalut port: "+ std::to_string( port));      
    }    

    bool use_json;
    if (!nh.getParam("/egm/mqtt_to_joint_position_controller/use_json",use_json))
    {
      use_json = false;
      ROS_WARN_STREAM("use json flag not found " + nh.getNamespace() + "mqtt_to_joint_position_controller/use_json. Using defalut json flag: true " );      
    }   

    ROS_INFO_STREAM("Connencting mqtt: "<< client_id << ", host: " << host_str << ", port: " << port);

    
    cnr::drapebot::MQTTDrapebotClient mqtt_drapebot_client(client_id.c_str(), host_str.c_str(), port, use_json);
    
    ROS_INFO_STREAM("Connencted to MQTT client name: "<< client_id << " address: " << host_str);
    
    std::string mqtt_command_topic;
    if (!nh.getParam("/egm/mqtt_to_joint_position_controller/mqtt_command_topic", mqtt_command_topic))
    {
      mqtt_command_topic = "/robot_1/command";
      ROS_WARN_STREAM("mqtt_command_topic not found under " + nh.getNamespace() + "/mqtt_to_joint_position_controller/mqtt_command_topic . Using defalut mqtt command topic name: "+ mqtt_command_topic);  
    }

    ROS_INFO_STREAM("Subscribing to: " << mqtt_command_topic);
    if (mqtt_drapebot_client.subscribe(NULL, mqtt_command_topic.c_str(), 1) != 0)
    {
      ROS_ERROR_STREAM("Error on Mosquitto subscribe topic: " << mqtt_command_topic );
      return -1;
    }


    std::vector<std::string> joint_names;
    if(!nh.getParam("/egm/mqtt_to_joint_position_controller/joints",joint_names))
    {
      ROS_WARN_STREAM("joint_names: " << nh.getNamespace() );
    }

    if (joint_names.size()==0)
    {
      ROS_WARN_STREAM("Empty joint_names, retun -1.");
      return -1;
    }

    sensor_msgs::JointState jointState;
   
    for( std::string& name : joint_names )
      jointState.name.push_back(name);
 
    jointState.position.resize(jointState.name.size());
    jointState.velocity.resize(jointState.name.size());
    jointState.effort.resize(jointState.name.size());


    cnr::drapebot::drapebot_msg last_msg;

    while(ros::ok())
    {
      if (mqtt_drapebot_client.loop(4) != 0 )
      {
        ROS_ERROR_STREAM("Error on Mosquitto loop function");
        return -1;
      }

      if (!mqtt_drapebot_client.isFirstMsgRec())
        ROS_WARN_THROTTLE(5.0,"First command message not received yet from microinterpolator." );
      else
      {
        mqtt_drapebot_client.getLastReceivedMessage(last_msg);      

        for (size_t i=0; i<(MSG_AXES_LENGTH-1); i++)
          jointState.position.at(i) = last_msg.joints_values_[i]; 

        jointStatePublisher.publish(jointState);        
      }
    }

    return 0;

}