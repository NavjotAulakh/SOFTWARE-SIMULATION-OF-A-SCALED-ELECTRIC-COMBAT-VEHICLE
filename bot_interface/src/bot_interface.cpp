#include <sstream>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

geometry_msgs::Twist latestMsg;
nav_msgs::Odometry odomMsg;
sensor_msgs::JointState statesMsg;
int val = 0;
int val2 = 0;

double px = 0;
double py = 0;
double pz = 0;


double ox = 0;
double oy = 0;
double oz = 0;
double ow = 1;
    

bool called = false;

void botc_callback(const geometry_msgs::Twist& msg)
{
  latestMsg.linear = msg.linear;
  latestMsg.angular = msg.angular;
  called = true;
}

void bots_callback(const nav_msgs::Odometry& msg)
{
  odomMsg = msg;
  val = 1;
}


void jointStatesCallback(const sensor_msgs::JointState& msg)
{
  statesMsg = msg;
  val2 = 1;
}


int main(int argc, char**argv)
{
  ROS_INFO("Starting bot_interface");
  ros::init(argc, argv, "bot_commander");
  ros::NodeHandle n;
  ros::Publisher CMD = n.advertise<geometry_msgs::Twist>("/bot/bot_velocity_controller/cmd_vel", 1);  
  ros::Publisher jointStates = n.advertise<sensor_msgs::JointState>("/bot/bot_velocity_controller/joint_states", 1);  
  ros::Subscriber sub = n.subscribe("/cmd_vel", 50, &botc_callback);
  ros::Subscriber sub2 = n.subscribe("/odom", 50, &bots_callback);
  ros::Subscriber sub3 = n.subscribe("/bot/joint_states", 50, &jointStatesCallback);

  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(20);


while(ros::ok())
  {
  // Broadcast transform from map to base_link

   px = odomMsg.pose.pose.position.x;
   py = odomMsg.pose.pose.position.y;
   pz = odomMsg.pose.pose.position.z;


  if (val == 1)
  {
     ox = odomMsg.pose.pose.orientation.x;
     oy = odomMsg.pose.pose.orientation.y;
     oz = odomMsg.pose.pose.orientation.z;
     ow = odomMsg.pose.pose.orientation.w;
  }
      
    
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
          ros::Time::now(),"map", "odom"));

    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(ox, oy, oz, ow), tf::Vector3(px, py, pz)),
          ros::Time::now(),"map", "base_link"));
  

    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), 
         tf::Vector3(0.0, 0.0, 0.3)), ros::Time::now(),"base_link", "hokuyo_link"));

    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), 
         tf::Vector3(0.0, 0.0, 0.3)), ros::Time::now(),"base_link", "/bot/hokuyo_link"));
  


    if (called)
    {
      CMD.publish(latestMsg);
    }

    if (val2==1)
    {
      jointStates.publish(statesMsg);
    }

    
    ros::spinOnce();

  }
  
}


