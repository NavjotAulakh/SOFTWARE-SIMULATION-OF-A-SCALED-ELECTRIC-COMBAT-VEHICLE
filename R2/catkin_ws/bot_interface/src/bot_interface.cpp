#include <sstream>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

geometry_msgs::Twist latestMsg;
nav_msgs::Odometry odomMsg;

int direction = 0;

void botc_callback(const geometry_msgs::Twist& msg)
{
  latestMsg.linear = msg.linear;
  latestMsg.angular = msg.angular;
}

void bots_callback(const nav_msgs::Odometry& msg)
{
  odomMsg.pose = msg.pose;
}

int main(int argc, char**argv)
{
  ROS_INFO("Starting bot_interface");
  ros::init(argc, argv, "bot_commander");
  ros::NodeHandle n;
  ros::Publisher RW0 = n.advertise<std_msgs::Float64>("/bot/right_wheel_0_joint/command", 50);
  ros::Publisher RW1 = n.advertise<std_msgs::Float64>("/bot/right_wheel_1_joint/command", 50);
  ros::Publisher RW2 = n.advertise<std_msgs::Float64>("/bot/right_wheel_2_joint/command", 50);
  ros::Publisher RW3 = n.advertise<std_msgs::Float64>("/bot/right_wheel_3_joint/command", 50);

  ros::Publisher LW0 = n.advertise<std_msgs::Float64>("/bot/left_wheel_0_joint/command", 50);
  ros::Publisher LW1 = n.advertise<std_msgs::Float64>("/bot/left_wheel_1_joint/command", 50);
  ros::Publisher LW2 = n.advertise<std_msgs::Float64>("/bot/left_wheel_2_joint/command", 50);
  ros::Publisher LW3 = n.advertise<std_msgs::Float64>("/bot/left_wheel_3_joint/command", 50);
  //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber sub = n.subscribe("/cmd_vel", 50, &botc_callback);
  ros::Subscriber sub2 = n.subscribe("/odom", 50, &bots_callback);

  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(50);
/*

  ros::Time current_time, last_time;

  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
*/

  int count = 0;

while(ros::ok())
  {
  // Broadcast transform from map to base_link
  double px = odomMsg.pose.pose.position.x;
  double py = odomMsg.pose.pose.position.y;
  double pz = odomMsg.pose.pose.position.z;
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"map", "odom"));

 broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(px, py, pz)),
        ros::Time::now(),"map", "base_link"));

    count+= 1;
    std_msgs::Float64 left;
    left.data = 0.0;
    std_msgs::Float64 right;
    right.data = 0.0;

    if(latestMsg.linear.x != 0)
    {
      right.data = latestMsg.linear.x;
      left.data = latestMsg.linear.x;
    }

    if(latestMsg.linear.y != 0)
    {
      right.data += latestMsg.linear.y;
      left.data += latestMsg.linear.y;
    }

    if (latestMsg.angular.z > 0 )
    {
      right.data += latestMsg.angular.z;
      left.data -= latestMsg.angular.z;
    }
    else if (latestMsg.angular.z < 0 )
    {
      left.data += latestMsg.angular.z;
      right.data -= latestMsg.angular.z;
    }

    RW0.publish(right);
    RW1.publish(right);
    RW2.publish(right);
    RW3.publish(right);

    LW0.publish(left);
    LW1.publish(left);
    LW2.publish(left);
    LW3.publish(left);

    /*
vx = latestMsg.linear.x;
    vy = latestMsg.linear.y;
    vth = latestMsg.angular.z;
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    x += delta_x/100;
    y += delta_y/100;
    th += delta_th/100;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;

*/
    ros::spinOnce();

  }
  
}

