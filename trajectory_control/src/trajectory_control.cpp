#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
int main(int argc,char *argv[])
{
    //pub msgs to control the turtle
    //topic:/turtle1/cmd_vel
    //type:geometry_msgs/Twist
    //step:include the head file
    //initialize  ros node
    //create handle of ros
    //create pub
    ros::init(argc,argv,"traj_control"); //initialize node
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::Rate rate(10);
    geometry_msgs::Twist twist;
    twist.linear.x = 1.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    while(ros::ok())
    {
        pub.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

