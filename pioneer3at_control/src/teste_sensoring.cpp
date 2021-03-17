#include <iostream>
#include <cstdio>
#include <chrono>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Vector3.h"

using namespace std;

/* Global variable for desired position and orientation {x, y, theta} */
vector<double> desiredState{10.0, 0.0, 0.0};   // In fact, theta can be neglected. The goal point
                                               // is what matters.

double distBtwPoints(const geometry_msgs::Vector3 state)
{ 
    return sqrt( pow( state.x - desiredState[0], 2) 
                + pow( state.y - desiredState[1], 2) );
}

const geometry_msgs::Vector3 fromQuaternionToRPY(const geometry_msgs::Quaternion msg)
{
    tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
    tf::Matrix3x3 m(q);
    
    geometry_msgs::Vector3 angles;
    
    /* X -> roll, Y -> pitch, Z -> yaw */
    m.getRPY(angles.x, angles.y, angles.z);
    
    std::cout << "Roll: " << angles.x << ", Pitch: " << angles.y << ", Yaw: " << angles.z << std::endl;

    return angles;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Vector3 state;
    
    /*ROS_INFO("\n|**********************|");
    cout << "Got odometry message.\n->\t";
    cout << "Seq: " << msg->header.seq << "\tTime Stamp: " << msg->header.stamp.sec << "," << msg->header.stamp.nsec << " sec\n->\t";
    cout << "ID: " << msg->header.frame_id.c_str() << "\tChild ID: " << msg->child_frame_id.c_str() << "\n";
    cout << "Twist: X = " << msg->twist.twist.linear.x << "\t Y = " << msg->twist.twist.linear.y << "\t W_Z = " << msg->twist.twist.angular.z << "\n"; 
    cout << "|**********************|\n\n\n";*/
    
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;

    printf("X coordinate: %f\n", state.x);
    printf("Y coordinate: %f\n", state.y);
    printf("Distance between points: %f\n", distBtwPoints(state) );

    state.z = ( fromQuaternionToRPY(msg->pose.pose.orientation) ).z;
}

int main(int argc, char** argv)
{   
    getchar();
    cout << "Enter a character to start simulation.\n";
    getchar();

    ros::init(argc, argv, "teste_sensoring");
    ros::NodeHandle teste_sensoring;

    ros::Subscriber odometry_sub = teste_sensoring.subscribe("/pioneer3at/odom", 1000, odometryCallback);
    
    ros::Rate loop_rate(10);
    
    /* Actuation Twist */
    geometry_msgs::Twist actuation;

    getchar();
    
    int count = 0;
    while( ros::ok() )
    {
        ros::spinOnce();
        
        loop_rate.sleep();
        ++count;
    }

	return 0;
}   