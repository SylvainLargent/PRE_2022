#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "rrt_lib/Point3.hpp"
#include "rrt_lib/Segment.hpp"
#include "rrt_lib/Env.hpp"
#include "rrt_lib/command.hpp"
#include "rrt_lib/Node.hpp"
#include "rrt_lib/RRT_STAR.hpp"
#include "rrt_lib/RRT.hpp"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>



geometry_msgs::PoseStamped coord;

void callBackPosition(const geometry_msgs::PoseStamped msg) {
    coord.pose.position.x = msg.pose.position.x;
    coord.pose.position.y = msg.pose.position.y;
    coord.pose.position.z = msg.pose.position.z;
    // ROS_INFO("Current position: (%f, %f, %f)", coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "capture_position");
    ros::NodeHandle nh_pos;

    ros::Subscriber pos_sub;
    pos_sub = nh_pos.subscribe<geometry_msgs::PoseStamped>("/mocap_node/Robot_1/pose",1000, callBackPosition);

    ros::Rate loop_rate(5);
    Point3 position;
//Initializing the position of optitrack 
    while(coord.pose.position.x == 0){
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ofstream position_stream;
    position_stream.open("/home/student/Desktop/Sylvain/3D_rrt_plot_cpp/csv_files/position.txt");

    while (ros::ok()) {
            position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
            position_stream << coord.pose.position.x << "," << coord.pose.position.y << "," << coord.pose.position.z << std::endl;
            ros::spinOnce();
            loop_rate.sleep();
    }
    
    position_stream.close();
}