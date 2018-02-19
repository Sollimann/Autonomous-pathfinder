#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <vector>
#include <math.h>
#include <armadillo>

using namespace arma;

class PathPlanner{
private:
    ros::Subscriber sub_pos;
    ros::Publisher pub_vel;
    ros::Publisher pub_point;
    ros::Publisher pub_point_smooth;
    mat path_points;
    double pos_x, pos_y, ori_z, ang_z, setpoint_x, setpoint_y;
    int current_point;
    double threshold_switch;
public:
    PathPlanner(ros::NodeHandle &nh);
    void callback_odom( const nav_msgs::OdometryConstPtr& poseMsg);

};


PathPlanner::PathPlanner(ros::NodeHandle &nh) {
    // Setting the row of path_points which is the current target setpoint
    current_point = 0;
    threshold_switch = 0.1;

    // Generating a path for the bot to follow. (x,y) coordinates:
    path_points     << 1 << 0 << endr
                    << 1 << 1 << endr
                    << 2 << 1 << endr
                    << 3 << 1 << endr
                    << 3 << 2 << endr
                    << 3 << 3 << endr
                    << 4 << 3 << endr
                    << 4 << 4 << endr;

    setpoint_x = path_points(current_point, 0);
    setpoint_y = path_points(current_point, 1);

    // Defining subscribers and publishers:
    sub_pos = nh.subscribe("/odom",1,&PathPlanner::callback_odom, this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    pub_point = nh.advertise<geometry_msgs::Point>("/pathplanner/setpoint",1);
    pub_point_smooth = nh.advertise<geometry_msgs::Point>("/pathplanner/setpoint_smooth",1);

}


void PathPlanner::callback_odom( const nav_msgs::OdometryConstPtr& poseMsg){

    geometry_msgs::Point cmd_setpoint;
    geometry_msgs::Point cmd_setpoint_smooth;

    pos_x = poseMsg->pose.pose.position.x;
    pos_y = -(poseMsg->pose.pose.position.y);
    ori_z = poseMsg->pose.pose.orientation.z;
    ang_z = ori_z*2.19;

    setpoint_x = path_points(current_point, 0);
    setpoint_y = path_points(current_point, 1);

    double error = (setpoint_x - pos_x) * (setpoint_x - pos_x) + (setpoint_y - pos_y) * (setpoint_y - pos_y);
    std::cout << "Setpoint (x,y) =  (" << setpoint_x << ", " << setpoint_y << ")" << std::endl;
    std::cout << "Position (x,y) =  (" << pos_x << ", " << pos_y << ")" << std::endl;
    std::cout << "Error: " << error << std::endl;


    if (error < threshold_switch){
        // Switch setpoint
        current_point++;
        setpoint_x = path_points(current_point, 0);
        setpoint_y = path_points(current_point, 1);
    }

    cmd_setpoint.x = setpoint_x;
    cmd_setpoint.y = setpoint_y;
    cmd_setpoint.z = 0;

    cmd_setpoint_smooth.x = pos_x + 0.01 * setpoint_x;
    cmd_setpoint_smooth.y = pos_y + 0.01 * setpoint_y;
    cmd_setpoint_smooth.z = 0;
    std::cout << "Smooth setpoint (x,y) =  (" << pos_x + 0.01 * setpoint_x << ", " << pos_y + 0.01 * setpoint_y << ")" << std::endl;

    pub_point.publish(cmd_setpoint);
    pub_point_smooth.publish(cmd_setpoint_smooth);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    PathPlanner pp(nh);

    ros::spin();
    return 0;
}