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
#include <std_msgs/Float32.h>

using namespace arma;

class PathPlanner{
private:
    ros::Subscriber sub_pos;
    ros::Publisher pub_vel;
    ros::Publisher pub_point;
    ros::Publisher pub_point_smooth;
    ros::Publisher pub_x_y_yaw;
    mat path_points;
    double pos_x, pos_y, ang_z, setpoint_x, setpoint_y;
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
    pub_x_y_yaw = nh.advertise<geometry_msgs::Point>("pathplanner/x_y_yaw",1);

}


void PathPlanner::callback_odom( const nav_msgs::OdometryConstPtr& poseMsg){

    geometry_msgs::Point cmd_setpoint;
    geometry_msgs::Point cmd_setpoint_smooth;
    geometry_msgs::Point cmd_x_y_yaw;

    pos_x = poseMsg->pose.pose.position.x;
    pos_y = -(poseMsg->pose.pose.position.y);

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

    // Calculating orientation [-PI, PI]
    float x = poseMsg->pose.pose.orientation.x;
    float y = poseMsg->pose.pose.orientation.y;
    float z = poseMsg->pose.pose.orientation.z;
    float w = poseMsg->pose.pose.orientation.w;


    ang_z = -atan2((2.0 * (w*z + x*y)), (1.0 - 2.0 * (y*y + z*z)));
    std::cout << "Angle: " << ang_z << std::endl;

    cmd_x_y_yaw.x = pos_x;
    cmd_x_y_yaw.y = pos_y;
    cmd_x_y_yaw.z = ang_z;
    pub_x_y_yaw.publish(cmd_x_y_yaw);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    PathPlanner pp(nh);

    ros::spin();
    return 0;
}