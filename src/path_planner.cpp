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

//namespace MapEnums
//{
    enum {UNKNOWN = 0, OPEN = 1, WALL = 2};
//}
//typedef MapEnums::MapEnum MapEnum;

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
    cube wall_map;
    mat flood_fill_map;
public:
    PathPlanner(ros::NodeHandle &nh);
    void callback_odom( const nav_msgs::OdometryConstPtr& poseMsg);
    void initializeWallMap();
    void printWallMap();
    void initializeFloodFillMap();
    void set_flood_fill_map_value(int x_pos, int y_pos, int value);
    int get_flood_fill_map_value(int x_pos, int y_pos);

};


PathPlanner::PathPlanner(ros::NodeHandle &nh){
    // Initializing wall map:
    initializeWallMap();
    initializeFloodFillMap();

    // Setting the row of path_points which is the current target setpoint
    current_point = 0;
    threshold_switch = 0.01;

    // Generating a path for the bot to follow. (x,y) coordinates:
    path_points     << 0 << 1 << endr
                    << 1 << 1 << endr
                    << 1 << 2 << endr
                    << 1 << 3 << endr
                    << 2 << 3 << endr
                    << 3 << 3 << endr
                    << 3 << 4 << endr
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

    // Redefining coordinate system from
    //          x       y
    //          |  to   |
    //          |       |
    // y <-------       -------> x

    pos_y = poseMsg->pose.pose.position.x;
    pos_x = -(poseMsg->pose.pose.position.y);

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

    cmd_setpoint_smooth.x = setpoint_x; //0.7 * pos_x + 0.3 * setpoint_x;
    cmd_setpoint_smooth.y = setpoint_y; //0.7 * pos_y + 0.3 * setpoint_y;
    cmd_setpoint_smooth.z = 0;
    std::cout << "Smooth setpoint (x,y) =  (" << pos_x + 0.01 * setpoint_x << ", " << pos_y + 0.01 * setpoint_y << ")" << std::endl;

    pub_point.publish(cmd_setpoint);
    pub_point_smooth.publish(cmd_setpoint_smooth);

    // Calculating orientation [-PI, PI]
    float x = poseMsg->pose.pose.orientation.x;
    float y = poseMsg->pose.pose.orientation.y;
    float z = poseMsg->pose.pose.orientation.z;
    float w = poseMsg->pose.pose.orientation.w;

    // Converting quaterninon to yaw angle:
    ang_z = -atan2((2.0 * (w*z + x*y)), (1.0 - 2.0 * (y*y + z*z)));
    std::cout << "Angle: " << ang_z << std::endl;

    cmd_x_y_yaw.x = pos_x;
    cmd_x_y_yaw.y = pos_y;
    cmd_x_y_yaw.z = ang_z;
    pub_x_y_yaw.publish(cmd_x_y_yaw);

}


void PathPlanner::initializeWallMap(){

    // 9x9 matrix containing information if there is a wall or not in direction
    // North, East, South and/or West for the point.
    // Each point has a vector of 4 elements indicating whether or not a wall is nearby, and in which direction.
    // [N, E, S, W] could either be UNKNOWN, OPEN or WALL
    wall_map = cube(9,9,4, fill::zeros);

    // Fill the information which is already known into the map, i.e outer boundaries has a wall.
    // (row,col)=(0,0) is bottom left
    // (row,col)=(0,8) is bottom right
    // (row,col)=(8,8) is top right
    // (row,col)=(8,0) is top left

    for (int row = 0; row < 9; row++){
        for (int col = 0; col < 9; col++){
            if (row == 0){
                wall_map(row, col, 2) = WALL; // wall to the south
            }
            if (row == 8){
                wall_map(row, col, 0) = WALL; // wall to the north
            }
            if (col == 0){
                wall_map(row, col, 3) = WALL; // wall to the west
            }
            if (col == 8){
                wall_map(row, col, 1) = WALL; // wall to the east
            }
        }
    }
    std::cout << "Wall map initialized!" << std::endl;

    printWallMap();
}


void PathPlanner::printWallMap(){
    // Printing the map to the terminal
    std::cout << "South wall map: " << std::endl;
    for (int row = 8; row >= 0; row--){
        std::cout << std::endl;
        for (int col = 0; col < 9; col++){
            std::cout << wall_map(row, col, 2) << " ";
        }
    }
    std::cout  << "\n" << std::endl;

    std::cout << "East wall map: " << std::endl;
    for (int row = 8; row >= 0; row--){
        std::cout << std::endl;
        for (int col = 0; col < 9; col++){
            std::cout << wall_map(row, col, 1) << " ";
        }
    }
    std::cout  << "\n" << std::endl;

    std::cout << "North wall map: " << std::endl;
    for (int row = 8; row >= 0; row--){
        std::cout << std::endl;
        for (int col = 0; col < 9; col++){
            std::cout << wall_map(row, col, 0) << " ";
        }
    }
    std::cout  << "\n" << std::endl;

    std::cout << "West wall map: " << std::endl;
    for (int row = 8; row >= 0; row--){
        std::cout << std::endl;
        for (int col = 0; col < 9; col++){
            std::cout << wall_map(row, col, 3) << " ";
        }
    }
    std::cout  << "\n" << std::endl;
}


void PathPlanner::initializeFloodFillMap(){
    flood_fill_map  << 8 << 7 << 6 << 5 << 4 << 5 << 6 << 7 << 8 << endr
                    << 7 << 6 << 5 << 4 << 3 << 4 << 5 << 6 << 7 << endr
                    << 6 << 5 << 4 << 3 << 2 << 3 << 4 << 5 << 6 << endr
                    << 5 << 4 << 3 << 2 << 1 << 2 << 3 << 4 << 5 << endr
                    << 4 << 3 << 2 << 1 << 0 << 1 << 2 << 3 << 4 << endr
                    << 5 << 4 << 3 << 2 << 1 << 2 << 3 << 4 << 5 << endr
                    << 6 << 5 << 4 << 3 << 2 << 3 << 4 << 5 << 6 << endr
                    << 7 << 6 << 5 << 4 << 3 << 4 << 5 << 6 << 7 << endr
                    << 8 << 7 << 6 << 5 << 4 << 5 << 6 << 7 << 8 << endr;

    std::cout << "Flood fill map initialized!" << std::endl;
    std::cout <<  " Printed: " << std::endl;
    flood_fill_map.print();

    //set_flood_fill_map_value(3,5,10);
    //flood_fill_map(5,3) = 10;
    //flood_fill_map.print();
    //std::cout << "get_flood_fill_map_value(3,5) = " << get_flood_fill_map_value(3,5) << std::endl;
}


void PathPlanner::set_flood_fill_map_value(int x_pos, int y_pos, int value){
    flood_fill_map(8 - y_pos, x_pos) = value;
}

int PathPlanner::get_flood_fill_map_value(int x_pos, int y_pos){
    return flood_fill_map(8 - y_pos, x_pos);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    PathPlanner pp(nh);

    ros::spin();
    return 0;
}