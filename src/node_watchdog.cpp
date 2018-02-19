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

class Watchdog{
private:
    ros::NodeHandle nh;
    //ros::Subscriber sub_pos;
    ros::Subscriber sub_vel;
    ros::Publisher pub_vel;
    double max_trans_vel; // Max translational velocity [m/s]
    double max_rot_vel; // Max rotational velocity [deg/s]

public:
    Watchdog();
    void velocityWatchdog(const geometry_msgs::Twist& velMsg);
};


Watchdog::Watchdog(){
    // Defining subscribers and publishers:
    sub_vel = nh.subscribe("/mobile_base/commands/velocity",1,&Watchdog::velocityWatchdog, this);
    //sub_pos = nh.subscribe("/odom",1,&PathPlanner::callback_odom, this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    max_trans_vel = 0.7; // Max translational velocity [m/s]
    max_rot_vel = 180.0 * (3.14159265 / 180.0); // Max rotational velocity [rad/s]

}


void Watchdog::velocityWatchdog(const geometry_msgs::Twist& velMsg){
    geometry_msgs::Twist cmd_vel;
    cmd_vel = velMsg;

    if (velMsg.linear.x > max_trans_vel){
        cmd_vel.linear.x = max_trans_vel;
        std::cout << "Linear x velocity exceeds limit" << std::endl;
    }
    if (velMsg.linear.x < -max_trans_vel){
        cmd_vel.linear.x = -max_trans_vel;
        std::cout << "Linear x velocity exceeds limit" << std::endl;
    }
    if (velMsg.angular.z > max_rot_vel){
        cmd_vel.angular.z = max_rot_vel;
        std::cout << "Angular z velocity exceeds limit" << std::endl;
    }
    if (velMsg.angular.z < -max_rot_vel){
        cmd_vel.angular.z = -max_rot_vel;
        std::cout << "Angular z velocity exceeds limit" << std::endl;
    }
    pub_vel.publish(cmd_vel);

}




int main(int argc, char** argv) {
    ros::init(argc, argv, "node_watchdog");
    Watchdog node;
    while (ros::ok()){
        ros::spin();
    }
    return 0;
}