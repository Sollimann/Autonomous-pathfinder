#include "ros/ros.h"
#include <geometry_msgs/Twist.h> //velocities
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/tfMessage.h> //transforms
#include <tf/transform_datatypes.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <vector>
#include <math.h>

using namespace std_msgs;

class Control{
private:
    ros::Publisher pub_control_speed;
    ros::Subscriber sub_setpoint;
    ros::Subscriber sub_position;

    double trans_x, trans_heading, trans_x_head_pri, I_heading, I_pos, dt;
    double pos_x, pos_y, heading, setpoint_x, setpoint_y, setpoint_heading;
    double error_x,error_y, error_heading, error_pos,error_pos_prev;
    double PI, Kp_fwd, Kp_ang, Ki_ang, Ki_fwd, Kd_fwd;
    int SWITCH, count;
    double RAMP;
public:
    Control(ros::NodeHandle nh);

    //callbacks
    void get_setpoint(const geometry_msgs::Point& setpointMsg);
    void get_position(const geometry_msgs::Point& posMsg);

    //Functions
    void calculate_speed();
    void spin();
};

//Constructor
Control::Control(ros::NodeHandle nh) {

    //Initialize
    pos_x = 0;
    pos_y = 0;
    heading = 0;
    setpoint_x = 0;
    setpoint_y = 0;
    setpoint_heading = 0;
    error_x = 0;
    error_y = 0;
    error_heading = 0;
    error_pos = 0;
    error_pos_prev = 0;
    PI = 3.14159265359;
    I_heading = 0;
    I_pos = 0;
    dt = 0.1; //10Hz
    Kp_ang = 0.35; //Heading gain     //0.3 - 0.35
    Ki_ang = 0.05;                   //0.05 - 0.05
    Kp_fwd = 0.1; //Forward gain       //0.1 - 0.1
    Ki_fwd = 0.003;                    //0.0 - 0.001
    Kd_fwd = 0.05;                    //0.05 - 0.05

    // Callback updates set point position in x and y
    sub_position = nh.subscribe("pathplanner/x_y_yaw",1,&Control::get_position,this);
    sub_setpoint = nh.subscribe("/pathplanner/setpoint",1,&Control::get_setpoint, this);

    pub_control_speed = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

//Subscribes to current position
void Control::get_position(const geometry_msgs::Point& posMsg) {

    pos_x = posMsg.x;
    pos_y = posMsg.y;
    heading = posMsg.z;

}

//This function will subscribe to setpoint messages from
//the pathplanner node.
void Control::get_setpoint(const geometry_msgs::Point& setpointMsg) {

    //Position from pathplanner node
    setpoint_x = setpointMsg.x;
    setpoint_y = setpointMsg.y;
    setpoint_heading = atan2(error_x, error_y);

    //Calculate setpoint heading
    error_x = (setpoint_x - pos_x);
    error_y = (setpoint_y - pos_y);
    error_pos_prev = error_pos;
    error_pos = sqrt(error_x * error_x + error_y * error_y);
    error_heading = setpoint_heading - heading;

    // Yaw angle is defined as [0, 2pi], so the residuals can not be less than or grater than +-pi:
    if (error_heading < -PI)
    {
        error_heading += 2*PI;
    }
    if (error_heading >= PI)
    {
        error_heading -= 2*PI;
    }

    calculate_speed();
}

void Control::calculate_speed(){
    geometry_msgs::Twist base_cmd;


    //****** PI-controller *********

    //Proportional term

    //Integral term
    I_heading += dt * error_heading;
    I_pos += dt * error_pos;

/*
    std::cout << "Proportional_fwd: " << Kp_fwd * error_pos << std::endl;
    std::cout << "Integral_fwd: " << Ki_fwd * I_pos << std::endl;
    std::cout << "Derivate_fwd: " << Kd_fwd * (error_pos_prev - error_pos) << std::endl;
    std::cout << "Proportional_head: " << -Kp_ang * error_heading<< std::endl;
    std::cout << "Integral_head: " << - Ki_ang * I_heading << std::endl;

*/
    //Compute gain


    if (abs(error_heading) > 0.20) {
        //heading
        trans_heading = -Kp_ang * error_heading - Ki_ang * I_heading;
        std::cout << "trans_heading before: " << trans_heading;
        trans_heading *= pow(2.0,(trans_heading)+1.0);
        std::cout << " ->   trans_heading after: " << trans_heading << std::endl;

        //velocity
        trans_x = Kp_fwd * error_pos + Ki_fwd * I_pos + Kd_fwd * (error_pos_prev - error_pos);
        std::cout << "trans_x before: " << trans_x;
        trans_x *= (1.0/pow(4.0,(trans_heading)+1.0));
        std::cout << " ->   trans_x after: " << trans_x << std::endl;
    }else{
        trans_heading = -Kp_ang * error_heading - Ki_ang * I_heading;
        trans_x = Kp_fwd * error_pos + Ki_fwd * I_pos + Kd_fwd * (error_pos_prev - error_pos);
    }
    //******************************
    std::cout << "trans_heading: " << trans_heading * 180.0 / PI << " deg/s " << std::endl;
    std::cout << "trans_x: " << trans_x << " m/s " << std::endl;

    /*
    std::cout << "---------------------------------" << std::endl;
    std::cout << "(x,y) = (" << pos_x << ", " << pos_y << ") " << std::endl;
    std::cout << "Setpoint (x,y) = (" << setpoint_x << ", " << setpoint_y << ") " << std::endl;
    std::cout << "error_x: " << error_x << std::endl;
    std::cout << "error_y: " << error_y << std::endl;
    std::cout << "error_pos " << error_pos << std::endl;
    std::cout << "heading: " << heading << " [rad] = " << heading * 180.0 / PI << " [deg] " << std::endl;
    std::cout << "setpoint_heading: " << setpoint_heading << " [rad] = " << setpoint_heading * 180.0 / PI << " [deg] " << std::endl;
    std::cout << "error_heading " << error_heading << " [rad] = " << error_heading * 180.0 / PI << " [deg] " << std::endl;
    std::cout << "trans_heading: " << trans_heading * 180.0 / PI << " deg/s " << std::endl;
    std::cout << "trans_x: " << trans_x << " m/s " << std::endl;
*/


    base_cmd.linear.x = trans_x;
    base_cmd.angular.z = trans_heading;
    pub_control_speed.publish(base_cmd);
}


void Control::spin(){
    ros::Rate loop_rate(1.0 / dt);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "node_controller");
    ros::NodeHandle nh;

    Control *control;
    Control temp_control(nh);
    control = &temp_control;
    control->spin();

    return 0;
}