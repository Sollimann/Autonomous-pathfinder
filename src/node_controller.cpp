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

            //topics
            ros::Publisher pub_control_speed;
            ros::Subscriber sub_setpoint;
            ros::Subscriber sub_position;

            //variables
            double trans_x, trans_z;
            double pos_x, pos_y, ori_z, ang_z,set_x,set_y,set_z;
            double error_x, error_y, error_z, error_dist;

        public:

            /* Constructor */
            Control(ros::NodeHandle nh);

            /* Constants */
            float PI;

            //Forward gain
            double Kp_fwd = 10.0;


            //Heading gain
            double Kp_ang = 0.05;


            //Functions

            //This function will subscribe to setpoint messages from
            //the pathplanner node.
            void get_setpoint(const geometry_msgs::Point& setpointMsg);
            void get_position(const geometry_msgs::Point& posMsg);
            void calculate_speed();
};



            //Constructor
            Control::Control(ros::NodeHandle nh) {

                //Initialize
                pos_x = 0;
                pos_y = 0;
                ori_z = 0;
                ang_z = 0;
                set_x = 0;
                set_y = 0;
                set_z = 0;
                error_x = 0;
                error_y = 0;
                error_z = 0;
                error_dist = 0;


                PI = 3.14159265359;
                //callback updates set point position in x and y
                sub_position = nh.subscribe("pathplanner/x_y_yaw",1,&Control::get_position,this);
                sub_setpoint = nh.subscribe("/pathplanner/setpoint_smooth",1,&Control::get_setpoint, this);


                //pub
                pub_control_speed = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);


            }

            //Subscribes to current position
            void Control::get_position(const geometry_msgs::Point& posMsg) {

                //raw position in Euler
                pos_x = posMsg.x;
                pos_y = posMsg.y;
                    ang_z = posMsg.z;


                //std::cout << "Position x: " << pos_x << std::endl;
                //std::cout << "Position y: " << pos_y << std::endl;
                //std::cout << "Orientation z: " << ang_z << std::endl;
            }



            //This function will subscribe to setpoint messages from
            //the pathplanner node.
            void Control::get_setpoint(const geometry_msgs::Point& setpointMsg) {

                //Position from pathplanner node
                set_x = setpointMsg.x;
                set_y = setpointMsg.y;


                //set_z = setpointMsg.z;  // [-pi,pi]calculate setpoint for the angle

                //std::cout << "setpoint_x: " << set_x << std::endl;
                //std::cout << "setpoint_y: " << set_y << std::endl;
                //std::cout << "setpoint_z: " << set_z << std::endl;



                //Calculate setpoint heading
                error_x = (set_x - pos_x);
                error_y = (set_y - pos_y);

                /*
                if (fabs(error_x) < 0.001 && error_y > 0.1) {
                    set_z = 2 * atan2(error_x * sin(ang_z / 2), cos(ang_z / 2));
                }
                else if(fabs(error_x) < 0.001 && error_y < -0.1){

                    set_z = -2 * atan2(error_x * sin(ang_z/2),cos(ang_z/2));
                }
                else{
                    set_z = atan2(error_y, error_x);
                }

                 */
               //run and publish

                 calculate_speed();


                //set_z = acos(error_x*sin(ang_z))/();
            }


            void Control::calculate_speed() {
                /*
                // Yaw angle is defined as [0, 2pi], so the residuals can not be less than or grater than +-pi:
                if (error_z < -PI)
                {
                    error_z += 2*PI;
                }
                if (error_z >= PI)
                {
                    error_z -= 2*PI;
                }
                */

                error_z = acos(error_y*sin(ang_z)+error_x*cos(ang_z))/sqrt(error_x*error_x + error_y*error_y);
                error_dist = sqrt(error_x*error_x + error_y*error_y);
                float theta_z = (error_x*sin(ang_z)+error_y*cos(ang_z))/sqrt(error_x*error_x + error_y*error_y);



                //P-controller
                trans_z = Kp_ang * error_z;
                trans_x = Kp_fwd * error_dist;


                std::cout << "error_x " << error_x << std::endl;
                std::cout << "error_y " << error_y << std::endl;
                //std::cout << "set_z " << set_z << std::endl;
                std::cout << "ang_z " << ang_z << std::endl;
                std::cout << "error_z " << error_z << std::endl;
                std::cout << "error_dist " << error_dist << std::endl;
                //std::cout << "abs(error_z) " << fabs(error_z) << std::endl;

                geometry_msgs::Twist base_cmd;

                if (fabs(error_z) > 0.02 ){
                    base_cmd.angular.z = trans_z;
                    base_cmd.linear.x = 0;
                    std::cout << "trans_z: " << trans_z << std::endl;
                }
                else {
                    base_cmd.linear.x = trans_x;
                    std::cout << "TRANS_X: " << trans_x << std::endl;
                    trans_x = 0;
                }


                pub_control_speed.publish(base_cmd);
            }


int main(int argc, char** argv){
    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    Control control(nh);

    ros::spin();
    return 0;
}