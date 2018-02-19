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

using namespace arma;
using namespace std_msgs;

class Control{

        private:

            //topics
            ros::Publisher pub_control_speed;
            ros::Subscriber sub_position;

            //variables
            double trans_x, trans_z;
            double pos_x, pos_y, ori_z, ang_z,set_x,set_y,set_z,quat;

        public:

            /* Constructor */
            Control(ros::NodeHandle nh);

            /* Constants */

            //Forward gain
            static double Kp_fwd = 0.1;


            //Heading gain
            static double Kp_ang = 0.1;

            //callback updates set point position in x and y
            sub_setpoint = nh.subscribe("/pathplanner/setpoint_smooth",1,&Control::get_setpoint, this);
            sub_position = nh.subscribe("/odom",1,&PathPlanner::get_position, this);

            //pub
            pub_control_speed = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);


            //Functions

            //This function will subscribe to setpoint messages from
            //the pathplanner node.
            void get_setpoint(const geometry_msgs::Point& setpointMsg);
            void get_position(const nav_msgs::OdometryConstPtr& posMsg);
            void calculate_speed();

};

            //Subscribes to current position
            void Control::get_position( const nav_msgs::OdometryConstPtr& posMsg) {

                pos_x = posMsg->pose.pose.position.x;
                pos_y = -(posMsg->pose.pose.position.y);

                //Quartanions convertion to euler
                float x = poseMsg->pose.pose.orientation.x;
                float y = poseMsg->pose.pose.orientation.y;
                float z = poseMsg->pose.pose.orientation.z;
                float w = poseMsg->pose.pose.orientation.w;

                ang_z = atan2((2.0 * (w*z + x*y)), (1.0 - 2.0 * (y*y + z*z)));
                std::cout << "ang: " << ang_z << std::endl;

                std::cout << "Position x: " << pos_x << std::endl;
                std::cout << "Position y: " << pos_y << std::endl;
                std::cout << "Orientation z: " << ang_z << std::endl;


            }


            //This function will subscribe to setpoint messages from
            //the pathplanner node.
            void Control::get_setpoint(const geometry_msgs::Point& setpointMsg) {

                set_x = setpointMsg->x;
                set_y = setpointMsg->y;
                set_z = atan2(y,x);   //calculate setpoint for the angle

                std::cout << "setpoint_x: " << set_x << std::endl;
                std::cout << "setpoint_y: " << set_y << std::endl;
                std::cout << "setpoint_z: " << set_z << std::endl;
            }


            void Control::calculate_speed() {

                //setpoint in 2 or 3 quadrant
                //orientation in 2 or 3 quadrant
                if (set_z < 0 && ang_z < 0){

                    error_z = set_z - ang_z;

                }
                //setpoint in 2 or 3 quadrant
                //orientation in 1 or 4 quadrant
                else if (set_z < 0 && ang_z > 0){

                    error_z = min(,)

                }
                //setpoint in 1 or 4 quadrant
                //orientation in 1 or 4 quadrant
                else if (set_z > 0 && ang_z > 0){

                    error_z = set_z - ang_z;

                }
                //setpoint in 1 or 4 quadrant
                //orientation in 2 or 3 quadrant
                else {


                }

                error_x = abs(set_x) - abs(pos_x);
                error_y = abs(set_y - pos_y;
                error_dist = sqrt(error_x^2 + error_y^2);




                //Calculate proper angle error


                error_z =   set_z - ang_z;


                //P-controller
                trans_x = Kp_fwd * error_dist;
                trans_z = Kp_ang * error_z;


                geometry_msgs::Twist base_cmd;
                base_cmd.linear.x = trans_x;
                base_cnd.linear.z = trans_z;


                //pub_control_speed.publish(base_cmd);
            }


int main(int argc, char** argv){
    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    Control control(nh);

    ros::spin();
    return 0;
}