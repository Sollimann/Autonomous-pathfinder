#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

class BotController{
    private:
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;
	ros::Subscriber scanSub;

        int count, wall, goal;
        double trans_x, trans_z;
        double pos_x, pos_y, ori_z, ang_z;
        double target_x, target_y, target_o, target_r;
        double init_r, init_ang_z, init_x, init_y;
	std::vector<float> dist;
	float min_dist;
    public:
        BotController(ros::NodeHandle &nh){
		count = 0;
		wall = 0;
		trans_x = 0;
		trans_z = 0;
		goal = 0;
 		scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&BotController::processLaserScan,this);
            	pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
            	vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
           
    	}
    void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
        dist = scan->ranges;
            min_dist = 100;
		
       
        //min_dist = std::min_element(dist.front(),dist.back());
     for (int i=1; i< dist.size(); i++) {
                     if (dist[i] < min_dist && dist[i] > 0.1) {
                      min_dist = dist[i];
                      }
                    }
    }


        void callback( const nav_msgs::OdometryConstPtr& poseMsg){
		double PI_ = 2.7;
		int turn = 0;
		geometry_msgs::Twist base_cmd;
		target_x = 5; // Change travel distance here
		target_o = PI_/2;;
		pos_x = poseMsg->pose.pose.position.x;
		pos_y = poseMsg->pose.pose.position.y;
		ori_z = poseMsg->pose.pose.orientation.z;
		ang_z = ori_z*2.19;



		if(count == 0){
			init_x = poseMsg->pose.pose.position.x;
			init_ang_z = ori_z*2.19;

			count = 1;
		}// count = 0 ends here

		if(pos_x - init_x < target_x){
			trans_x = 0.6;//*dist; // Change robot velocity
		}else{
		turn = 1;
		trans_x = 0;
		}

		if (min_dist < 1.2){
			turn = 1;
			if (min_dist < 0.7){
				turn = 1;
				trans_x = 0;
			}
		}

		if (turn == 1){
			if( fabs(ang_z - init_ang_z) < target_o){
			trans_z = -0.5; // Change robot angular velocity
			}else{
				trans_z = 0;
				count = 0;
				wall = 1;
			}
		}

		if (wall == 1){

			if (dist[639] > 1.5){
				turn = 2;
				trans_x = 0.15;
			}
		}



		if (turn == 2){
			if( fabs(ang_z - init_ang_z) < target_o){
			trans_z = 0.3; // Change robot angular velocity
			}else{
				trans_z = 0;
				count = 0;
			}
		}

		if ( goal == 0) {
			if ( ( 8.8 < pos_x ) && ( pos_x < 11 ) && ( -5.6 < pos_y ) && ( pos_y < -2 ) ){
				ros::Duration(5.0).sleep();
				goal = 1;
			
			}
		}

		if ( goal == 1) {
			if ( ( 0.1 < pos_x ) && ( pos_x < 1.4 ) && ( -3.9 < pos_y ) && ( pos_y < -3.7 ) ){
				//ros::Duration(5.0).sleep();
				goal = 2;
				
			
			}
		}

		if (goal == 2){
			if( fabs(ang_z - init_ang_z) < target_o){
			trans_x = 0.3;
			trans_z = 0.3; // Change robot angular velocity
			}else{
				trans_z = 0;
				count = 0;
				goal = 0;
			}
		}

		
		if ( ( -1 < pos_x ) && ( pos_x < -0.5 ) && ( -4 < pos_y ) && ( pos_y < -1.5 ) ){
				trans_x = 0;
				trans_z = 0;
		}


		base_cmd.linear.x = trans_x;
		if( (turn == 1) || (turn == 2) || (goal == 2)){
			base_cmd.angular.z = trans_z;
		}
		

            vel_pub.publish(base_cmd);
            std::cout<< std::setprecision(2) << std::fixed;
            std::cout << poseMsg->header.stamp
                      << " Current:" << pos_x << "," << pos_y << std::endl;

        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    BotController bc(nh);

    ros::spin();
    return 0;
}
