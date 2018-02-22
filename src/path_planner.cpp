#include <ros/ros.h>
#include <utility> //stack
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
#include <std_msgs/Float32MultiArray.h>

using namespace arma;
//enum {UNKNOWN = 0, OPEN = 1, WALL = 2};
enum {OPEN = 0, WALL = 1};


class PathPlanner{
private:
    ros::Subscriber sub_pos;
    ros::Subscriber sub_dist;
    ros::Publisher pub_point;
    ros::Publisher pub_x_y_yaw;
    mat path_points;
    double pos_x, pos_y, ang_z, setpoint_x, setpoint_y;
    int pos_x_int_last, pos_y_int_last;
    int current_point;
    double threshold_switch, dt, PI, dist_mid;
    cube wall_map;

    //Flood fill
    mat flood_fill_map;

    //Stack of pair coordinates
    typedef std::pair <int, int> Stack;
    std::vector<Stack> coordinate;
    bool sector_checked;

public:
    PathPlanner(ros::NodeHandle &nh);
    void callback_odom( const nav_msgs::OdometryConstPtr& poseMsg);
    void callback_distances( const std_msgs::Float32MultiArray& distMsg);
    void initializeWallMap();
    void printWallMap();
    void spin();
    void set_wall(int pos_x_int, int pos_y_int, int direction);
    std::vector<unsigned int> get_wall(int pos_x_int, int pos_y_int);
    void fill_walls();


    //Flood fill
    void initializeFloodFillMap();
    void set_flood_fill_map_value(int x_pos, int y_pos, int value);
    int get_flood_fill_map_value(int x_pos, int y_pos);
    void set_next_destination_cell();

    //Track the path of the robot
    //Stack
};


PathPlanner::PathPlanner(ros::NodeHandle &nh){
    // Initializing wall map:
    initializeWallMap();
    initializeFloodFillMap();

    // Setting the row of path_points which is the current target setpoint
    current_point = 0;
    threshold_switch = 0.01;
    dt = 0.1;
    PI = 3.14159265;
    dist_mid = 3;

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
    sub_dist = nh.subscribe("/distance_logger/distances",1,&PathPlanner::callback_distances, this);
    pub_point = nh.advertise<geometry_msgs::Point>("/pathplanner/setpoint",1);
    pub_x_y_yaw = nh.advertise<geometry_msgs::Point>("pathplanner/x_y_yaw",1);

}

void PathPlanner::callback_odom( const nav_msgs::OdometryConstPtr& poseMsg){

    geometry_msgs::Point cmd_setpoint;
    geometry_msgs::Point cmd_x_y_yaw;

    // Redefining coordinate system from
    //          x       y
    //          |  to   |
    //          |       |
    // y <-------       -------> x

    pos_y = poseMsg->pose.pose.position.x;
    pos_x = -(poseMsg->pose.pose.position.y);

    // Check if x and y coordinate is close to a node position. Close = within 0.2m x 0.2m square around node position
    // If so, we want to check for walls while being close to this point
    if (fabs(round(pos_y) - pos_y) < 0.4 && fabs(round(pos_x) - pos_x) < 0.4){
        fill_walls();
    }

    //setpoint_x = path_points(current_point, 0);
    //setpoint_y = path_points(current_point, 1);

    double error_pos = (setpoint_x - pos_x) * (setpoint_x - pos_x) + (setpoint_y - pos_y) * (setpoint_y - pos_y);
    //std::cout << "Setpoint (x,y) =  (" << setpoint_x << ", " << setpoint_y << ")" << std::endl;
    //std::cout << "Position (x,y) =  (" << pos_x << ", " << pos_y << ")" << std::endl;
    //std::cout << "Error: " << error << std::endl;


    if (error_pos < threshold_switch){

        //set_next_destination_cell();

        //current_point++;
        //setpoint_x = path_points(current_point, 0);
        //setpoint_y = path_points(current_point, 1);
    }

    cmd_setpoint.x = setpoint_x;
    cmd_setpoint.y = setpoint_y;
    //cmd_setpoint.z = 0;

    pub_point.publish(cmd_setpoint);

    // Calculating orientation [-PI, PI]
    float x = poseMsg->pose.pose.orientation.x;
    float y = poseMsg->pose.pose.orientation.y;
    float z = poseMsg->pose.pose.orientation.z;
    float w = poseMsg->pose.pose.orientation.w;

    // Converting quaterninon to yaw angle:
    ang_z = -atan2((2.0 * (w*z + x*y)), (1.0 - 2.0 * (y*y + z*z)));
    //std::cout << "Angle: " << ang_z << std::endl;

    cmd_x_y_yaw.x = pos_x;
    cmd_x_y_yaw.y = pos_y;
    cmd_x_y_yaw.z = ang_z;
    pub_x_y_yaw.publish(cmd_x_y_yaw);
}

void PathPlanner::callback_distances( const std_msgs::Float32MultiArray& distMsg){
    dist_mid = distMsg.data[1];
}

void PathPlanner::initializeWallMap(){

    // 9x9 matrix containing information if there is a wall or not in direction
    // North, East, South and/or West for the point.
    // Each point has a vector of 4 elements indicating whether or not a wall is nearby, and in which direction.
    // [N, E, S, W] could either be OPEN or WALL
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

    //printWallMap();
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


void PathPlanner::set_next_destination_cell() {

    // 1.) Detect my current cell
    int pos_x_int = (int) round(pos_x);
    int pos_y_int = (int) round(pos_y);
    pos_x_int_last = pos_x_int;
    pos_y_int_last = pos_y_int;

    // 2.) Update the stack with all visited locations
    if ((pos_x_int != pos_x_int_last) && (pos_y_int != pos_y_int_last)) {
        coordinate.push_front(std::make_pair(pos_x_int, pos_y_int));

        // 3.) Investigate my surrounding cells
        // Here we need to check all surrounding four cells

        //Initialize the nearby region of cells in a vector
        std::vector<int> region;
        region.reserve(4);

        //Check if the surrounding cells already have registered a wall
        std::vector<unsigned int> wall_vec = get_wall(pos_x_int,pos_y_int);

        //Fill up the region with flood fill values [N,E,S,W]
        //region[int,int,int,int]
        region(0) = get_flood_fill_map_value(pos_x_int,pos_y_int+1);
        region(1) = get_flood_fill_map_value(pos_x_int+1,pos_y_int);
        region(2) = get_flood_fill_map_value(pos_x_int,pos_y_int-1);
        region(3) = get_flood_fill_map_value(pos_x_int-1,pos_y_int);

        //Iterator detects value of largest, *points to the value
        int MAX = *std::max_element(region.begin(), region.end());
    }


    //Scan flood fill vector to find the lowest possible entry
    int min_position = std::min_element(region.begin(), region.end());

    //Checking lowest value cell entry has a wall
    while(wall_vec(min_position) == 1){
        region(min_position) = MAX+1;
        min_position = std::min_element(region.begin(), region.end());
    }


    if(wall_vec(min_position) == 0) {
        cmd_setpoint.x = setpoint_x;
        cmd_setpoint.y = setpoint_y;
        cmd_setpoint.z = min_position;
    }else{
        cmd_setpoint.x = setpoint_x;
        cmd_setpoint.y = setpoint_y;
        cmd_setpoint.z = 4; //value 5 indicates that we want to give our next destination
    }




    //North coordinate


            // --> 3i.) Check flood fill matrix in surrounding cells

            // --> 3ii.) When all cells have been tracked, check if the lowest value cell has a wall

                            // All ACCESSIBLE cell-values will be compared to my cell value
                            // if, my cell value is lower than any surrounding cells
                            // --> update the cell’s value to one greater than the minimum
                            //value of its accessible neighbours

                            // else if (one surrounding cell is lower than the others)
                            // --> present this cell as next destination coordinate

                            // else if (surrounding ACCESSIBLE cells have same value)
                            // --> pick the cell you have not yet accessed

            // --> 3iii.) Check only the the vector index of the wall closest to my cell position

            // --> 3iii.) If, (the cell has a wall (is not open) closest to my current position)
            // --> Then this cell is not ACCESSIBLE
            //            else. --> cell is ACCESSIBLE






    // 4.) Publish the next setpoint for the controller to read









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
    //flood_fill_map.print();

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


void PathPlanner::set_wall(int pos_x_int, int pos_y_int, int direction){
    // direction is a number = {0=North, 1=East, 2=South, 3=west}
    // (row,col)=(0,0) is bottom left
    // (row,col)=(0,8) is bottom right
    // (row,col)=(8,8) is top right
    // (row,col)=(8,0) is top left

    if (wall_map(pos_y_int, pos_x_int, direction) != WALL) {
        // Detected a wall we did not know about
        wall_map(pos_y_int, pos_x_int, direction) = WALL;

        if (direction == 0 && pos_y_int != 8) { //North wall. Fill south wall to adjecent node
            wall_map(pos_y_int + 1, pos_x_int, 2) = WALL;
            std::cout << "Wall detected north for position (x,y) = (";
        } else if (direction == 1 && pos_x_int != 8) { //East wall. Fill west wall to adjecent node
            wall_map(pos_y_int, pos_x_int + 1, 3) = WALL;
            std::cout << "Wall detected east for position (x,y) = (";
        } else if (direction == 2 && pos_y_int != 0) { //South wall. Fill north wall to adjecent node
            wall_map(pos_y_int - 1, pos_x_int, 0) = WALL;
            std::cout << "Wall detected south for position (x,y) = (";
        } else if (direction == 3 && pos_x_int != 0) { //West wall. Fill east wall to adjecent node
            wall_map(pos_y_int, pos_x_int - 1, 1) = WALL;
            std::cout << "Wall detected west for position (x,y) = (";
        }

        std::cout << pos_x_int << ", " << pos_y_int << ")" << std::endl;
        std::cout << "New wall map:" << std::endl;
        //printWallMap();
    }
}


std::vector<unsigned int> PathPlanner::get_wall(int pos_x_int, int pos_y_int){

    int N = wall_map(pos_y_int,pos_x_int,0);
    int E = wall_map(pos_y_int,pos_x_int,1);
    int S = wall_map(pos_y_int,pos_x_int,2);
    int W = wall_map(pos_y_int,pos_x_int,3);

    return {N,E,S,W};
}

void PathPlanner::fill_walls(){
    int pos_x_int = (int) round(pos_x);
    int pos_y_int = (int) round(pos_y);
    int direction = -1;
    double ang_z_deg = ang_z * 180.0 / PI;

    if (ang_z_deg > 0 && ang_z_deg < 3){
        direction = 0; // North
    }
    else if (ang_z_deg < 0 && ang_z_deg > -3){
        direction = 0; // North
    }
    else if (ang_z_deg > 87 && ang_z_deg < 93){
        direction = 1; // East
    }
    else if (ang_z_deg > 177 || ang_z_deg < -177){
        direction = 2; // South
    }
    else if (ang_z_deg < -87 && ang_z_deg > -93){
        direction = 3; // West
    }

    if (dist_mid < 1.5 && direction != -1){
        set_wall(pos_x_int, pos_y_int, direction);
    }else if(direction != -1){
        sector_checked = true;
    }else{sector_checked = false}

}


void PathPlanner::spin(){
    ros::Rate loop_rate(1.0 / dt);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

PathPlanner *pp;

int main(int argc, char** argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    PathPlanner temp_pp(nh);
    pp = &temp_pp;
    pp->spin();

    //PathPlanner pp(nh);
    //ros::spin();
    return 0;
}