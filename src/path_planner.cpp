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
#include <deque>
#include <queue>
#include <math.h>
#include <armadillo>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
using namespace std;
using namespace arma;
enum {OPEN = 0, WALL = 1};

//#define PI          3.14159265

class PathPlanner{
private:
    ros::Subscriber sub_pos;
    ros::Subscriber sub_dist;
    ros::Publisher pub_point;
    ros::Publisher pub_x_y_yaw;

    int pos_x_int_last, pos_y_int_last, setpoint_x, setpoint_y, setpoint_x_prev, setpoint_y_prev;
    double pos_x, pos_y, ang_z, THRESHOLD_SWITCH, dt, PI, dist_mid, dist_left, dist_right;
    double dist_side_max;
    cube wall_map;

    //Flood fill
    mat flood_fill_map;
    vec neighbour_flood_fill_values = zeros<vec>(4);
    int min_heading, current_flood_fill_value;

public:
    PathPlanner(ros::NodeHandle &nh);
    void callbackOdom( const nav_msgs::OdometryConstPtr& poseMsg);
    void callbackDistances( const std_msgs::Float32MultiArray& distMsg);
    void initializeWallMap();
    void printWallMap();
    void printQueue(std::deque<pair<int,int> > queue);
    void spin();
    void setWall(int x_pos, int y_pos, int direction);
    bool hasWall(int x_pos, int y_pos, int direction);
    void checkForWalls();

    //Flood fill
    void initializeFloodFillMap();
    void setFloodFillMapValue(int x_pos, int y_pos, int value);
    int getFloodFillMapValue(int x_pos, int y_pos);
    bool isPreviousPosition(int x_pos, int y_pos, int heading);
    void setNextDestinationCell();
    void updateFloodFillMap();
    double getManhattanDistance(double x_pos, double y_pos);

};


PathPlanner::PathPlanner(ros::NodeHandle &nh){
    // Initializing wall map:
    initializeWallMap();
    initializeFloodFillMap();
    pos_x_int_last = -1;
    pos_y_int_last = -1;

    THRESHOLD_SWITCH = 0.0025;
    dt = 0.1;
    PI = 3.14159265;
    dist_mid = 3;
    dist_right = 3;
    dist_left = 3;
    setpoint_x = 0;
    setpoint_y = 0;
    dist_side_max = 1.1;

    // Defining subscribers and publishers:
    sub_pos = nh.subscribe("/odom",1,&PathPlanner::callbackOdom, this);
    sub_dist = nh.subscribe("/distance_logger/distances",1,&PathPlanner::callbackDistances, this);
    pub_point = nh.advertise<geometry_msgs::Point>("/pathplanner/setpoint",1);
    pub_x_y_yaw = nh.advertise<geometry_msgs::Point>("pathplanner/x_y_yaw",1);

}

void PathPlanner::callbackOdom( const nav_msgs::OdometryConstPtr& poseMsg){

    geometry_msgs::Point cmd_setpoint;
    geometry_msgs::Point cmd_x_y_yaw;

    // Redefining coordinate system from
    //          x       y
    //          |  to   |
    //          |       |
    // y <-------       -------> x

    pos_y = poseMsg->pose.pose.position.x;
    pos_x = -(poseMsg->pose.pose.position.y);
    double error_pos = (setpoint_x - pos_x) * (setpoint_x - pos_x) + (setpoint_y - pos_y) * (setpoint_y - pos_y);

    checkForWalls();

    if (error_pos < THRESHOLD_SWITCH){

        setpoint_x_prev = setpoint_x;
        setpoint_y_prev = setpoint_y;

        if (!(setpoint_x == 4 && setpoint_y == 4)){
            updateFloodFillMap();
            setNextDestinationCell();
        }

    }

    cmd_setpoint.x = setpoint_x;
    cmd_setpoint.y = setpoint_y;
    cmd_setpoint.z = 4;

    pub_point.publish(cmd_setpoint);

    // Calculating orientation [-PI, PI]
    float x = poseMsg->pose.pose.orientation.x;
    float y = poseMsg->pose.pose.orientation.y;
    float z = poseMsg->pose.pose.orientation.z;
    float w = poseMsg->pose.pose.orientation.w;

    // Converting quaterninon to yaw angle:
    ang_z = -atan2((2.0 * (w*z + x*y)), (1.0 - 2.0 * (y*y + z*z)));

    cmd_x_y_yaw.x = pos_x;
    cmd_x_y_yaw.y = pos_y;
    cmd_x_y_yaw.z = ang_z;
    pub_x_y_yaw.publish(cmd_x_y_yaw);
}

void PathPlanner::callbackDistances( const std_msgs::Float32MultiArray& distMsg){
    dist_mid = distMsg.data[1];
    dist_left = distMsg.data[0];
    dist_right = distMsg.data[2];

    // Use the distance to the left to estimate the wall thickness.
    // Use this wall thickness to find a limit which determines if we measure an open space or wall
    // when doing wall detection
    static bool initialized = false;
    if (!initialized){
        double wall_thickness = 1 - 2 * dist_left * cos(60.0 * PI / 180.0);
        double alpha = (1.2 - 0.85) / (0.29 - 0.19);
        dist_side_max = 1.2 - alpha * (wall_thickness - 0.19);
        std::cout << "Estimated wall thickness: " << wall_thickness << " meters " << std::endl;
        std::cout << "Gives dist_side_max = " << dist_side_max << std::endl;
        initialized = true;
    }
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



void PathPlanner::printQueue(std::deque<pair<int,int> > stack) {
    std::deque<pair<int,int> > q = stack;
    std::cout << "Stack: " << endl;
    while (!q.empty())
    {
        std::cout << "(" << q.back().first << " , " << q.back().second << ")" << std::endl;
        q.pop_back();
    }
    std::cout << std::endl;
    std::cout << "Stack ending " << std::endl;
}


bool PathPlanner::isPreviousPosition(int x_pos, int y_pos, int heading){

    if (heading == 0){
        y_pos++;
    }else if(heading == 1){
        x_pos++;
    }else if(heading == 2){
        y_pos--;
    }else{
        x_pos--;
    }

    return (x_pos == pos_x_int_last && y_pos == pos_y_int_last);

}


void PathPlanner::setNextDestinationCell() {
    std::cout << "Switching set point..." << std::endl;

    // 1.) Detect my current cell
    int pos_x_int = (int) round(pos_x);
    int pos_y_int = (int) round(pos_y);
    double manhattan_distance = 1000;

    // 2.) Update the stack with all visited locations
    neighbour_flood_fill_values(0) = getFloodFillMapValue(pos_x_int, pos_y_int + 1);
    neighbour_flood_fill_values(1) = getFloodFillMapValue(pos_x_int + 1, pos_y_int);
    neighbour_flood_fill_values(2) = getFloodFillMapValue(pos_x_int, pos_y_int - 1);
    neighbour_flood_fill_values(3) = getFloodFillMapValue(pos_x_int - 1, pos_y_int);

    //std::cout << "Printed flood fill values to neighbours: " << std::endl;
    //neighbour_flood_fill_values.print();

    //Update the flood fill value of my current cell
    current_flood_fill_value = getFloodFillMapValue(pos_x_int, pos_y_int);

    // Find the neighbouring cell without a wall in between with the lowest flood fill value
    int minimumReachableFloodFillValue = 10000;
    for (int i = 0; i < 4; i++){
        if (!hasWall(pos_x_int, pos_y_int, i)){
            //Evaluate if this is the next destination
            double x_next, y_next;
            if (i == 0){x_next = pos_x_int;       y_next = pos_y_int + 1;}
            if (i == 1){x_next = pos_x_int + 1;   y_next = pos_y_int;}
            if (i == 2){x_next = pos_x_int;       y_next = pos_y_int - 1;}
            if (i == 3){x_next = pos_x_int - 1;   y_next = pos_y_int;}

            int neighbourCellFloodFillValue = neighbour_flood_fill_values(i);
            if (neighbourCellFloodFillValue < minimumReachableFloodFillValue){
                minimumReachableFloodFillValue = neighbourCellFloodFillValue;
                min_heading = i;
                std::cout << "n < m" << std::endl;
                manhattan_distance = getManhattanDistance(x_next, y_next);
            }
            else if (neighbourCellFloodFillValue == minimumReachableFloodFillValue){
                std::cout << "n == m" << std::endl;

                // Choose the heading which minimizes (pos_x - pos_x_goal)^2 + (pos_y - pos_y_goal)^2
                double manhattan_distance_new = getManhattanDistance(x_next, y_next);
                if (manhattan_distance_new < manhattan_distance){
                    min_heading = i;
                    manhattan_distance = manhattan_distance_new;
                    std::cout << "Choosing heading " << i << " since it minimizes Manhattan distance" << std::endl;
                }

                // Prioritize another heading than where we came from
                if(isPreviousPosition(pos_x_int, pos_y_int, min_heading)){
                    min_heading = i;
                    std::cout << "isPreviousPosition true" << std::endl;
                }
            }
        }
    }


    std::cout << "Best heading: " << min_heading << std::endl;

    //New setpoint
    if (min_heading == 0) {
        setpoint_y++;
    } else if (min_heading == 1) {
        setpoint_x++;
    } else if (min_heading == 2) {
        setpoint_y--;
    } else {
        setpoint_x--;
    }

    //Print flood fill
    std::cout << "Flood fill map: " << std::endl;
    flood_fill_map.print();
    std::cout << std::endl;

    pos_x_int_last = pos_x_int;
    pos_y_int_last = pos_y_int;

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

    flood_fill_map  << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
                    << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr;

    /*
    flood_fill_map      << 16 << 15 << 14 << 13 << 12 << 11 << 10 << 9 << 8 << endr
                        << 15 << 14 << 13 << 12 << 11 << 10 << 9 << 8 << 7 << endr
                        << 14 << 13 << 12 << 11 << 10 << 9 << 8 << 7 << 6 << endr
                        << 13 << 12 << 11 << 10 << 9 << 8 << 7 << 6 << 5 << endr
                        << 12 << 11 << 10 << 9 << 8 << 7 << 6 << 5 << 4 << endr
                        << 11 << 10 << 9 << 8 << 7 << 6 << 5 << 4 << 3 << endr
                        << 10 << 9 << 8 << 7 << 6 << 5 << 4 << 3 << 2 << endr
                        << 9 << 8 << 7 << 6 << 5 << 4 << 3 << 2 << 1 << endr
                        << 8 << 7 << 6 << 5 << 4 << 3 << 2 << 1 << 0 << endr;
                        */

    std::cout << "Flood fill map initialized!" << std::endl;
    //std::cout <<  " Printed: " << std::endl;
    //flood_fill_map.print();
}


void PathPlanner::setFloodFillMapValue(int x_pos, int y_pos, int value){
    flood_fill_map(8 - y_pos, x_pos) = value;
}

int PathPlanner::getFloodFillMapValue(int x_pos, int y_pos){

    if (x_pos < 0 || y_pos < 0 || x_pos > 8 || y_pos > 8){
        return 1000;}
    else{
    return flood_fill_map(8 - y_pos, x_pos);}
}


void PathPlanner::setWall(int x_pos, int y_pos, int direction){
    // direction is a number = {0=North, 1=East, 2=South, 3=west}
    // (row,col)=(0,0) is bottom left
    // (row,col)=(0,8) is bottom right
    // (row,col)=(8,8) is top right
    // (row,col)=(8,0) is top left

    if (wall_map(y_pos, x_pos, direction) != WALL) {
        // Detected a wall we did not know about
        wall_map(y_pos, x_pos, direction) = WALL;

        if (direction == 0 && y_pos != 8) { //North wall. Fill south wall to adjecent node
            wall_map(y_pos + 1, x_pos, 2) = WALL;
            std::cout << "Wall detected north for position (x,y) = (";
        } else if (direction == 1 && x_pos != 8) { //East wall. Fill west wall to adjecent node
            wall_map(y_pos, x_pos + 1, 3) = WALL;
            std::cout << "Wall detected east for position (x,y) = (";
        } else if (direction == 2 && y_pos != 0) { //South wall. Fill north wall to adjecent node
            wall_map(y_pos - 1, x_pos, 0) = WALL;
            std::cout << "Wall detected south for position (x,y) = (";
        } else if (direction == 3 && x_pos != 0) { //West wall. Fill east wall to adjecent node
            wall_map(y_pos, x_pos - 1, 1) = WALL;
            std::cout << "Wall detected west for position (x,y) = (";
        }

        std::cout << x_pos << ", " << y_pos << ")" << std::endl;
        //std::cout << "New wall map:" << std::endl;
        //printWallMap();
    }
}


bool PathPlanner::hasWall(int x_pos, int y_pos, int direction){
    if (wall_map(y_pos, x_pos, direction) == WALL){
        return true;
    }
    else{
        return false;
    }
}

void PathPlanner::checkForWalls(){
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

    // Check if x and y coordinate is close to a node position. Close = within 0.4m x 0.4m square around node position
    // If so, we want to check for walls while being close to this point
    if (fabs(pos_y_int - pos_y) < 0.4 && fabs(pos_x_int - pos_x) < 0.4){
        // Check for walls in front:
        if (dist_mid < 0.8 && direction != -1){
            if (!hasWall(pos_x_int,pos_y_int, direction)){std::cout << "Found wall in front of current node" << std::endl;}
            setWall(pos_x_int, pos_y_int, direction);
        }
        else if (dist_mid < 1.5 && direction != -1){
            if (!hasWall(setpoint_x, setpoint_y, direction)){std::cout << "Found wall in front of setpoint node" << std::endl;}
            setWall(setpoint_x, setpoint_y, direction);
        }
    }

    // Check for walls adjacent to the node we are going to if we are close to the coordinate in between the setpoints
    if (fabs((setpoint_x*0.3 + setpoint_x_prev*0.7) - pos_x) < 0.05 && fabs((setpoint_y*0.3 + setpoint_y_prev*0.7) - pos_y) < 0.05){
        // Check for walls to the side of the node front of the current position

        if (dist_left < dist_side_max && direction != -1){

            // Wall left for the node in front
            int wall_direction = direction - 1; // Direction shifted counterclockwise since wall to left
            if (wall_direction < 0 ){wall_direction = 3;} // If we go from north(=0) to west(=3)
            if (!hasWall(setpoint_x, setpoint_y, wall_direction)){std::cout << "Found wall left" << std::endl;}
            setWall(setpoint_x, setpoint_y, wall_direction);
        }

        if (dist_right < dist_side_max && direction != -1){

            // Wall right for the node in front
            int wall_direction = direction + 1; // Direction shifted clockwise since wall to right
            if (wall_direction > 3 ){wall_direction = 0;} // If we go from west(=3) to north(=0)
            if (!hasWall(setpoint_x,setpoint_y, wall_direction)){std::cout << "Found wall right" << std::endl;}
            setWall(setpoint_x, setpoint_y, wall_direction);
        }
    }
}


void PathPlanner::updateFloodFillMap(){
    std::cout << "Updating flood fill map..." << std::endl;

    int pos_x_int = (int) round(pos_x);
    int pos_y_int = (int) round(pos_y);

    mat visited_flood_fill_map(9, 9, fill::zeros);
    mat is_queued(9, 9, fill::zeros);

    std::deque<pair<int,int> > flood_fill_queue;
    flood_fill_queue.push_back(std::make_pair(4,4));
    pair<int,int> node;
    node = flood_fill_queue.front();
    int distance_from_goal_to_node = 0;
    int nrVisited = 0;

    while (nrVisited < 81){ // While not all nodes are visited

        // Assign all nodes in the queue with their distance
        int queue_length = flood_fill_queue.size();
        int x_queue, y_queue;
        for (unsigned i=0; i < queue_length; i++){
            node = flood_fill_queue.at(i);
            //std::cout << "nrVisited: " << nrVisited << "\t Visiting (x,y)=(" << node.first << ", " << node.second;
            //std::cout << ") and assigning it the value " << distance_from_goal_to_node << std::endl;
            setFloodFillMapValue(node.first, node.second, distance_from_goal_to_node);
            visited_flood_fill_map(node.first, node.second) = 1; //visited
            nrVisited++;
        }

        // Add the neighbours off all newly visited nodes to the new queue
        for(unsigned j = 0; j < queue_length; j++){ //For each node in the queue
            node = flood_fill_queue.at(j);
            for (int i = 0; i < 4; i++){ // Check in all 4 directions for unvisited open space, and if so, add to queue
                if (!hasWall(node.first, node.second, i)){
                    //std::cout << "Open space in direction " << i << " to position (" << node.first << ", " << node.second << ")" << std::endl;
                    if (i == 0){x_queue = node.first;       y_queue = node.second + 1;}
                    if (i == 1){x_queue = node.first + 1;   y_queue = node.second;}
                    if (i == 2){x_queue = node.first;       y_queue = node.second - 1;}
                    if (i == 3){x_queue = node.first - 1;   y_queue = node.second;}

                    // If the cell is not visited before and is not already in the queue -> queue it!
                    if (visited_flood_fill_map(x_queue, y_queue) != 1 && is_queued(x_queue, y_queue) != 1){
                        flood_fill_queue.push_back(std::make_pair(x_queue, y_queue));
                        is_queued(x_queue, y_queue) = 1;
                    }
                }
            }
        }

        // Remove the visited nodes from the queue:
        for (int i = 0; i < queue_length; i++){
            flood_fill_queue.pop_front();
        }

        // Next iteration in breadth first search. Increase the distance
        distance_from_goal_to_node++;
        //std::cout << "Increase distance_from_goal_to_node to: " << distance_from_goal_to_node << std::endl;

        //printQueue(flood_fill_queue);
    }
}

double PathPlanner::getManhattanDistance(double x_pos, double y_pos){
    return sqrt((x_pos - 4) * (x_pos - 4) + (y_pos - 4) * (y_pos - 4));
}


void PathPlanner::spin(){
    ros::Rate loop_rate(1.0 / dt);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    PathPlanner *pp;
    PathPlanner temp_pp(nh);
    pp = &temp_pp;
    pp->spin();

    return 0;
}