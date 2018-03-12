#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>
#include <queue>
#include <armadillo>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace arma;
enum {OPEN = 0, WALL = 1};
enum {UNDEFINED = -1, NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};
enum {GOAL_NOT_REACHED = 4, GOAL_REACHED = 5};

#define PI 3.14159265
#define THRESHOLD_SWITCH 0.0025
#define dt 0.1
#define MAX_ANGLE_DEVIATION 3.0
#define GRID_SIZE 9
#define GOAL_X 4
#define GOAL_Y 4
#define FRONT_WALL_DETECTION_LIMIT_THIS_CELL 0.8
#define FRONT_OPEN_DETECTION_LIMIT_THIS_CELL 1.0
#define FRONT_WALL_DETECTION_LIMIT_NEXT_CELL 1.5
#define FRONT_OPEN_DETECTION_LIMIT_NEXT_CELL 1.9
#define SIDE_WALL_DETECTION_ZONE 0.05
#define FRONT_WALL_DETECTION_ZONE 0.4

class PathPlanner{
private:
    ros::Subscriber sub_pos;
    ros::Subscriber sub_dist;
    ros::Publisher pub_point;
    ros::Publisher pub_x_y_yaw;

    int pos_x_int_last, pos_y_int_last, setpoint_x, setpoint_y, setpoint_x_prev, setpoint_y_prev, goal_reached;
    double pos_x, pos_y, ang_z, dist_mid, dist_left, dist_right;
    double dist_side_max;
    cube wall_map;

    //Flood fill
    mat flood_fill_map;
    vec neighbour_flood_fill_values;// = zeros<vec>(4);
    int min_heading;

public:
    PathPlanner(ros::NodeHandle &nh);
    void callbackOdom( const nav_msgs::OdometryConstPtr& poseMsg);
    void callbackDistances( const std_msgs::Float32MultiArray& distMsg);
    void initializeWallMap();
    void spin();
    void setWall(int x_pos, int y_pos, int direction);
    void removeWall(int x_pos, int y_pos, int direction);
    bool hasWall(int x_pos, int y_pos, int direction);
    void checkForWalls();

    //Flood fill
    void initializeFloodFillMap();
    void setFloodFillMapValue(int x_pos, int y_pos, int value);
    int getFloodFillMapValue(int x_pos, int y_pos);
    void setNextDestinationCell();
    void updateFloodFillMap();
    double getManhattanDistance(double x_pos, double y_pos);
    void printFloodFillMapWithWalls();
};


PathPlanner::PathPlanner(ros::NodeHandle &nh){
    // Initializing wall map:
    initializeWallMap();
    initializeFloodFillMap();
    pos_x_int_last = UNDEFINED;
    pos_y_int_last = UNDEFINED;

    dist_mid = 3;
    dist_right = 3;
    dist_left = 3;
    setpoint_x = 0;
    setpoint_y = 0;
    dist_side_max = 1.1;
    neighbour_flood_fill_values.zeros(4);
    goal_reached = GOAL_NOT_REACHED;

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

        if (!(setpoint_x == GOAL_X && setpoint_y == GOAL_Y)){
            updateFloodFillMap();
            setNextDestinationCell();
        }
        else{
            if (goal_reached == GOAL_NOT_REACHED){
                std::cout << "Goal reached!" << std::endl;
            }
            goal_reached = GOAL_REACHED;

        }
    }

    // If the bot detects a new wall in front as it is heading towards the new setpoint,
    // this node can not be reached, and we need to set a new setpoint. By
    if (hasWall(setpoint_x_prev, setpoint_y_prev, min_heading)){
        std::cout << "The setpoint could not be reached after all! Re-planning a new set point..." << std::endl;
        setpoint_x = setpoint_x_prev;
        setpoint_y = setpoint_y_prev;
        updateFloodFillMap();
        setNextDestinationCell(); // Gets a new set point based on the new wall-information
    }

    cmd_setpoint.x = setpoint_x;
    cmd_setpoint.y = setpoint_y;
    cmd_setpoint.z = goal_reached;

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
        double alpha = (1.2 - 0.80) / (0.29 - 0.19);
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
    wall_map = cube(GRID_SIZE, GRID_SIZE, 4, fill::zeros);

    // Fill the information which is already known into the map, i.e outer boundaries has a wall.
    // (row,col)=(0,0) is bottom left
    // (row,col)=(0,8) is bottom right
    // (row,col)=(8,8) is top right
    // (row,col)=(8,0) is top left

    for (int row = 0; row < GRID_SIZE; row++){
        for (int col = 0; col < GRID_SIZE; col++){
            if (row == 0){
                wall_map(row, col, SOUTH) = WALL; // wall to the south
            }
            if (row == (GRID_SIZE - 1)){
                wall_map(row, col, NORTH) = WALL; // wall to the north
            }
            if (col == 0){
                wall_map(row, col, WEST) = WALL; // wall to the west
            }
            if (col == (GRID_SIZE - 1)){
                wall_map(row, col, EAST) = WALL; // wall to the east
            }
        }
    }

    std::cout << "Wall map initialized!" << std::endl;
}

void PathPlanner::setNextDestinationCell() {
    std::cout << "Switching set point..." << std::endl;

    // 1.) Detect my current cell
    int pos_x_int = (int) round(pos_x);
    int pos_y_int = (int) round(pos_y);
    double manhattan_distance = 1000;

    // 2.) Update the stack with all visited locations
    neighbour_flood_fill_values(NORTH) = getFloodFillMapValue(pos_x_int, pos_y_int + 1);
    neighbour_flood_fill_values(EAST) = getFloodFillMapValue(pos_x_int + 1, pos_y_int);
    neighbour_flood_fill_values(SOUTH) = getFloodFillMapValue(pos_x_int, pos_y_int - 1);
    neighbour_flood_fill_values(WEST) = getFloodFillMapValue(pos_x_int - 1, pos_y_int);

    // Find the neighbouring cell without a wall in between with the lowest flood fill value
    int minimumReachableFloodFillValue = 10000;
    for (int i = NORTH; i <= WEST; i++){
        if (!hasWall(pos_x_int, pos_y_int, i)){
            //Evaluate if this is the next destination
            double x_next, y_next;
            if (i == NORTH){x_next = pos_x_int;       y_next = pos_y_int + 1;}
            if (i == EAST){x_next = pos_x_int + 1;   y_next = pos_y_int;}
            if (i == SOUTH){x_next = pos_x_int;       y_next = pos_y_int - 1;}
            if (i == WEST){x_next = pos_x_int - 1;   y_next = pos_y_int;}

            int neighbourCellFloodFillValue = neighbour_flood_fill_values(i);
            if (neighbourCellFloodFillValue < minimumReachableFloodFillValue){
                minimumReachableFloodFillValue = neighbourCellFloodFillValue;
                min_heading = i;
                manhattan_distance = getManhattanDistance(x_next, y_next);
            }
            else if (neighbourCellFloodFillValue == minimumReachableFloodFillValue){
                // Choose the heading which minimizes (pos_x - pos_x_goal)^2 + (pos_y - pos_y_goal)^2
                double manhattan_distance_new = getManhattanDistance(x_next, y_next);
                if (manhattan_distance_new < manhattan_distance){
                    min_heading = i;
                    manhattan_distance = manhattan_distance_new;
                    std::cout << "Choosing heading " << i << " since it minimizes Manhattan distance" << std::endl;
                }
            }
        }
    }

    std::cout << "Current position: (" << pos_x_int << ", " << pos_y_int << ")" << std::endl;
    std::cout << "Best heading: " << min_heading << std::endl;

    //New setpoint
    if (min_heading == NORTH) {
        setpoint_y++;
    } else if (min_heading == EAST) {
        setpoint_x++;
    } else if (min_heading == SOUTH) {
        setpoint_y--;
    } else {
        setpoint_x--;
    }
    std::cout << "Setpoint: (" << setpoint_x << ", " << setpoint_y << ")" << std::endl;

    //Print flood fill
    std::cout << "Flood fill map: " << std::endl;
    //flood_fill_map.print();
    printFloodFillMapWithWalls();
    std::cout << std::endl;

    pos_x_int_last = pos_x_int;
    pos_y_int_last = pos_y_int;

}

void PathPlanner::initializeFloodFillMap(){
    flood_fill_map.zeros(GRID_SIZE, GRID_SIZE);
    updateFloodFillMap();
    std::cout << "Flood fill map initialized!" << std::endl;
    std::cout <<  " Printed: " << std::endl;
    //flood_fill_map.print();
    printFloodFillMapWithWalls();
}


void PathPlanner::setFloodFillMapValue(int x_pos, int y_pos, int value){
    flood_fill_map(GRID_SIZE - 1 - y_pos, x_pos) = value;
}

int PathPlanner::getFloodFillMapValue(int x_pos, int y_pos){

    if (x_pos < 0 || y_pos < 0 || x_pos > (GRID_SIZE - 1) || y_pos > (GRID_SIZE - 1)){
        return 1000;}
    else{
    return flood_fill_map(GRID_SIZE - 1 - y_pos, x_pos);}
}


void PathPlanner::setWall(int x_pos, int y_pos, int direction){
    // direction is a number = {0=North, 1=East, 2=South, 3=west}
    // (row,col)=(0,0) is bottom left
    // (row,col)=(0,8) is bottom right
    // (row,col)=(8,8) is top right
    // (row,col)=(8,0) is top left

    if (!hasWall(x_pos, y_pos, direction)) {
        // Detected a wall we did not know about
        wall_map(y_pos, x_pos, direction) = WALL;

        if (direction == NORTH) { //North wall. Fill south wall to adjecent node
            wall_map(y_pos + 1, x_pos, SOUTH) = WALL;
            std::cout << "Wall detected north for position (x,y) = (";
        } else if (direction == EAST) { //East wall. Fill west wall to adjecent node
            wall_map(y_pos, x_pos + 1, WEST) = WALL;
            std::cout << "Wall detected east for position (x,y) = (";
        } else if (direction == SOUTH) { //South wall. Fill north wall to adjecent node
            wall_map(y_pos - 1, x_pos, NORTH) = WALL;
            std::cout << "Wall detected south for position (x,y) = (";
        } else if (direction == WEST) { //West wall. Fill east wall to adjecent node
            wall_map(y_pos, x_pos - 1, EAST) = WALL;
            std::cout << "Wall detected west for position (x,y) = (";
        }

        std::cout << x_pos << ", " << y_pos << ")" << std::endl;
    }
}

void PathPlanner::removeWall(int x_pos, int y_pos, int direction){
    // direction is a number = {0=North, 1=East, 2=South, 3=west}
    // (row,col)=(0,0) is bottom left
    // (row,col)=(0,8) is bottom right
    // (row,col)=(8,8) is top right
    // (row,col)=(8,0) is top left

    if (hasWall(x_pos, y_pos, direction)) {
        // Remove a wall where it should not be one
        wall_map(y_pos, x_pos, direction) = OPEN;

        if (direction == NORTH) { //North wall. Fill south wall to adjecent node
            wall_map(y_pos + 1, x_pos, SOUTH) = OPEN;
            std::cout << "Wall removed north for position (x,y) = (";
        } else if (direction == EAST) { //East wall. Fill west wall to adjecent node
            wall_map(y_pos, x_pos + 1, WEST) = OPEN;
            std::cout << "Wall removed east for position (x,y) = (";
        } else if (direction == SOUTH) { //South wall. Fill north wall to adjecent node
            wall_map(y_pos - 1, x_pos, NORTH) = OPEN;
            std::cout << "Wall removed south for position (x,y) = (";
        } else if (direction == WEST) { //West wall. Fill east wall to adjecent node
            wall_map(y_pos, x_pos - 1, EAST) = OPEN;
            std::cout << "Wall removed west for position (x,y) = (";
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
    int pos_x_int_next = pos_x_int;
    int pos_y_int_next = pos_y_int;
    int direction = UNDEFINED;
    double ang_z_deg = ang_z * 180.0 / PI;

    if (ang_z_deg > 0 && ang_z_deg < MAX_ANGLE_DEVIATION){
        direction = NORTH;
        pos_y_int_next++;
    }
    else if (ang_z_deg < 0 && ang_z_deg > -MAX_ANGLE_DEVIATION){
        direction = NORTH;
        pos_y_int_next++;
    }
    else if (ang_z_deg > (90 - MAX_ANGLE_DEVIATION) && ang_z_deg < (90 + MAX_ANGLE_DEVIATION)){
        direction = EAST;
        pos_x_int_next++;
    }
    else if (ang_z_deg > (180 - MAX_ANGLE_DEVIATION) || ang_z_deg < (-180 + MAX_ANGLE_DEVIATION)){
        direction = SOUTH;
        pos_y_int_next--;
    }
    else if (ang_z_deg < -90 + MAX_ANGLE_DEVIATION && ang_z_deg > (-90 - MAX_ANGLE_DEVIATION)){
        direction = WEST;
        pos_x_int_next--;
    }


    // Check if x and y coordinate is close to a node position. Close = within 0.4m x 0.4m square around node position
    // If so, we want to check for walls while being close to this point
    if (    fabs(pos_y_int - pos_y) < FRONT_WALL_DETECTION_ZONE &&
            fabs(pos_x_int - pos_x) < FRONT_WALL_DETECTION_ZONE &&
            direction != UNDEFINED){
        // Check for walls in front:
        if (dist_mid < FRONT_WALL_DETECTION_LIMIT_THIS_CELL){
            if (!hasWall(pos_x_int,pos_y_int, direction)){
                std::cout << "Found wall in front of current node" << std::endl;
            }
            setWall(pos_x_int, pos_y_int, direction);
        }
        else if(dist_mid > FRONT_OPEN_DETECTION_LIMIT_THIS_CELL){
            if (hasWall(pos_x_int,pos_y_int, direction)){
                std::cout << "The wall in front of current node was wrong! Removing wall from wall map" << std::endl;
            }
            removeWall(pos_x_int, pos_y_int, direction);
        }

        if (dist_mid > FRONT_OPEN_DETECTION_LIMIT_THIS_CELL &&
            dist_mid < FRONT_WALL_DETECTION_LIMIT_NEXT_CELL){
            if (!hasWall(pos_x_int_next, pos_y_int_next, direction)){
                std::cout << "Found wall in front of set point node" << std::endl;
            }
            setWall(pos_x_int_next, pos_y_int_next, direction);
        }
        else if (dist_mid > FRONT_OPEN_DETECTION_LIMIT_NEXT_CELL){
            if (hasWall(pos_x_int_next, pos_y_int_next, direction)){
                std::cout << "The wall in front of set point node was wrong! Removing wall from wall map" << std::endl;
            }
            removeWall(pos_x_int_next, pos_y_int_next, direction);
        }
    }

    // Check for walls adjacent to the node we are going to if we are close to the coordinate in between the setpoints
    if (    fabs((setpoint_x*0.3 + setpoint_x_prev*0.7) - pos_x) < SIDE_WALL_DETECTION_ZONE &&
            fabs((setpoint_y*0.3 + setpoint_y_prev*0.7) - pos_y) < SIDE_WALL_DETECTION_ZONE){
        // Check for walls to the side of the node front of the current position

        if (dist_left < dist_side_max && direction != UNDEFINED){

            // Wall left for the node in front
            int wall_direction = direction - 1; // Direction shifted counterclockwise since wall to left
            if (wall_direction < NORTH ){wall_direction = WEST;} // If we go from north(=0) to west(=3)
            if (!hasWall(pos_x_int_next, pos_y_int_next, wall_direction)){std::cout << "Found wall left" << std::endl;}
            setWall(pos_x_int_next, pos_y_int_next, wall_direction);
        }

        if (dist_right < dist_side_max && direction != UNDEFINED){

            // Wall right for the node in front
            int wall_direction = direction + 1; // Direction shifted clockwise since wall to right
            if (wall_direction > WEST ){wall_direction = NORTH;} // If we go from west(=3) to north(=0)
            if (!hasWall(pos_x_int_next, pos_y_int_next, wall_direction)){std::cout << "Found wall right" << std::endl;}
            setWall(pos_x_int_next, pos_y_int_next, wall_direction);
        }
    }
}


void PathPlanner::updateFloodFillMap(){
    std::cout << "Updating flood fill map..." << std::endl;

    mat visited_flood_fill_map(GRID_SIZE, GRID_SIZE, fill::zeros);
    mat is_queued(GRID_SIZE, GRID_SIZE, fill::zeros);

    std::deque<pair<int,int> > flood_fill_queue;
    flood_fill_queue.push_back(std::make_pair(GOAL_X, GOAL_Y));
    pair<int,int> node;
    node = flood_fill_queue.front();
    int distance_from_goal_to_node = 0;
    int nrVisited = 0;
    int total_nr_of_nodes = GRID_SIZE * GRID_SIZE;

    while (nrVisited < total_nr_of_nodes){ // While not all nodes are visited

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
            for (int i = NORTH; i <= WEST; i++){ // Check in all 4 directions for unvisited open space, and if so, add to queue
                if (!hasWall(node.first, node.second, i)){
                    //std::cout << "Open space in direction " << i << " to position (" << node.first << ", " << node.second << ")" << std::endl;
                    if (i == NORTH){x_queue = node.first;       y_queue = node.second + 1;}
                    if (i == EAST){x_queue = node.first + 1;   y_queue = node.second;}
                    if (i == SOUTH){x_queue = node.first;       y_queue = node.second - 1;}
                    if (i == WEST){x_queue = node.first - 1;   y_queue = node.second;}

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

        if (distance_from_goal_to_node > total_nr_of_nodes){
            // Some detected walls must be wrong, or there is a node in the map which cant be reached.
            // Hence the while loop goes forever. Need to reset the map.
            std::cout << "Flood fill unable to fill all nodes. Must be non-reachable nodes or" <<
                      "falsely detected walls. Resets the map" << std::endl;
            initializeWallMap();
            flood_fill_map.zeros();
            visited_flood_fill_map.zeros();
            is_queued.zeros();
            flood_fill_queue.clear();
            flood_fill_queue.push_back(std::make_pair(GOAL_X, GOAL_Y));
            node = flood_fill_queue.front();
            distance_from_goal_to_node = 0;
            nrVisited = 0;
        }
    }
}

double PathPlanner::getManhattanDistance(double x_pos, double y_pos){
    return sqrt((x_pos - GOAL_X) * (x_pos - GOAL_X) + (y_pos - GOAL_Y) * (y_pos - GOAL_Y));
}

void PathPlanner::printFloodFillMapWithWalls() {
    // create north wall:
    std::cout << "+";
    for (int k = 0; k < (GRID_SIZE - 1); k++){
        std::cout << "----+";
    }
    std::cout << "----+" << std::endl;

    for (int y = GRID_SIZE - 1; y >= 0; y--){
        for(int j = 0; j < 2; j++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                if (j == 0) { // Print walls west and east
                    int value = getFloodFillMapValue(x, y);
                    if (x == 0) { // Check for wall west for x = 0
                        std::cout << "| " << std::setw(2) << value;
                        if (hasWall(x, y, EAST)){
                            std::cout << " | ";
                        }
                        else{
                            std::cout << "   ";
                        }
                    }
                    else { // x > 0
                        std::cout << std::setw(2) << value;
                        if (hasWall(x, y, EAST)) {
                            std::cout << " | ";
                        }
                        else{
                            std::cout << "   ";
                        }
                    }
                    if(x == (GRID_SIZE - 1)){
                        std::cout << std::endl;
                    }
                }
                else if (j == 1){ // j==1, Print walls south
                    if(x == 0){
                        std::cout << "+";
                    }
                    if(hasWall(x,y,SOUTH)){
                        std::cout << "----+";
                    }
                    else{ // No wall
                        std::cout << "    +";
                    }
                    if(x == (GRID_SIZE - 1)){
                        std::cout << std::endl;
                    }
                }
            }
        }
    }
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