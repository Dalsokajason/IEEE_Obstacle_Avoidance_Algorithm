#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <ctime>
#include <thread>
#include <chrono>

using namespace std;

// Struct to store path coordinates
struct Point {
    int row, col;
};

// Structure to hold data for the priority queue for the A* algorithm
struct AStarNode {
    int row, col;
    int f_score;

    // A heap that will store lowest score at top, so we need to define the comparison operator
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

//declarations so code works
void generateRandomObstacle(vector<vector<int>>& mapMatrix);
Point averageObstaclePosition(vector<vector<int>>& mapMatrix);
void searchState(vector<vector<int>>& mapMatrix);

int main(){

    int rows = 78;
    int cols = 78;
    
    //this seeds the random number generator with current time
    srand(static_cast<unsigned int>(time(nullptr)));

    //double distance = min_distance + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_distance - min_distance));
    //double angle = min_angle + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_angle - min_angle));

    // For testing purposes, we will use fixed values for distance and angle
    //double distance = 10;
    //double angle = 30;
    //for sumo my idea is to continue to use an occupancy grid to track the position of the opponent bot. Of course whatever we detect will not be the full
    //representation of the opponent bot, but it will be a good enough representation to track the opponent bot and make decisions based on that. The occupancy grid will be updated with the position of the opponent bot as we detect it. The occupancy grid will be used to make decisions on how to move the robot to avoid the opponent bot and to try to push the opponent bot out of the ring. The occupancy grid will also be used to make decisions on how to move the robot to try to push the opponent bot out of the ring. The occupancy grid will also be used to make decisions on how to move the robot to try to push the opponent bot out of the ring.

    vector<vector<int>> mapMatrix(rows, vector<int>(cols, 0));
    mapMatrix[rows/2][0] = 2; // Start point
    mapMatrix[rows/2][cols - 1] = 2; // Goal
    
    for(int i = 0; i < 150; i++){
        //generateRandomObstacle(mapMatrix);
        //cout << "Obstacle " << i + 1 << " generated." << endl;
    }

    searchState(mapMatrix);

    //set the start and goal points for the A* algorithm the point is structured as {row, col}
    //Point start = {rows/2, 0};
    //Point goal = {rows/2, cols - 1};

    //vector<Point> path = a_star_occupancy_grid(mapMatrix, start, goal);

    //print out the path found by the A* algorithm NOT USING
    //if (path.empty()) {
        //cout << "No path found!" << endl;
    //} else {
        //cout << "Path found: " << endl;
        //for (const auto& pt : path) {
            //mapMatrix[pt.row][pt.col] = 7; // Mark path on the map as 7s
            //cout << "(" << pt.row << ", " << pt.col << ") -> ";
            //wait for 1 second before printing the next point
            //std::this_thread::sleep_for(std::chrono::seconds(1));
        //}
        //cout << "Goal" << endl;
    //}

    //print out the matrix
    for(int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            std::cout << mapMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    

    return 0;
}

void generateRandomObstacle(vector<vector<int>>& mapMatrix, int min_distance, int max_distance, int min_angle, int max_angle) {
    int rows = mapMatrix.size();
    int cols = mapMatrix[0].size();
    //double min_distance = 20.0;
    //double max_distance = 40.0;
    //double min_angle = -15.0;
    //double max_angle = 15.0;
    double distance = min_distance + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_distance - min_distance));
    double angle = min_angle + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_angle - min_angle));
    double pi = 3.14159265358979323846;
    double angle_rad = angle * (pi / 180.0);
    double x = distance * cos(angle_rad);
    double y = distance * sin(angle_rad);
    
    //cout << "x = " << x << ", y = " << y << endl;
    // Obstacle position is set to x and y coord, but -1 on each to account for 0 indexing in the matrix
    mapMatrix[static_cast<int>(rows/2 + (round(y) - 1))][static_cast<int>(round(x) - 1)] = 1; // 1 denotes an obstacle
}

//Euclidean distance heuristic
double euclidean_heuristic(int r1, int c1, int r2, int c2) {
    return std::sqrt(std::pow(r1 - r2, 2) + std::pow(c1 - c2, 2));
}

// Main A* Search algorithm for an occupancy grid
void searchState(vector<vector<int>>& mapMatrix) {
    int rows = mapMatrix.size();
    int cols = mapMatrix[0].size();

    //calculate the average position of the obstacles in the mapMatrix
    //Search State
    for(int i = 0; i < 150; i++){
        generateRandomObstacle(mapMatrix, 20, 30, 5, 35);
    }
    Point prev_avgObstaclePos = averageObstaclePosition(mapMatrix);
    cout << "Previous average obstacle position: (" << prev_avgObstaclePos.row << ", " << prev_avgObstaclePos.col << ")" << endl;
    //wait for 0.5 seconds before calculating the average position of the obstacles again
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    srand(static_cast<unsigned int>(time(nullptr)));
    for(int i = 0; i < 150; i++){
        generateRandomObstacle(mapMatrix, 20, 30, -35, -5);
    }
    Point curr_avgObstaclePos = averageObstaclePosition(mapMatrix);
    cout << "Current average obstacle position: (" << curr_avgObstaclePos.row << ", " << curr_avgObstaclePos.col << ")" << endl;

    //calculate the difference between the previous average position and the current average position
    int deltaRow = curr_avgObstaclePos.row - prev_avgObstaclePos.row;
    int deltaCol = curr_avgObstaclePos.col - prev_avgObstaclePos.col;
    //vector<Point> opponentMovement = {prev_avgObstaclePos, curr_avgObstaclePos};

    //create a vector perpendicular to the opponentMovement vector
    Point perpendicularVector = {2 * deltaCol, -2 * deltaRow};
    cout << "Perpendicular vector: (" << perpendicularVector.row << ", " << perpendicularVector.col << ")" << endl;
    //create a point at the perpendicular vector from the current average position of the obstacles
    Point targetPoint = {curr_avgObstaclePos.row + perpendicularVector.row, curr_avgObstaclePos.col + perpendicularVector.col};
    cout << "Target point: (" << targetPoint.row << ", " << targetPoint.col << ")" << endl;
    //mark the target point on the mapMatrix as 3
    if (targetPoint.row >= 0 && targetPoint.row < rows && targetPoint.col >= 0 && targetPoint.col < cols) {
            mapMatrix[targetPoint.row][targetPoint.col] = 3; // 3 denotes the target point
        }
    //get the magnitude and direction of the target point from the start point (rows/2, 0)
    int startRow = rows / 2;
    int startCol = 0;
    int deltaTargetRow = targetPoint.row - startRow;
    int deltaTargetCol = targetPoint.col - startCol;
    double magnitude = std::sqrt(std::pow(deltaTargetRow, 2) + std::pow(deltaTargetCol, 2));
    double direction = std::atan2(deltaTargetRow, deltaTargetCol) * (180.0 / 3.14159265358979323846);
    cout << "Magnitude: " << magnitude << ", Direction: " << direction << endl;
    
}

Point averageObstaclePosition(vector<vector<int>>& mapMatrix) {
    int rows = mapMatrix.size();
    if (rows == 0) return {0, 0};
    int cols = mapMatrix[0].size();

    int obstacleCount = 0;
    int obstacleRowSum = 0;
    int obstacleColSum = 0;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (mapMatrix[r][c] == 1) { // 1 denotes an obstacle
                obstacleCount++;
                obstacleRowSum += r;
                obstacleColSum += c;
            }
        }
    }

    if (obstacleCount == 0) {
        return {0, 0}; // No obstacles found
    }

    return {obstacleRowSum / obstacleCount, obstacleColSum / obstacleCount};
}

class Robot {
    private:
        Point position;
        double orientation; // in degrees

    public:
        Robot(Point startPos, double startOrientation) {
            //start position will be manually given for now, but will be fed from the imu in the future
            position = startPos;
            //we will feed this info from the imu
            orientation = startOrientation;
        }
    
    void stepTo(Point newPos) {
        
    }

};
