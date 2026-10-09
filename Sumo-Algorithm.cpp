#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <ctime>

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
int heuristic(int r1, int c1, int r2, int c2);
vector<Point> a_star_occupancy_grid(vector<vector<int>>& mapMatrix, Point start, Point goal);

int main(){

    int rows = 50;
    int cols = 50;
    
    srand(static_cast<unsigned int>(time(nullptr)));

    //double distance = min_distance + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_distance - min_distance));
    //double angle = min_angle + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_angle - min_angle));

    // For testing purposes, we will use fixed values for distance and angle
    //double distance = 10;
    //double angle = 30;
    vector<vector<int>> mapMatrix(rows, vector<int>(cols, 0));
    mapMatrix[rows/2][0] = 2; // Start point
    mapMatrix[rows/2][cols - 1] = 2; // Goal
    
    for(int i = 0; i < 150; i++){
        generateRandomObstacle(mapMatrix);
        cout << "Obstacle " << i + 1 << " generated." << endl;
    }

    //set the start and goal points for the A* algorithm the point is structured as {row, col}
    Point start = {rows/2, 0};
    Point goal = {rows/2, cols - 1};

    vector<Point> path = a_star_occupancy_grid(mapMatrix, start, goal);

    //print out the path found by the A* algorithm
    if (path.empty()) {
        cout << "No path found!" << endl;
    } else {
        cout << "Path found: " << endl;
        for (const auto& pt : path) {
            mapMatrix[pt.row][pt.col] = 7; // Mark path on the map as 7s
            cout << "(" << pt.row << ", " << pt.col << ") -> ";
            //wait for 1 second before printing the next point
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        cout << "Goal" << endl;
    }

    //print out the matrix
    for(int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            std::cout << mapMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    

    return 0;
}

void generateRandomObstacle(vector<vector<int>>& mapMatrix) {
    int rows = mapMatrix.size();
    int cols = mapMatrix[0].size();
    double min_distance = 20.0;
    double max_distance = 35.0;
    double min_angle = -15.0;
    double max_angle = 15.0;
    double distance = min_distance + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_distance - min_distance));
    double angle = min_angle + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_angle - min_angle));
    double pi = 3.14159265358979323846;
    double angle_rad = angle * (pi / 180.0);
    double x = distance * cos(angle_rad);
    double y = distance * sin(angle_rad);
    
    cout << "x = " << x << ", y = " << y << endl;
    // Obstacle position is set to x and y coord, but -1 on each to account for 0 indexing in the matrix
    mapMatrix[static_cast<int>(rows/2 + (round(y) - 1))][static_cast<int>(round(x) - 1)] = 1; // 1 denotes an obstacle
}

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

// Manhattan distance heuristic
int heuristic(int r1, int c1, int r2, int c2) {
    return std::abs(r1 - r2) + std::abs(c1 - c2);
}

//Euclidean distance heuristic
double euclidean_heuristic(int r1, int c1, int r2, int c2) {
    return std::sqrt(std::pow(r1 - r2, 2) + std::pow(c1 - c2, 2));
}

// Main A* Search algorithm for an occupancy grid
std::vector<Point> a_star_occupancy_grid(vector<vector<int>>& mapMatrix, Point start, Point goal) {
    int rows = mapMatrix.size();
    if (rows == 0) return {};
    int cols = mapMatrix[0].size();

    // Priority Queue using std::greater to sort smallest f_score first
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> frontier;
    frontier.push({start.row, start.col, 0});

    // 1D flat vectors mapped by (row * cols + col) for O(1) lookups
    std::vector<int> cost_so_far(rows * cols, -1); 
    std::vector<int> came_from(rows * cols, -1);

    // Initialize starting node costs
    int start_idx = start.row * cols + start.col;
    cost_so_far[start_idx] = 0;

    // 4-Directional movements: Up, Down, Left, Right
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    bool path_found = false;

    while (!frontier.empty()) {
        AStarNode current = frontier.top();
        frontier.pop();

        // Goal reached early exit
        if (current.row == goal.row && current.col == goal.col) {
            path_found = true;
            break;
        }

        int current_idx = current.row * cols + current.col;

        // Skip processed paths if a cheaper one was already recorded
        if (current.f_score - heuristic(current.row, current.col, goal.row, goal.col) > cost_so_far[current_idx]) {
            continue;
        }

        // Check all 4 neighbors
        for (int i = 0; i < 4; ++i) {
            int next_r = current.row + dr[i];
            int next_c = current.col + dc[i];

            // 1. Boundary Check
            if (next_r >= 0 && next_r < rows && next_c >= 0 && next_c < cols) {
                // 2. Collision Check (1 denotes obstacle)
                if (mapMatrix[next_r][next_c] == 1) {
                    continue;
                }

                int next_idx = next_r * cols + next_c;
                int new_cost = cost_so_far[current_idx] + 1; // Step weight of 1

                // 3. Evaluation Check
                if (cost_so_far[next_idx] == -1 || new_cost < cost_so_far[next_idx]) {
                    cost_so_far[next_idx] = new_cost;
                    int f_score = new_cost + euclidean_heuristic(next_r, next_c, goal.row, goal.col);
                    
                    frontier.push({next_r, next_c, f_score});
                    came_from[next_idx] = current_idx;
                }
            }
        }
    }

    // Path Reconstruction
    std::vector<Point> path;
    if (!path_found) return path; // Return empty vector if path is blocked

    int curr_idx = goal.row * cols + goal.col;
    while (curr_idx != -1) {
        path.push_back({curr_idx / cols, curr_idx % cols});
        curr_idx = came_from[curr_idx];
    }
    std::reverse(path.begin(), path.end());
    return path;
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
        //the robot will move to the new position but will need to reorient itself first based on the angle to the new position and then move forward to the new position
        // Implementation for reorientation and movement
        orientation = atan2(newPos.row - position.row, newPos.col - position.col) * (180.0 / 3.14159265358979323846);
        //move + 1 in the direction of the new position.
        position = point{position.row + (newPos.row - position.row) / std::max(1, std::abs(newPos.row - position.row)), 
                         position.col + (newPos.col - position.col) / std::max(1, std::abs(newPos.col - position.col))};
    }

}
