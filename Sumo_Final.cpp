//Blue Cheese Hmmm Yum yum num
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <ctime>
#include <thread>
#include <chrono>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>


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
void searchState(vector<vector<int>>& mapMatrix, Point targetPoint);
void flankState(vector<vector<int>>& mapMatrix, Point targetPoint);
void combatState(vector<vector<int>>& mapMatrix);
void alignToOpponent(vector<vector<int>>& mapMatrix);
void scanMap(vector<vector<int>>& mapMatrix);

const int rows = 78;
const int cols = 78;

//assign the pins later
const int IN1 = 27;
const int IN2 = 26;
const int EN_A = 14;  //PWM

const int IN3 = 33;
const int IN4 = 32;
const int EN_B = 25;  //PWM

const int IN5 = 0;
const int IN6 = 0;
const int EN_C = 0;  //PWM

const int IN7 = 0;
const int IN8 = 0;
const int EN_D = 0;  //PWM

//TOF
Adafruit_VL53L0X tof;
Servo scanServo;
const int SERVO_PIN = 18;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int LEFT_ANGLE = 40;
const int RIGHT_ANGLE = 140;
int currentAngle = 90;
int direction = 1;
const int STEP_SIZE = 5;

//CHANGE THE DISTANCE DETECT OPPONENT HEREEE
const int DETECT_DISTANCE = 600; // mm

// controls how often servo moves
unsigned long lastServoMove = 0;
const int SERVO_INTERVAL = 15;   // ms

void setup() {
  //motor1 LEFT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN_A, OUTPUT);
  //motor2 RIGHT
  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(EN_B, OUTPUT);
  //motor3 LEFT
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(EN_C, OUTPUT);
  //motor4 RIGHT
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(EN_D, OUTPUT);

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  if (!tof.begin()) {
    Serial.println("ERROR: VL53L0X not found!");
    while (1);
  }
  Serial.println("VL53L0X READY");
  // start continuous distance measurements
  if (!tof.startRangeContinuous(30)) {
    Serial.println("ERROR: Could not start continuous ranging!");
    while (1);
  }

  scanServo.setPeriodHertz(50);

  scanServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  scanServo.write(currentAngle);

  delay(500);

  Serial.println("SERVO READY");
  Serial.println("START SCANNING...");
}


//Class for MotorMovement

class MotorMovement {
public:
  int speed;
  float distance;
  float angle;

  //odd motors are left
  void leftSideForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  }
  void leftSideBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }
  //even motors are right
  void rightSideForward() {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN7, HIGH);
    digitalWrite(IN8, LOW);
  }
  void rightSideBackward() {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
  }
  void allmotorspeed(int speed) {
    //speed = constrain(speed, 0, 225);
    analogWrite(EN_A, speed);
    analogWrite(EN_B, speed);
    analogWrite(EN_C, speed);
    analogWrite(EN_D, speed);
  }
  void moveForward(int speed) {
    leftSideForward();
    rightSideForward();
    allmotorspeed(speed);
  }
  void moveBackward(int speed) {
    leftSideBackward();
    rightSideBackward();
    allmotorspeed(speed);
  }

  void turnToLeft(int speed) {
    //want the rightmost tires to go forward and leftmost to go back
    rightSideForward();
    leftSideBackward();
    allmotorspeed(speed);
  }
  void turnToRight(int speed) {
    //to turn to the right, leftmost motors go forward and rightmost go backward
    rightSideBackward();
    leftSideForward();
    allmotorspeed(speed);
  }

  void brake() {
    allmotorspeed(0);  //does not write anything
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
  }
  void navigate(float distance, float angle, int speed) {
    if (angle == 0) {
      moveForward(speed);
    } else if (angle > 0) {
      turnToRight(speed);
    } else if (angle < 0) {
      turnToLeft(speed);
    }
  }
   void testingMotors() {
    moveForward(200);
    delay(500);

    brake();
    delay(500);

    turnToRight(180);
    delay(500);

    brake();
    delay(500);

    turnToLeft(180);
    delay(500);

    brake();
    delay(200);

    moveBackward(150);
    delay(500);

    brake();
    delay(500);
  } // Closes testingMotors()

}; // Closes MotorMovement class

MotorMovement motor; // jarvis, create my object

void loop() {
  //MOTORS
  motor.testingMotors();

  //TOF
  if (millis() - lastServoMove >= SERVO_INTERVAL) {

    lastServoMove = millis();

    currentAngle += STEP_SIZE * direction;

    if (currentAngle >= RIGHT_ANGLE) {
      currentAngle = RIGHT_ANGLE;
      direction = -1;
    }

    if (currentAngle <= LEFT_ANGLE) {
      currentAngle = LEFT_ANGLE;
      direction = 1;
    }

    scanServo.write(currentAngle);
  }
  
  if (tof.isRangeComplete()) {

    int distance = tof.readRange();

    Serial.print("Angle: ");
    Serial.print(currentAngle);

    Serial.print(" deg | Distance: ");

    if (distance > 2000) {
      Serial.println("OUT OF RANGE");
    } else {

      Serial.print(distance);
      Serial.print(" mm");

      if (distance < DETECT_DISTANCE) {
        Serial.print("   <<< OPPONENT");
      }

      Serial.println();
    }
  }

  //ALGORITHM
  Point targetPoint = {0, 0}; // jarvis, initialize targetPoint
  
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
  
  searchState(mapMatrix, targetPoint); //spend some time looking for the 
  //delay(1000);
  flankState(mapMatrix, targetPoint);
  //delay(1000);
  combatState(mapMatrix);
  //delay(1000);
    
  //print out the matrix
  for(int i = 0; i < rows; i++){
      for (int j = 0; j < cols; j++){
          std::cout << mapMatrix[i][j] << " ";
      }
      std::cout << std::endl;
  }
}

void generateRandomObstacle(vector<vector<int>>& mapMatrix, int min_distance, int max_distance, int min_angle, int max_angle) {
    int rows = mapMatrix.size();
    int cols = mapMatrix[0].size();

    double distance = min_distance + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_distance - min_distance));
    double angle = min_angle + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) / (max_angle - min_angle));
    double pi = 3.14159265358979323846;
    double angle_rad = angle * (pi / 180.0);
    double x = distance * cos(angle_rad);
    double y = distance * sin(angle_rad);

    mapMatrix[static_cast<int>(rows/2 + (round(y) - 1))][static_cast<int>(round(x) - 1)] = 1; // 1 denotes an obstacle
}

//Euclidean distance heuristic
double euclidean_heuristic(int r1, int c1, int r2, int c2) {
    return std::sqrt(std::pow(r1 - r2, 2) + std::pow(c1 - c2, 2));
}

//jarvis,
void searchState(vector<vector<int>>& mapMatrix, Point targetPoint) {
    int startRow = rows / 2;
    int startCol = 0;
    while(targetPoint == {0, 0}) {
        int rows = mapMatrix.size();
        int cols = mapMatrix[0].size();
        for(int i = 0; i < 150; i++){
            //generateRandomObstacle(mapMatrix, 35, 40, 5, 35); //simulating scanninng the previous position of the opponent bot.
        }
        //Point prev_avgObstaclePos = averageObstaclePosition(mapMatrix);
        //Just for testing purposes, me will use fixed values for the previous average position of the obstacles
        Point prev_avgObstaclePos = {rows/2, 1}; // Initialize to the start point for testing
        mapMatrix[prev_avgObstaclePos.row][prev_avgObstaclePos.col] = 4; // 4 denotes the average position of the obstacles
        //cout << "Previous average obstacle position: (" << prev_avgObstaclePos.row << ", " << prev_avgObstaclePos.col << ")" << endl;
    
        //make sure to clear the mapMatrix of previous obstacles and opponent bot positions before generating new ones
        for (int i = 0; i < mapMatrix.size(); ++i) {
            for (int j = 0; j < mapMatrix[0].size(); ++j) {
                if (mapMatrix[i][j] == 1) { // 1 denotes an obstacle
                    mapMatrix[i][j] = 0; // Clear previous obstacles and target point
                }
            }
        }
    
        //NVM, NOT USING DELAY FOR OPPONENT MOVEMENT VECTOR wait for 0.5 seconds before calculating the average position of the obstacles again
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        srand(static_cast<unsigned int>(time(nullptr)));
        for(int i = 0; i < 150; i++){
            generateRandomObstacle(mapMatrix, 25, 30, -35, -5); //simulating scanning the current position of the opponent bot.
        }
        Point curr_avgObstaclePos = averageObstaclePosition(mapMatrix);
        mapMatrix[curr_avgObstaclePos.row][curr_avgObstaclePos.col] = 4; // 4 denotes the current average position of the obstacles
        cout << "Current average obstacle position: (" << curr_avgObstaclePos.row << ", " << curr_avgObstaclePos.col << ")" << endl;
    
        //calculate the difference between the previous average position and the current average position
        //int deltaRow = curr_avgObstaclePos.row - prev_avgObstaclePos.row;
        //int deltaCol = curr_avgObstaclePos.col - prev_avgObstaclePos.col;
        int deltaRow = curr_avgObstaclePos.row - startRow;
        int detlaCol = curr_avgObstaclePos.col - startCol;
        cout << "delta movement vector: (" << deltaRow << ", " << deltaCol << ")" << endl;
    
        //create a vector perpendicular to the opponentMovement vector
        Point perpendicularVector = {1 * deltaCol, -1 * deltaRow};
        cout << "Perpendicular vector: (" << perpendicularVector.row << ", " << perpendicularVector.col << ")" << endl;
        //create a point at the perpendicular vector from the current average position of the obstacles
        targetPoint = {curr_avgObstaclePos.row + perpendicularVector.row, curr_avgObstaclePos.col + perpendicularVector.col};
        cout << "Target point: (" << targetPoint.row << ", " << targetPoint.col << ")" << endl;
        //mark the target point on the mapMatrix as 3
        if (targetPoint.row >= 0 && targetPoint.row < rows && targetPoint.col >= 0 && targetPoint.col < cols) {
                mapMatrix[targetPoint.row][targetPoint.col] = 3; // 3 denotes the target point
        }
        
        //get the magnitude and direction of the target point from the start point (rows/2, 0)
        int deltaTargetRow = targetPoint.row - startRow;
        int deltaTargetCol = targetPoint.col - startCol;
        double magnitude = std::sqrt(std::pow(deltaTargetRow, 2) + std::pow(deltaTargetCol, 2));
        double direction = std::atan2(deltaTargetRow, deltaTargetCol) * (180.0 / 3.14159265358979323846);
        cout << "Target point" << " magnitude: " << magnitude << ", direction: " << direction << " degrees" << endl;
    }
}

void flankState(vector<vector<int>>& mapMatrix, Point targetPoint) {
    int rows = mapMatrix.size();
    int cols = mapMatrix[0].size();

    //get the magnitude and direction of the target point from the start point (rows/2, 0)
    int startRow = rows / 2;
    int startCol = 0;
    int deltaTargetRow = targetPoint.row - startRow;
    int deltaTargetCol = targetPoint.col - startCol;
    double magnitude = std::sqrt(std::pow(deltaTargetRow, 2) + std::pow(deltaTargetCol, 2));
    double direction = std::atan2(deltaTargetRow, deltaTargetCol) * (180.0 / 3.14159265358979323846);
    
    //turn the robot to face the target point
    //angleDist() is Haydens function that outputs the angle of the robot from origin
    //while (angleDist() != direction) {
        //if (angleDist() < direction) {
            //turnToRight(100);
        //} else {
            //turnToLeft(100);
        //}
    //}

    //make robot move forward to the target point
    //cartDist is Haydens function that outputs the distance of the robot from origin
    //while (cartDist() < magnitude) {
    //    moveForward(100);
    //}

    //turn the robot 180 degrees to face the opponent bot
    //while (angleDist() != direction + 180) {
        //if (angleDist() < direction + 180) {
            //turnToRight(100);
        //} else {
            //turnToLeft(100);
        //}
    //}
}

void combatState(vector<vector<int>>& mapMatrix) {
    int minAngle = -15;
    int maxAngle = 15;
    //the robot is now presummably facing the opponent bot and is at its side. This function will make the robot move forward to push the opponent bot out of the ring.
    
    //run this function every 1 second for as long as the robot is not more than 70 cm away from the origin which is found by using the cartDist() function. This is to make sure that the robot does not go out of bounds of the ring.
    //if robot is too far move backwards

    //while (cartDist() < 70) {
        //move forward at 100 speed for 1 second
        //moveForward(100);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //stop the robot
        //stopMoving();
        //scan the map to update the positions of the obstacles and the opponent bot
        //scanMap(mapMatrix);
        //align the robot to face the opponent bot's average obstacle position  
        //alignToOpponent(mapMatrix);
    //}
    //while (cartDist() >= 70) {
        //move backwards at 100 speed for 2 second
        //moveBackward(100);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        //stop the robot
        //stopMoving();
        //scan the map to update the positions of the obstacles and the opponent bot
        //scanMap(mapMatrix);
        //align the robot to face the opponent bot's average obstacle position  
        //alignToOpponent(mapMatrix);
    //}



}

void alignToOpponent(vector<vector<int>>& mapMatrix) {
    // This function will align the robot to face the opponent bot's average obstacle position.
    //we need to rescan the map to update the positions of the obstacles and the opponent bot
    scanMap(mapMatrix);

    Point avgObstaclePos = averageObstaclePosition(mapMatrix);
    if (avgObstaclePos.row != 0 || avgObstaclePos.col != 0) {
        // Calculate the angle to the average obstacle position
        int deltaRow = avgObstaclePos.row - (mapMatrix.size() / 2); // Assuming bot is at center row
        int deltaCol = avgObstaclePos.col - 0; // Assuming bot is at column 0
        double angleToOpponent = std::atan2(deltaRow, deltaCol) * (180.0 / 3.14159265358979323846);

        // Adjust orientation to face the opponent bot
        //while (angleDist() != angleToOpponent) {
            //if (angleDist() < angleToOpponent) {
                //turnToRight(100);
            //} else {
                //turnToLeft(100);
            //}
        //}
    }
}

void scanMap(vector<vector<int>>& mapMatrix) {
    //this function will scan the map and update the positions of the obstacles and the opponent bot
    //for now we will just generate random obstacles to simulate the scanning of the map

    //make sure to clear the mapMatrix of previous obstacles and opponent bot positions before generating new ones
    for (int i = 0; i < mapMatrix.size(); ++i) {
        for (int j = 0; j < mapMatrix[0].size(); ++j) {
            if (mapMatrix[i][j] == 1 || mapMatrix[i][j] == 4 || mapMatrix[i][j] == 3) { // 1 denotes an obstacle, 4 denotes the average position of the obstacles, 3 denotes the target point
                mapMatrix[i][j] = 0; // Clear previous obstacles and target point
            }
        }
    }

    //TOF FUNCTION HERE

    for(int i = 0; i < 150; i++){
        generateRandomObstacle(mapMatrix, 10, 40, -35, 35);
    }
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
