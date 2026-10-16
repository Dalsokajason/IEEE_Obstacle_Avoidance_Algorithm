//assign the pins later 
const int IN1 = 0;
const int IN2 = 0;
const int EN_A = 0;//PWM

const int IN3 = 0;
const int IN4 = 0;
const int EN_B = 0;//PWM

const int IN5 = 0;
const int IN6 = 0;
const int EN_C = 0;//PWM

const int IN7 = 0;
const int IN8 = 0;
const int EN_D = 0;//PWM

void setup() {
  //motor1 LEFT
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(EN_A,OUTPUT);
  //motor2 RIGHT
  pinMode(IN4,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(EN_B,OUTPUT);
 //motor3 LEFT
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(EN_C, OUTPUT);
  //motor4 RIGHT
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(EN_D, OUTPUT);
}


//Class for MotorMovement 

class MotorMovement{
  public: int speed;
   float distance; 
   float angle;

  //odd motors are left 
  void leftSideForward(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  }
  void leftSideBackward(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }
  //even motors are right
  void rightSideForward(){
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN7,HIGH);
    digitalWrite(IN8, LOW);
  }
  void rightSideBackward(){
    digitalWrite(IN3,LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN7,LOW);
    digitalWrite(IN8, HIGH);
  }
  void allmotorspeed(int speed){
    //speed = constrain(speed, 0, 225);
    analogWrite(EN_A,speed);
    analogWrite(EN_B,speed);
    analogWrite(EN_C,speed);
    analogWrite(EN_D,speed);
  }
  void moveForward(int speed){
    leftSideForward();
    rightSideForward();
    allmotorspeed(speed);
  }
  void moveBackward(int speed){
    leftSideBackward();
    rightSideBackward();
    allmotorspeed(speed);
  }

  void turnToLeft(int speed){
  //want the rightmost tires to go forward and leftmost to go back 
    rightSideForward();
    leftSideBackward();
    allmotorspeed(speed);
  }
  void turnToRight(int speed){
    //to turn to the right, leftmost motors go forward and rightmost go backward
    rightSideBackward();
    leftSideForward();
    allmotorspeed(speed);
  }

  void brake(){
    allmotorspeed(0);//does not write anything 
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
  }
  void navigate(float distance, float angle,int speed){
    if (angle == 0){
      moveForward(speed);
    } 
    else if (angle > 0){
      turnToRight(speed);
    }
    else if (angle < 0){
      turnToLeft(speed);
    }
  }
  void testingMotors(){
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
};

MotorMovement motor;
void loop() {
  motor.testingMotors();
}
