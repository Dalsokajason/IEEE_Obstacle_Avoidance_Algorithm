//Class for MotorMovement 

class MotorMovement{
  public: int speed;
   float distance; 
   float angle;

  //odd motors are left 
  void leftSideForward(){
    digitalWrite(m1forwardPin, HIGH);
    digitalWrite(m1backitupPin, LOW);
    digitalWrite(m3forwardPin, HIGH);
    digitalWrite(m3backitupPin, LOW);
  }
  void leftSideBackward(){
    digitalWrite(m1forwardPin, LOW);
    digitalWrite(m1backitupPin, HIGH);
    digitalWrite(m3forwardPin, LOW);
    digitalWrite(m3backitupPin, HIGH);
  }
  //even motors are right
  void rightSideForward(){
    digitalWrite(m2forwardPin,HIGH);
    digitalWrite(m2backitupPin, LOW);
    digitalWrite(m4forwardPin,HIGH);
    digitalWrite(m4backitupPin, LOW);
  }
  void rightSideBackward(){
    digitalWrite(m2forwardPin,LOW);
    digitalWrite(m2backitupPin, HIGH);
    digitalWrite(m4forwardPin,LOW);
    digitalWrite(m4backitupPin, HIGH);
  }
  void moveForward(int speed){
    leftSideForward();
    rightSideForward();
  }
  void moveBackward(int speed){
    leftSideBackward();
    rightSideBackward();
  }

  void turnToLeft(int speed){
  //want the rightmost tires to go forward and leftmost to go back 
    rightSideForward();
    leftSideBackward();
  }
  void turnToRight(int speed){
    //to turn to the right, leftmost motors go forward and rightmost go backward
    rightSideBackward();
    leftSideForward();
  }

  void stopMoving(){
    digitalWrite(m1forwardPin, LOW);
    digitalWrite(m1backitupPin, LOW);
    digitalWrite(m2forwardPin, LOW);
    digitalWrite(m2backitupPin, LOW);
    digitalWrite(m3forwardPin, LOW);
    digitalWrite(m3backitupPin, LOW);
    digitalWrite(m4forwardPin, LOW);
    digitalWrite(m4backitupPin, LOW);
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

};
