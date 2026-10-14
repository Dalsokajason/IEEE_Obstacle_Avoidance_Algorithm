#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>

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


void loop() {
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
}
