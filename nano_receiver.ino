#include <Servo.h>
Servo servoRight;
Servo servoLeft;
byte CMDstart;

int RIGHT_STOP_US = 1500;
int LEFT_STOP_US  = 1500;

int RIGHT_FWD_US = 1300;
int LEFT_FWD_US  = 1700;

void driveFor(int rUs, int lUs, int tMs) {
  digitalWrite(13, HIGH);
  servoRight.writeMicroseconds(rUs);
  servoLeft.writeMicroseconds(lUs);
  delay(tMs);
  servoRight.writeMicroseconds(RIGHT_STOP_US);
  servoLeft.writeMicroseconds(LEFT_STOP_US);
  digitalWrite(13, LOW);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(200);
  servoRight.attach(5);
  servoLeft.attach(6);
  pinMode(13, OUTPUT);
  servoRight.writeMicroseconds(RIGHT_STOP_US);
  servoLeft.writeMicroseconds(LEFT_STOP_US);
  delay(50);
}

void loop() {
  while (Serial.available() > 0) {
    delay(1);
    CMDstart = Serial.read();

    if (CMDstart == 'f') {
      int t = Serial.parseInt();
      driveFor(RIGHT_FWD_US, LEFT_FWD_US, t);
    }
    if (CMDstart == 'l') {
      int t = Serial.parseInt();
      driveFor(1700, 1700, t);
    }
    if (CMDstart == 'r') {
      int t = Serial.parseInt();
      driveFor(1300, 1300, t);
    }
    if (CMDstart == 'b') {
      int t = Serial.parseInt();
      driveFor(1700, 1300, t);
    }
    if (CMDstart == 's') {
      int t = Serial.parseInt();
      servoRight.writeMicroseconds(RIGHT_STOP_US);
      servoLeft.writeMicroseconds(LEFT_STOP_US);
      delay(t);
    }
  }
}