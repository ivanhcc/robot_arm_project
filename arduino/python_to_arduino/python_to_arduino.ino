#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <InverseK.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Degree <-> Radian conversions
float d2r(float b) {
  return b / 180.0 * PI - HALF_PI;
}
float r2d(float a) {
  return (a + HALF_PI) * 180 / PI;
}

// Constants
#define SERVO_MIN  102
#define SERVO_MAX  528
#define NUM_SERVOS 5
#define FREQUENCY  50

int servoPins[NUM_SERVOS] = {0, 1, 2, 3, 4};
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 165};

// Convert angle to pulse width
int pulseWidth(double angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

// Smoothly move a servo to a target angle
void smoothTurn(int servo_number, double targetAngle, int stepDelay = 10) {
  targetAngle = constrain(targetAngle, 0, 180);
  int current = currentAngles[servo_number];

  if (targetAngle > current) {
    for (int i = current; i <= targetAngle; i++) {
      pwm.setPWM(servoPins[servo_number], 0, pulseWidth(i));
      delay(stepDelay);
    }
  } else {
    for (int i = current; i >= targetAngle; i--) {
      pwm.setPWM(servoPins[servo_number], 0, pulseWidth(i));
      delay(stepDelay);
    }
  }

  currentAngles[servo_number] = targetAngle;
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(10);

  // Initialize robot arm dimensions
  Link base, upperarm, forearm, hand;
  base.init(0, d2r(0.0), d2r(180.0));
  upperarm.init(90, d2r(0.0), d2r(180.0));
  forearm.init(70, d2r(0.0), d2r(180.0));
  hand.init(66, d2r(0.0), d2r(180.0));
  InverseK.attach(base, upperarm, forearm, hand);

  homeposition();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();  // Remove \r and extra spaces

    if (cmd == "HOME") {
      homeposition();
      Serial.println("Moved to HOME");
    }

    else if (cmd.startsWith("MOVE")) {
      // Manual split instead of sscanf
      cmd.trim();
      int firstSpace = cmd.indexOf(' ');
      if (firstSpace != -1) {
        String params = cmd.substring(firstSpace + 1);
        float x, y, z;
        int sep1 = params.indexOf(' ');
        int sep2 = params.indexOf(' ', sep1 + 1);
        if (sep1 > 0 && sep2 > sep1) {
          x = params.substring(0, sep1).toFloat();
          y = params.substring(sep1 + 1, sep2).toFloat();
          z = params.substring(sep2 + 1).toFloat();
          movetopos(x, y, z);
          Serial.println("Moved to position");
        } else {
          Serial.println("Invalid MOVE format");
        }
      } else {
        Serial.println("Invalid MOVE command");
      }
    }

    else if (cmd.startsWith("GRIPPER")) {
      int x;
      int space = cmd.indexOf(' ');
      if (space > 0) {
        String val = cmd.substring(space + 1);
        x = val.toInt();
        gripper(x);
        Serial.println("Gripper command executed");
      } else {
        Serial.println("Invalid GRIPPER command");
      }
    }

    else {
      Serial.println("Unknown command");
    }
  }
}

// Move to home position
void homeposition() {
  float a0, a1, a2, a3;
  InverseK.solve(0, 0, 226, a0, a1, a2, a3);
  smoothTurn(0, r2d(a0) - 2);  // 2 degree offset
  smoothTurn(1, r2d(a1));
  smoothTurn(2, r2d(a2));
  smoothTurn(3, r2d(a3));
}

// Move to 3D position
void movetopos(float x, float y, float z) {
  float a0, a1, a2, a3;
  if (InverseK.solve(x, y, z, a0, a1, a2, a3)) {
    smoothTurn(0, r2d(a0) - 2);  // 2 degree offset
    smoothTurn(1, r2d(a1));
    smoothTurn(2, r2d(a2));
    smoothTurn(3, r2d(a3));
  } else {
    Serial.println("No solution found!");
  }
}

// Control gripper
void gripper(int x) {
  if (x == 0) {
    smoothTurn(4, 165);  // close
  } else {
    smoothTurn(4, 60);   // open
  }
}
