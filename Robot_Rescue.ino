#include <Arduino.h>
#include <Servo.h>
#include "Nextion_Due.h"
#include "Wire.h"
#include "pins.h"
#include "constants.h"
#include "missions.h"

int sensors[8]     = {0};
int firstLineIndex = -1;
int lastLineIndex  = -1;
int targetIndex    =  3;
int amountSeen     = 0;

bool atWall;
bool turning = false;

char redPath[40];
char neutralPath[40];
char redPickup[40];
char neutralPickup[40];
int redSteps;
int neutralSteps;
int missionNum = 1;

int redIndex = 0;
int neutralIndex = 0;

Servo arm;
Servo dump;



Nextion myNextion;

void setup() {
  // initialize line sensors
  for(int i = 0; i < 8; i++)
  {
    pinMode(LINE_SENSOR[i], INPUT);
  }

  // initialize motor controllers
  pinMode(WHEEL_DIR_LB, OUTPUT);
  pinMode(WHEEL_DIR_LF, OUTPUT);
  pinMode(WHEEL_DIR_RB, OUTPUT);
  pinMode(WHEEL_DIR_RF, OUTPUT);

  pinMode(LEDY, OUTPUT);
  pinMode(LEDG, OUTPUT);

  pinMode(WHEEL_SPEED_L, OUTPUT);
  pinMode(WHEEL_SPEED_R, OUTPUT);

  pinMode(WHEEL_STBY, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  pinMode(FRONT_LEFT_SENSOR, INPUT);//changed
  pinMode(FRONT_RIGHT_SENSOR, INPUT);//changed
  pinMode(DUMPER, OUTPUT); //changed

  arm.write(ARM_MIDDLE);

  dump.attach(DUMPER);
  dump.write(DONT_DUMP);//initalize servo at closed position.

  writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
  digitalWrite(WHEEL_STBY  , HIGH);

  Serial.begin(9600);

  myNextion = Nextion(nexSerial, 9600);

  Serial3.begin(115200);
  Serial3.println("Starting Up...");

  //Serial.begin(9600);
  Wire.begin();
  delay(500);
  // MAKE SURE TO CHECK FOR NO YELLOW LED!
  
}

/*
  MANUEVERING:

The following functions relate to maneuvering the robot around the track, and
resulting states for accomplishing that task.

 */

/*
   Reads in the line sensor's current state, and stores them in the global
   variables above.
 */
void readLine() {
  amountSeen    = 0;
  lastLineIndex = -1;
  firstLineIndex = -1;
  for(int i = 7; i >= 0; --i) {
    sensors[i] = digitalRead(LINE_SENSOR[i]);
    if(sensors[i] == HIGH) {
      if(lastLineIndex == -1) {
        lastLineIndex = i;
      }
      ++amountSeen;
      firstLineIndex = i;
    }
  }
}

int readFrontRight(){//gets data val from right infraread sensor IMH
  return analogRead(FRONT_RIGHT_SENSOR);
}
int readFrontLeft(){//gets data val from left infraread sensor IMH
  return analogRead(FRONT_LEFT_SENSOR);
}

// TODO: get rid of this function
bool twoConsecutiveAtMiddle() {
  return twoConsecutive() && firstLineIndex >= TARGET_INDEX;
}


bool twoConsecutive() {
  int lowCount = 0;
  bool consecutive = false;
  for(int i = 0; i < 7; i++) {
    if(sensors[i] == HIGH && sensors[i + 1] == HIGH) {
      consecutive = true;
    }

    if(sensors[i] == LOW) {
      lowCount++;
    }
  }
  if(sensors[7] == LOW) {
    lowCount++;
  }
  return lowCount == 6 && consecutive;
}

/*
   New and improved wheel direction switch-er-oo. Use the constants FORWARDS and
   and BACKWARDS for this function. You could use true and false, but that would
   be very ape-ish of you.
 */
void writeWheelDirection(bool ldir, bool rdir) {
  digitalWrite(WHEEL_DIR_LF, ldir);
  digitalWrite(WHEEL_DIR_LB, !ldir);
  digitalWrite(WHEEL_DIR_RF, rdir);
  digitalWrite(WHEEL_DIR_RB, !rdir);
}

/*
   Write a direction to the wheels. If the wheel speed passed is negative, then
   the direction of that wheel is switched. Don't pass something over 255 or
   under 0.
 */
void writeToWheels(int ls, int rs) {
  writeWheelDirection(ls > 0, rs > 0); // Clean? Maybe more confusing?
  if(rs > 0) {
    digitalWrite(LEDG, HIGH);
  } else {
    digitalWrite(LEDG, LOW);
  }
  if(ls > 0) {
    digitalWrite(LEDY, HIGH);
  } else {
    digitalWrite(LEDY, LOW);
  }
  analogWrite(WHEEL_SPEED_L, abs(ls));
  analogWrite(WHEEL_SPEED_R, abs(rs));
}

/*
   Follows the line, returns true if the robot is at a fork.
 */
bool lineFollow(int ts, int strictness) {
  int offset = firstLineIndex - TARGET_INDEX;
  int rightSpeed = ts - offset * strictness;
  int leftSpeed = ts + offset * strictness;

  if(amountSeen == 0) {
    writeToWheels(HALF_SPEED, HALF_SPEED);
  } else {
    writeToWheels(leftSpeed % 255, rightSpeed % 255);
  }

  // Return true if the sensors can see a fork
  if(/*distance sensor sees something*/) {
    atWall = true;
    return true;
  }
  return (digitalRead(FORK_SENSOR) == HIGH)//amountSeen > TURN_AMOUNT;
}


/*
   Turns based on direction. Use the constants LEFT and RIGHT for this function.
   Returns true if the robot is back on the line.
 */
bool turn(int spd, char dir) {
  static int lineCount = 0;
  if(dir == LEFT){
    writeToWheels(-spd, spd);
  }else if(dir == RIGHT){
    writeToWheels(spd, -spd);
  }else if(dir == FORWARD) {
    writeToWheels(spd, spd);
  }
  
  if(dir == BACK && !atWall) {
    if(twoConsecutiveAtMiddle()) { // if it isn't at a wall, the line sensors have to pass another line to turn completely around
      lineCount++;
    }
    if(lineCount == 2){
      lineCount = 0;
      return true;
    }
  }
  // Return true if the robot is back centered on the line
  else return twoConsecutiveAtMiddle();
}

/*
   Delays for ms number of milliseconds.
 */
bool delayState(int ms) {
  static int milliseconds = -1;
  if(milliseconds == -1) {
    milliseconds = millis();
  }
  else if(millis() - milliseconds >= ms) {
    milliseconds = -1;
    return true;
  }
  return false;
}


bool doTurnSequence(const char sequence[], int index, int maxSteps) {
  if(turning) {
    if(turn(HALF_SPEED, sequence[index])) {
      turning = false;
      return true;
    }
  } else {
    if(index + 1 < maxSteps){
      if(sequence[index + 1] == FORWARD) {
        turning = lineFollow(FULL_SPEED, 50);
        return false;
      }
    }
    turning = lineFollow(HALF_SPEED, 50);
  }
  return false;
}



/*
   STATES
 */

bool pickMissionState() {
  writeToWheels(0, 0);
  
  // Write the line sensor data
  for(int i = 0; i < 8; i++) {
    myNextion.setComponentValue("c" + String(i), sensors[i]);
  }

  // Display the rear sensors
  myNextion.setComponentValue("rightNum", readFrontRight());
  myNextion.setComponentValue("leftNum", readFrontLeft());

  // Display the mission number
  myNextion.setComponentValue("missionNum", missionNum);
  myNextion.setComponentText("update", "READY");

  // Switch through the missions by pressing the buttons;
  if (digitalRead(BUTTON3) == LOW) {
    while(digitalRead(BUTTON3) == LOW) {}
    if(missionNum == 4) missionNum = 1;
    else missionNum++;
  }

  // When the start button is pushed, the mission paths are loaded and the state is exited
  if (digitalRead(BUTTON1) == LOW) {
    if(missionNum == 1) {
      strcpy(redPath, RED_MISSION_1);
      strcpy(neutralPath, NEUTRAL_MISSION_1);
      strcpy(redPickup, RED_PICKUP_1);
      strcpy(neutralPickup, NEUTRAL_PICKUP_1);
      redSteps = RED_STEPS_1;
      neutralSteps = NEUTRAL_STEPS_1;
    }
    /*else if(missionNum == 2) {
      strcpy(redPath, RED_MISSION_2);
      strcpy(neutralPath, NEUTRAL_MISSION_2);
      strcpy(redPickup, RED_PICKUP_2);
      strcpy(neutralPickup, NEUTRAL_PICKUP_2);
      redSteps = RED_STEPS_2;
      neutralSteps = NEUTRAL_STEPS_2;
    }
    else if(missionNum == 3) {
      strcpy(redPath, RED_MISSION_3);
      strcpy(neutralPath, NEUTRAL_MISSION_3);
      strcpy(redPickup, RED_PICKUP_3);
      strcpy(neutralPickup, NEUTRAL_PICKUP_3);
      redSteps = RED_STEPS_3;
      neutralSteps = NEUTRAL_STEPS_3;
    }
    if(missionNum == 4) {
      strcpy(redPath, RED_MISSION_4);
      strcpy(neutralPath, NEUTRAL_MISSION_4);
      strcpy(redPickup, RED_PICKUP_4);
      strcpy(neutralPickup, NEUTRAL_PICKUP_4);
      redSteps = RED_STEPS_4;
      neutralSteps = NEUTRAL_STEPS_4;
    }*/
    redIndex = 0;
    neutralIndex = 0;
    return true;
  }
}

bool followRedPathState() {
  if(redIndex == redSteps) return true;
  if(doTurnSequence(redPath, redIndex, redSteps) && doPickupSequence(redPickup, redIndex, redSteps)) redIndex++;
  
  return false;
}

bool followNeutralPathState() {

  if(neutralIndex == neutralSteps) return true;
  if(doTurnSequence(neutralPath, neutralIndex, neutralSteps) && doPickupSequence(neutralPickup, neutralIndex, neutralSteps)) neutralIndex++;
  
  return false;
}

bool doneState(){
  myNextion.setComponentText("update", "DONE");
  return (digitalRead(BUTTON2) == LOW);
}

bool depositPeopleState(){
  dump.write(DO_DUMP);
  if(delayState(2000)) {
    dump.write(DONT_DUMP);
    return true;
  }
  return false;
  
}


void loop() {
  static int state = 0;
  readLine();
  switch(state)
  {
    case 0:
      if(pickMissionState())  state++;
      break;
    case 1:
      if(followRedPathState())  state++;
      break;
    case 2:
      if(depositPeopleState())  state++;
      break;
    case 3:
      if(followNeutralPathState()) state++;
      break;
    case 4:
      if(depositPeopleState()) state++;
      break;
  default:
      if(doneState()) state = 0;
      break;
  }

}
      

