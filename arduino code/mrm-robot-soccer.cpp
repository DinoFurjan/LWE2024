#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
//#include <mrm-ir-finder2.h>
#include <mrm-ir-finder3.h>
#include "mrm-robot-soccer.h"
const int MAX_TURN_RATE = 127;
const int MTR_PWR = 60;
int state = 0;
int motspwr = 0;
int calspd = 30;

const int LIN_SPD = 95;
const int LIN_OVR = 130;
const int LIN_OVR_STRF = 100;
const bool useLine = true;

const int marin = 69;

int sm;

const int a16 = 32;
const int a32 = 33;
const int S0 = 26;
const int S1 = 25;
const int S2 = 14;
const int S3 = 27;


int blk[32];
int whit[32];
int calc[32];
int ircl[32];

int err = 0;



/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotSoccer::RobotSoccer(char name[])
  : Robot(name) {
  motorGroup = new MotorGroupStar(this, mrm_bldc2x50, 0, mrm_bldc2x50, 1, mrm_bldc2x50, 2, mrm_bldc2x50, 3);

  // LED signs to be assigned to different actions follow. It is easier to follow action flow by checking the display.



  // LED Bounce

  // Actions
  pidXY = new Mrm_pid(0.5, 100, 0);        // PID controller, regulates motors' speeds for linear motion in the x-y plane: 4, 100, 0 - ok.
  pidRotation = new Mrm_pid(1.8, 90, 0);  // PID controller, regulates rotation around z axis
  pidKompass = new Mrm_pid(1.2, 90, 0);
  actionlineDebug = new ActionSoccerlineDebug(this);
  actionlineCal = new ActionSoccerlineCal(this);

  // The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
  // right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
  // called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right.
  // This test is not supposed to be called in code.
  mrm_bldc2x50->directionChange(0);  // Uncomment to change 1st wheel's rotation direction
  mrm_bldc2x50->directionChange(1);  // Uncomment to change 2nd wheel's rotation direction
  mrm_bldc2x50->directionChange(2);  // Uncomment to change 3rd wheel's rotation direction
  
  actionAdd(actionlineDebug);
  actionAdd(actionlineCal);

  // Buttons
  mrm_8x8a->actionSet(_actionLoop, 2);   // Button 3 starts user defined loop() function
  mrm_8x8a->actionSet(actionlineCal, 1);
  mrm_8x8a->actionSet(actionlineDebug, 0);
  mrm_8x8a->actionSet(_actionMenuMain, 3);  // Button 4 stops the robot and prints main menu
  // Set number of phototransistors in each line sensor.
  //mrm_ref_can->transistorCountSet(5, 0); // 5 instead of 6 since IR ball interferes with 6th transistor.
  //mrm_ref_can->transistorCountSet(8, 1);
  //mrm_ref_can->transistorCountSet(8, 2);
  //mrm_ref_can->transistorCountSet(8, 3);
}

/** Rear distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/


/** Ball's direction
@return - robot's front is 0�, positive angles clockwise, negative anti-clockwise. Back of the robot is 180�.
*/
int16_t RobotSoccer::ballAngle() {
  return mrm_ir_finder3->angle();
}

/** Read barrier
@return - true if interrupted
*/

/** Test barrier
*/
void RobotSoccer::barrierTest() {
  print("%i - %s ball\n\r", analogRead(SOCCER_BARRIER_PIN), barrier() ? "" : "no");
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
  //uint8_t red[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
  //uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
  //mrm_8x8a->bitmapCustomStore(red, green, 7);
}

/** Bouncing off the lines
*/

/** Reads push button switch
@number - 0 to 3, push button's ordinal number
@return - true if pressed
*/
bool RobotSoccer::button(uint8_t number) {
  return mrm_8x8a->switchRead(number);
}

/** Line sensor - brightness of the surface
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - brightness as an analog value.
*/
uint16_t RobotSoccer::brightness(uint8_t transistorNumber, uint8_t deviceNumber) {
  return mrm_ref_can->reading(transistorNumber, deviceNumber);
}

/** Calibrate all line sensors
 */
/*void RobotSoccer::calibrate() {
  go(0, 0, 25, 100);
  mrm_ref_can->calibrate(0);
  mrm_ref_can->calibrate(1);
  mrm_ref_can->calibrate(2);
  mrm_ref_can->calibrate(3);
  go(0, 0, 0, 0);
  end();
}
*/
/** Go around the ball and approach it.
*/


/** Front distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/


/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
@param speed - 0 to 100.
@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
Values between -180 and 180.
@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
numbers because a value 100 turns on all the motors at maximal speed.
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/


/** Test - go straight ahead.
*/


/** Approach oppoent's goal
*/

/**Compass
@return - North is 0�, clockwise are positive angles, values 0 - 360.
*/
float RobotSoccer::heading() {
  return mrm_imu->heading();
}

float RobotSoccer::headingRandom(int heading, int variation) {
  float newHeading = heading + (2 * (rand() % variation) - variation);
  if (newHeading > 180)
    newHeading -= 360;
  else if (newHeading < -180)
    newHeading += 360;
  return newHeading;
}

/** No ball detected - return to Your goal.
*/


/** Left distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm
*/

/** Line sensor
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if white line found
*/
void RobotSoccer::calCheck() {
  int dn = 0;
  int error = 0;
  int motspwr = 0;
  for (int i=0; i<32; i++) {
    calc[i]=0;
  }
  while (dn == 0) {
    error = heading() - headingToMaintain;
    if (error > 180) {
      error = error - 360;
    } 
    else if (error < -180) {
      error = error + 360;
    }
    motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
    odi(calspd, 90, motspwr/2);
    for (int i=0; i<32; i++) {
      if (lineCheck(i) == 1){
        calc[i] = 1;
        if (i == 0 && lineCheck(i) == 1) {
          dn = 1;
        }
      }
    }
  }
  dn = 0;
  for (int i = 0; i<32; i++) {
    if (calc[i] == 0) {
      err = 1;
      break;
    }
  }
  end();
}

void RobotSoccer::lineCal() {
  int dn = 0;
  int error = 0;
  int motspwr = 0;

  int t;
  int overshoot = 50; //overshoot ms
  if (setup()) {
    delay(200);    
    for (int i; i<32; i++){
      blk[i] = lineRead(i);
      whit[i] = lineRead(i) + 500;
    }
    dn = 0;
    headingToMaintain = heading();
    err = 0;
  }
  uint8_t linlr[8] = { 0b00000000, 0b00000000, 0b00000000, 0b11000000, 0b11000000, 0b00000000, 0b00000000, 0b00000000 };
  uint8_t ling[8] =  { 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000 };
  uint8_t linrr[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000011, 0b00000011, 0b00000000, 0b00000000, 0b00000000 };
  uint8_t clean[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
  uint8_t check[8] = { 0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b10100000, 0b01000000, 0b00000000 };
  uint8_t cross[8] = { 0b10000001, 0b01000010, 0b00100100, 0b00011000, 0b00011000, 0b00100100, 0b01000010, 0b10000001 };
  Serial.println("Calibrating Line sensors...");
  mrm_8x8a->bitmapCustomDisplay(linlr, ling);
  delay(1000);
  while (dn == 0) {
    error = heading() - headingToMaintain;
    if (error > 180) {
      error = error - 360;
    } 
    else if (error < -180) {
      error = error + 360;
    }
    motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
    odi(calspd, -90, motspwr/2);
    for (int i = 0; i<32; i++) {
      if (lineRead(i) < blk[i]){
        blk[i]=lineRead(i);
      }
      if (lineRead(i) > whit[i]){
        whit[i]=lineRead(i);
        if (i == 16) {
          dn = 1;
        }
      }
    }
  }
  t = overshoot;
  while (t > 0) {
    error = heading() - headingToMaintain;
    if (error > 180) {
      error = error - 360;
    } 
    else if (error < -180) {
      error = error + 360;
    }
    motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
    odi(calspd, -90, motspwr/2);
    delay(1);
    t--;
  }

  odi(10,0,0);
  mrm_8x8a->bitmapCustomDisplay(clean, check);
  delay(1000);
  Serial.println("Readings Complete, Calculating line values...");
  for (int i = 0; i<32; i++) {
    ircl[i] = (whit[i]+blk[i])/2 + 100;
  }
  delay(1000);
  mrm_8x8a->bitmapCustomDisplay(clean, check);
  Serial.println("Checking Calibration...");
  delay(1000);
  mrm_8x8a->bitmapCustomDisplay(linrr, ling);
  calCheck();
  if (err == 0) {
    mrm_8x8a->bitmapCustomDisplay(clean, check);
    Serial.println("Calibration Succesfull!");
  }
  else {
    mrm_8x8a->bitmapCustomDisplay(cross, clean);
    Serial.println("Calibration Failed!!");
  }
  odi(0,0,0);
  delay(2000); 
  for (int i; i<32; i++){
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(ircl[i]);
    Serial.print(" ,");
  }
  end();
}

int RobotSoccer::lineCheck(int x) {
  int c;
  c = ircl[x];
  if (lineRead(x) > c) {
    return(1);
  }
  else {
    return(0);
  }
}

int RobotSoccer::lineRead(int n) {
  int b1, b2, b3, b4;
  int delay = 300;
  if (n<16) {
    generateBinary(n, b4, b3, b2, b1);
    digitalWrite(S0, b1);
    digitalWrite(S1, b2);
    digitalWrite(S2, b3);
    digitalWrite(S3, b4);
    delayMicroseconds(delay);
    return(analogRead(a16));
  }
  else{
    generateBinary(n-16, b4, b3, b2, b1);
    digitalWrite(S0, b1);
    digitalWrite(S1, b2);
    digitalWrite(S2, b3);
    digitalWrite(S3, b4);
    delayMicroseconds(delay);
    return(analogRead(a32));
  }
}

void RobotSoccer::generateBinary(int num, int &a, int &b, int &c, int &d) {
  d = num % 2;
  num /= 2;
  c = num % 2;
  num /= 2;
  b = num % 2;
  num /= 2;
  a = num % 2;
}

void RobotSoccer::lineDebug() {
  int lin;
  /*for (int i = 0; i<32; i++) {
    Serial.print(i);
    Serial.print("-");
    Serial.print(lineCheck(i));
    Serial.print(",  ");
    Serial.print(lineRead(i));
    Serial.print(",  ");
    Serial.print(ircl[i]);
    Serial.print(",  ");
    Serial.print("  ");
  }*/
  
  LineLogic(lin);
  Serial.print("Angle: ");
  Serial.print(lin);
  Serial.println();
}

void RobotSoccer::odi(float speed, float direction_degrees, float rotation_speed) {
  if (direction_degrees >= 0) {
    direction_degrees = 360.0 - fmod(direction_degrees, 180.0);
  }
  else {
    direction_degrees = map(fmod(direction_degrees + 180.0, 180.0), 0, 180, 180, 0);
  }

  if (direction_degrees < 90) {
    mrm_bldc2x50->speedSet(0, constrain(map(direction_degrees, 0, 90, 0, speed) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 270) {
    mrm_bldc2x50->speedSet(0, constrain(map(direction_degrees, 270, 360, -speed, 0) + rotation_speed, -127, 127));
  }
  if (direction_degrees >= 90 && direction_degrees <=270) {
    mrm_bldc2x50->speedSet(0, constrain(map(direction_degrees, 90, 270, speed, -speed) + rotation_speed, -127, 127));
  }
  if (direction_degrees <= 60) {
    mrm_bldc2x50->speedSet(1, constrain(map(direction_degrees, 0, 60, speed*0.6, 0) + rotation_speed, -127, 127));
  }
  if (direction_degrees <= 360 && direction_degrees > 330) {
    mrm_bldc2x50->speedSet(1, constrain(map(direction_degrees, 330, 360, speed, speed*0.6) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 60 && direction_degrees <= 150) {
    mrm_bldc2x50->speedSet(1, constrain(map(direction_degrees, 60, 150, 0, -speed) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 150 && direction_degrees <= 240){
    mrm_bldc2x50->speedSet(1, constrain(map(direction_degrees, 150 , 240, -speed, 0) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 240 && direction_degrees <= 330){
    mrm_bldc2x50->speedSet(1, constrain(map(direction_degrees, 240, 330, 0, speed) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 300) {
    mrm_bldc2x50->speedSet(2, constrain(map(direction_degrees, 300 , 360, 0, -speed*0.6) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 240 && direction_degrees < 300) {
    mrm_bldc2x50->speedSet(2, constrain(map(direction_degrees, 240, 300 , speed, 0) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 120 && direction_degrees < 240) {
    mrm_bldc2x50->speedSet(2, constrain(map(direction_degrees, 120, 240, 0, speed) + rotation_speed, -127, 127));
  }
  if (direction_degrees > 30 && direction_degrees < 120) {
    mrm_bldc2x50->speedSet(2, constrain(map(direction_degrees, 30, 120, -speed, 0) + rotation_speed, -127, 127));
  }
  if (direction_degrees < 30) {
    mrm_bldc2x50->speedSet(2, constrain(map(direction_degrees, 0 ,30, -speed*0.6, -speed) + rotation_speed, -127, 127));
  }  
}

void RobotSoccer::LineLogic(int& linn) {
  int dir;
  int t;
  int error;
  bool brek = false;

  if (setup()) {
    headingToMaintain = heading();
  
  }
  if (lineCheck(8) || lineCheck(9) || lineCheck(10) || lineCheck(11) || lineCheck(12) || lineCheck(13) || lineCheck(14) || lineCheck(15)) {
    dir = 0;
  }
  else if (lineCheck(24) || lineCheck(25) || lineCheck(26) || lineCheck(27) || lineCheck(28) || lineCheck(29) || lineCheck(30) || lineCheck(31)) {
    dir = 1;
  }
  else if (lineCheck(0) || lineCheck(1) || lineCheck(2) || lineCheck(3) || lineCheck(4) || lineCheck(5) || lineCheck(6) || lineCheck(7)) {
    dir = 2;
    sm = 2;
  }
  else if (lineCheck(16) || lineCheck(17) || lineCheck(18) || lineCheck(19) || lineCheck(20) || lineCheck(21) || lineCheck(22) || lineCheck(23)) {
    dir = 3;
    sm = 1;
  }
  else {
    dir = -1;
  }
  if (dir == 0) {
    linn = 0;
    if (sm == 1) {
      linn = -25;
    }
    else if (sm == 2){
      linn = 25;
    }
    t = LIN_OVR;
    while (t != 0) {
      error = heading() - headingToMaintain;
      if (error > 180) {
        error = error - 360;
      } 
      else if (error < -180) {
        error = error + 360;
      }
      motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
      odi(LIN_SPD, linn, motspwr );
      t--;
      delay(1);
    }
    odi(0,0,0);
    sm = 0;
  }
  else if (dir == 1) {
    linn = -180;
    t = LIN_OVR;
    if (sm == 1) {
      linn = -155;
    }
    else if (sm == 2){
      linn = 155;
    }
    while (t != 0) {
      error = heading() - headingToMaintain;
      if (error > 180) {
        error = error - 360;
      } 
      else if (error < -180) {
        error = error + 360;
      }
      motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
      odi(LIN_SPD, linn, motspwr );
      t--;
      for (int i; i<32; i++) {
        if (i == 24) {
          i += 8; 
        }
        if (i == 8) {
          i += 8; 
        }
        if (lineRead(i)) {
          brek = true;
          break;
        }
      delay(1);
      }
      if (brek) {
        brek = false;
        break;
      }

    }
    odi(0,0,0);
    sm = 0;
  }
  else if (dir == 2) {
    linn = 90;
    t = LIN_OVR_STRF;
    while (t != 0) {
      error = heading() - headingToMaintain;
      if (error > 180) {
        error = error - 360;
      } 
      else if (error < -180) {
        error = error + 360;
      }
      motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
      odi(LIN_SPD, linn, motspwr );
      t--;
      for (int i; i<32; i++) {
        if (i == 0) {
          i += 8; 
        }
        if (i == 16) {
          i += 8; 
        }
        if (lineRead(i)) {
          brek = true;
          break;
        }

      }
      delay(1);
      if (brek) {
        brek = false;
        break;
      }
    }
    odi(0,0,0);
  }
  else if (dir == 3) {
    linn = -90;
    t = LIN_OVR_STRF;
    while (t != 0) {
      error = heading() - headingToMaintain;
      if (error > 180) {
        error = error - 360;
      } 
      else if (error < -180) {
        error = error + 360;
      }
      motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
      odi(LIN_SPD, linn, motspwr );
      t--;
      for (int i; i<32; i++) {
        if (i == 16) {
          i += 8; 
        }
        if (i == 0) {
          i += 8; 
        }
        if (lineRead(i)) {
          brek = true;
          break;
        }

      }
      delay(1);
      if (brek) {
        brek = false;
        break;
      }
    }
    
    odi(0,0,0);
  }
  else {
    linn = -360;
  }
}

void RobotSoccer::loop() {
  int bangle = 0;
  int error;
  int lin = 0;
  uint8_t red[8] = { 0b00000000, 0b00000000, 0b01100110, 0b01100110, 0b00000000, 0b00011000, 0b00000000, 0b00000000 };
  uint8_t green[8] = { 0b01111110, 0b11111111, 0b10011001, 0b10011001, 0b11111111, 0b01100110, 0b00111100, 0b00100100 };
  
  uint8_t redc[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
  uint8_t greenc[8] = { 0b01111110, 0b11111111, 0b10011001, 0b10011001, 0b11111111, 0b01100110, 0b00111100, 0b00100100 };

  uint8_t HRr[8] = { 0b01011010, 0b00000000, 0b00101010, 0b01010100, 0b00101010, 0b01010100, 0b00101000, 0b00010000 };
  uint8_t HRg[8] = { 0b00000000, 0b01011010, 0b01010100, 0b00101010, 0b01010100, 0b00101010, 0b00010100, 0b00001000 };

  if (setup()) {
    headingToMaintain = heading();
    
    // LEDSignText* signHR = new LEDSignText();  // Here, a text will be displayed instead of a 8x8 bitmap.
  }
  if (useLine == true) {
    LineLogic(lin);
  }
  else {
    lin = -360;
  }
  mrm_8x8a->bitmapCustomDisplay(redc, greenc);
  if (lin == -360) {
    if (mrm_ir_finder3->distance() > 2700) {
      state = 1;
    }
    if (mrm_ir_finder3->distance() < 2400) {
      state = 0;
    }
    if (mrm_ir_finder3->distance() == 0) {
      state = 2; 
    }
  }
  else {
    state = 3;
  }



  error = heading() - headingToMaintain;
  if (error > 180) {
    error = error - 360;
  } 
  else if (error < -180) {
    error = error + 360;
  }
  motspwr = pidKompass->calculate(error, false, MAX_TURN_RATE);
  // strcpy(signHR->text, "CROATIA");  



  if (ballAngle() >= 0) {
    bangle = 360.0 - fmod(ballAngle(), 180.0);
  } else {
    bangle = map(fmod(ballAngle() + 180.0, 180.0), 0, 180, 180, 0);
  }
  Serial.print(bangle);
  Serial.print(" ");
  Serial.print(mrm_ir_finder3->distance());
  Serial.print(" ");
  Serial.print(state);
  Serial.print(" ");
  Serial.print(bangle);
  Serial.print(" ");

  serialBT->print(bangle);
  serialBT->print(" ");
  serialBT->print(motspwr);
  serialBT->print(" ");
  serialBT->print(mrm_ir_finder3->distance());
  serialBT->print(" ");
  serialBT->print(state);
  serialBT->print(" ");
  serialBT->print(bangle);
  serialBT->print(" ");

  if (state == 1) {
    if (bangle <= 15 && bangle >= 0){
      odi(MTR_PWR, 0, motspwr);
      Serial.print("Napred");
      serialBT->print("Napred");
    }
    else if (bangle < 40 && bangle > 15){
      odi(MTR_PWR, map(bangle, 40, 27, -60, -20), motspwr);
      Serial.print("Desno Gore Di.");
      serialBT->print("Desno Gore Di.");
    }
    else if (bangle > 335 && bangle <= 360){
      odi(MTR_PWR-10, constrain(map(bangle, 350, 360, 60, 20), 60, 20), motspwr);
      Serial.print("Levo Gore Di.");
      serialBT->print("Levo Gore Di.");
    }
    else if (bangle < 70 && bangle >= 40){
      odi(MTR_PWR-20, map(bangle, 70, 40, -90, -45), motspwr);
      Serial.print("Desno.");
      serialBT->print("Desno.");
    }
    else if (bangle > 300 && bangle <= 335){
      odi(MTR_PWR-20, map(bangle, 300, 350, 90, 45), motspwr);
      Serial.print("Levoo.");
      serialBT->print("Levoo.");
    }
    else if (bangle >= 145 && bangle <= 180 ){
      odi(MTR_PWR-10, constrain(map(mrm_ir_finder3->distance(), 2500, 3000, 160, 90), 90, 170 ), motspwr);
      Serial.print("Levo dole di.");
      serialBT->print("Levo dole di.");
    }
    else if (bangle > 180 && bangle <= 215 ){
      odi(MTR_PWR-10, constrain(map(mrm_ir_finder3->distance(), 2500, 3000, -160, -90), -90, -170 ), motspwr);
      Serial.print("Desno dole di.");
      serialBT->print("Desno dole di.");
    }
    else{
      odi(MTR_PWR, -180, motspwr);
      Serial.print("Natrag.");
      serialBT->print("Natrag.");
    }
  }
  
  if (state == 0) {
    odi(MTR_PWR, ballAngle(), motspwr);
  }

  if (state == 2) {
    odi(MTR_PWR, -180, motspwr);
    serialBT->print("Ne vidi loptu, natrag");
  } 

  if (state == 3){
    odi(LIN_SPD, lin, motspwr);
  }

  Serial.println();
  serialBT->println();


}
void RobotSoccer::goAhead() {
	const uint8_t speed = 60;
	odi(speed, 0, motspwr);
	end();
}




/** Generic actions, use them as templates
*/


/** Starts robot
*/


/** Right distance to wall
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - in mm



// Marin, Dorian, Niksa, Nino je bio ovdje, zašto ovo čitaš
*/



/** Display fixed sign stored in sensor
@image - sign's number
*/

