#include <Arduino.h>
#include <mrm-8x8a.h>
#include <mrm-board.h>
#include <mrm-bldc2x50.h>
#include <mrm-can-bus.h>
#include <mrm-col-b.h>
#include <mrm-col-can.h>
#include <mrm-common.h>
#include <mrm-fet-can.h>
#include <mrm-imu.h>
//#include <mrm-ir-finder2.h>
#include <mrm-ir-finder3.h>
//#include <mrm-ir-finder-can.h>
#include <mrm-node.h>
#include <mrm-pid.h>
#include <mrm-ref-can.h>
#include <mrm-robot.h>
#include "mrm-robot-soccer.h"
#include <mrm-switch.h>
//#include <mrm-us.h>
#include <mrm-us-b.h>
#include <mrm-us1.h>


const int a16 = 32;
const int a32 = 33;
const int S0 = 26;
const int S1 = 25;
const int S2 = 14;
const int S3 = 27;


Robot *robot;

Preferences preferences;

void setup() {
  pinMode(a16, INPUT);
  pinMode(a32, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
	robot = new RobotSoccer((char*)"NG Soccer Bot"); // RobotLine, RobotMaze, RobotMin, RobotSoccer, or Your custom robot. "My robot" is Bluetooth name.
	robot->print("Start.\n\r");

}

void loop() {
	robot->refresh();
}
 