/*
 * RobotConstants.h
 *
 *  Created on: Feb 14, 2015
 *      Author: team4019 2014 code
 */

#ifndef SRC_ROBOTCONSTANTS_H_
#define SRC_ROBOTCONSTANTS_H_

//State Definitions for Holding doors, the integers are degrees that each arm is held out
const double STANDBY = 0;
const double HOLD = 90;
const double RELEASE = 120;

// State Definitions for robot orientation in angles
const double LEFT = 0;
const double UP = -90;
const double DOWN = 90;

// Autonomous mode state machine states
enum AutonomousState
{
	START = 0,
	DRIVE = 1,
	ROTATECW = 2,
	ROTATECCW = 3,
	PICKUPTOTE = 4,
	PLACETOTE = 5,
	END = 6,
	OPENARMS = 7,
	CLOSEARMS = 8,
	WAVEARMSLEFT = 9,
	WAVEARMSRIGHT = 10
};

// Debug states for testing and measuring robot constants
enum DebugState {
	OFF = 0,
	LINEARSPEED = 1,
	ROTATESPEED = 2,
	CONVEYORSPEED = 3,
	DOORSPEED  = 4,
	DANCE = 5
};

/* the time each loop of autonomous mode lasts in seconds */
const double timeIncrement = 0.005;

/* CHANGE THESE IF AUTONOMOUS IS OFF CALIBRATION */
const double anglePerSecond = 30; // how fast the robot rotates
const double feetPerSecond = 2; // how fast the robot moves forward
const double timeToLoad = 2;
const double doorAnglePerSecond = 100;
const double driftCorrect = 0; // negative is correct to the right, positive is correct to left

/* constant known distances in feet */
const double maxXDistanceAutoZone = 16.83; // X distance from starting position to end of Autonomous Zone
const double minXDistanceAutoZone = 10.33; // X distance from starting position to start of Autonomous Zone
const double YDistanceAutoZone = 27; // Y distance of Autonomous Zone
const double safeStopDistance = 3.33; // Distance to stop away from any object besides tote currently set to the max width of any other robot
const double minStopDistance = feetPerSecond/2; // distance to stop away from tote

const double distToCrate = 5.42;

/* constant motor speeds, from 0 (off) to 1 (full power) */
const double autoDriveSpeed = 1; // how fast to drive the robot
const double autoRotateSpeed = 1;
const double autoTraySpeed = 1.0; // how fast to deploy the tray
const double autoChainSpeed = 0.5; // how fast to drive the conveyor
const double autoDoorSpeed = 0.204;	   // right arm
const double autoDoorSpeed2 = 0.24;		// left arm

/* conversion factor for ultrasonic sensors */
const double feetPerVolt = 8.5; /* when in analog sense mode, .0008138? 8.5?*/

#endif /* SRC_ROBOTCONSTANTS_H_ */
