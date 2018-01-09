#include "WPILib.h"
#include <math.h>
#include "RobotConstants.h"

//IMPORTANT: Limit switches return a value corresponding to the current running through them, hence:
//Pushed in == 0
//Pushed out == 1

/*TODO:
 * Test!!!!!!
 * 		Holding Doors
 * Code Victor Holding Door Motors
 * three modes:
 * 		Standby: fully retracted; active at the beginning and when moving across the field
 * 		Deployment: open and carrying totes
 * 		Released: rotated past 90 degrees to drop totes
 * 			Code loop for the motors according to the state
 * 			Code State change for each button
 * 		Both motors should rotate in opposite directions
 * 		Possibly make it able to change states while the doors are moving (solve for door position using time)
 * 		More to come on control mechanisms: gyrometers, limit switches, timer?
 * 		Set them to button 2 along with the holding plate
 * 		Figure out what to do ^
 * Manage Operator Joystick buttons, determine what works best for the operator
 * Write Autonomous routine
*/

/*----------CONTROLS-----------
 * Driver's Joystick:
 * 		X and Y-Axes: Drive
 * Operator's Joystick:
 * 		Y-Axis: Move Chain Forward and Back
 * 		Trigger: Extend and Retract Holding Plate and Arms
 * 		Button 2: Extend Arms to Release Totes
 * 		Button 3: Set Chain's Speed to .5 * max
 * 		Button 4: Set Chain's Speed to .33 * max
 * 		Button 5: Set Chain's Speed to .75 * max
 * 		Button 6: Set Holding Doors to Standby
 * 		Button 7: Set Holding Doors to Deploy
 * 		Button 8: Set Holding Doors to Release
 -----------------------------*/

class Robot: public SampleRobot
{
	RobotDrive myRobot;
	Talon gearMotor, chainMotor1, chainMotor2;
	Victor doorMotor1, doorMotor2;
	DigitalInput cageOnSwitch, cageOffSwitch, frontSensor, backSensor;
	Joystick operatorStick, driverStick;
	//AnalogInput frontSensor, backSensor;
	SmartDashboard *dashboard;
	//PWM Ports (0,1) (2,3) and (7,8) are gear boxes
public:
	Robot() :
			myRobot(0, 1, 2, 3), // Initializes drive motors/Talons on PWM ports 0-3
								 // Motors 0&1 are on the same gearbox
								 // Motors 2&3 are on the same gearbox
			gearMotor(4),  // Initializes tray motor on PWM port 4
			chainMotor1(5), // Initializes chain motor 1 on PWM port 5, on same gearbox as motor 2
			chainMotor2(6), // Initializes chain motor 2 on PWM port 6, on same gearbox as motor 1
			doorMotor1(7), // Initializes door motor 1 on PWM port 7
			doorMotor2(8), // Initializes door motor 2 on PWM port 8
			cageOnSwitch(0), // Initializes digital I/O port 0 for outer tray limit switch
			cageOffSwitch(1),// Initializes digital I/O port 1 for inner tray limit switch
			frontSensor(2), // Initializes analog I/O port 2 for front ultrasonic sensor -- CHANGED FROM ANALOG
			backSensor(3),  // Initializes analog I/O port 3 for back ultrasonic sensor
			operatorStick(0), // Initializes chain joystick on USB port 0
			driverStick(1), // Initializes drive joystick on USB port 1
			dashboard() // Thanks coding team! the controls are great
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(true);
	}

	double changeDoorState(double &oldState, double newState)
	{

		if(abs((oldState - newState)) > 1.05*timeIncrement*doorAnglePerSecond){ //The doors still need to move
			doorMotor1.Set((2 * (oldState < newState) - 1) *(1-.26*(oldState>newState))* autoDoorSpeed);
			doorMotor2.Set(-(2 * (oldState < newState) - 1) * (1-.26*(oldState>newState))*autoDoorSpeed2);
			oldState += (2 * (oldState < newState) - 1) * timeIncrement * doorAnglePerSecond; //updates position
		}
		else{ //The doors have reached the target position
			doorMotor1.Set(0);
			doorMotor2.Set(0);
			oldState = newState;
		}

		return oldState;


		/*if (motorDirection != 0) //This is a failsafe to prevent the robot from changing states while the doors are moving
		{
			return;
		}
		movementTimer.Start(); //once the timer reaches timerLimit, the doors will stop in the main loop
		if (newState > oldState) //The arms need to extend
		{
			motorDirection = 1; //note that the motors will move in opposite directions even though this variable exists
		}
		else if (oldState > newState) //The arms need to retract
		{
			motorDirection = -1;
		}*/

		//timerLimit = abs(newState - oldState)/turningRate; //formula: t == (df - di)/v: this is the time that the motors need to run for
		//t2 == |(df - di - v1t1)|/v2
		//v1 == motorDirection * v
		//t1 == movementTimer.Get()
	}

	void Autonomous()
	{
		AutonomousState state = START; // this is the variable for the state machine's state

		// debugMode = OFF in normal operation
		// debugMode = LINEARSPEED, ROTATESPEED, CONVEYORSPEED, or DOORSPEED to measure
		//             the appropriate value
		DebugState debugMode = DANCE;
		int iterator = 0; // used for debug mode

		/* position of the robot in feet */
		double xPosition = 0;
		double yPosition = 0;
		double nextObstacle = 0;
		double robotOrientation = 0;
		double conveyorTime = 0;
		double doorPosition = STANDBY;
		double trueMotorSpeed = 0;

		int danceWaves = 0;

		bool hasTote = false; // does the robot have a tote?
		bool inAutoZone = false;

		if(!debugMode){
			while(IsAutonomous() && IsEnabled()){

				if(state == START)
				{
					// start the robot without the tote, in the autozone, and facing left
					hasTote = false;
					inAutoZone = false;
					robotOrientation = LEFT;
					state = DRIVE;
				}

				if(state == DRIVE){
					nextObstacle = frontSensor.Get() /** feetPerVolt*/;

					inAutoZone = (xPosition >= (maxXDistanceAutoZone + minXDistanceAutoZone)/2);

					// if at end position, place tote down
					if(inAutoZone && yPosition < 0){
						if(trueMotorSpeed > 0){
							trueMotorSpeed -= timeIncrement * autoDriveSpeed;
							xPosition += (robotOrientation == LEFT) * feetPerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
							yPosition += (robotOrientation/DOWN) * feetPerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
						}
						else{
							myRobot.ArcadeDrive(0,0,true);
							state = PLACETOTE;
						}
					}
					// move until either next to tote or within next object
					else if(nextObstacle >= hasTote * safeStopDistance + minStopDistance){
						trueMotorSpeed += (trueMotorSpeed != autoDriveSpeed) * timeIncrement * autoDriveSpeed;
						myRobot.ArcadeDrive(trueMotorSpeed,0,true);
						xPosition += (robotOrientation == LEFT) * feetPerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
						yPosition += (robotOrientation/DOWN) * feetPerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
					}
					//
					else{
						myRobot.ArcadeDrive(0,0,true);

						// pick up the tote if we don't have it
						if(!hasTote){
							state = PICKUPTOTE;
						}
						// if we're facing left and not yet in autozone, rotate counterclockwise
						else if(robotOrientation == LEFT && !inAutoZone){
							state = ROTATECCW;
						}
						// otherwise (facing left & in autozone) or (facing down), rotate clockwise
						else{
							state = ROTATECW;
						}
					}
				}

				if(state == ROTATECW){
					// if in Auto Zone, rotate until in up position, otherwise rotate to left
					if(robotOrientation > inAutoZone * UP){
						trueMotorSpeed += (trueMotorSpeed != autoDriveSpeed) * timeIncrement * autoDriveSpeed;
						myRobot.ArcadeDrive(0,trueMotorSpeed,true);
						robotOrientation -= anglePerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
					}
					else{
						if(trueMotorSpeed > 0){
							trueMotorSpeed -= (trueMotorSpeed != autoDriveSpeed) * timeIncrement * autoDriveSpeed;
							myRobot.ArcadeDrive(0,trueMotorSpeed,true);
							robotOrientation -= anglePerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
						}
						else{
							trueMotorSpeed = 0;
							robotOrientation = inAutoZone*UP;
							myRobot.ArcadeDrive(0,0,true);
							state = DRIVE;
						}
					}
				}

				if(state == ROTATECCW){
					// rotate until in down position
					if(robotOrientation < DOWN){
						trueMotorSpeed += (trueMotorSpeed != autoDriveSpeed) * timeIncrement * autoDriveSpeed;
						myRobot.ArcadeDrive(0,trueMotorSpeed,true);
						robotOrientation += anglePerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
					}
					else{
						if(trueMotorSpeed > 0){
							trueMotorSpeed -= (trueMotorSpeed != autoDriveSpeed) * timeIncrement * autoDriveSpeed;
							myRobot.ArcadeDrive(0,trueMotorSpeed,true);
							robotOrientation += anglePerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
						}
						else{
							trueMotorSpeed = 0;
							robotOrientation = DOWN;
							myRobot.ArcadeDrive(0,0,true);
							state = DRIVE;
						}
					}
				}

				if(state == PICKUPTOTE){
					// if the tray is out and the door is in hold position and the tote has been loaded, switch to rotate
					if(conveyorTime >= timeToLoad /*&& !cageOffSwitch.Get() && doorPosition >= HOLD*/){
						// turn off all motors
						chainMotor1.Set(0);
						chainMotor2.Set(0);
						gearMotor.Set(0);
						doorMotor1.Set(0);
						doorMotor2.Set(0);
						hasTote = true;
						state = ROTATECCW;
						conveyorTime = 0;
					}
					else{
						// if tray is deployed, speed is 0, otherwise it is autoTraySpeed
						gearMotor.Set(cageOffSwitch.Get() * autoTraySpeed);

						// if door is at hold position, speed is 0, otherwise it is opened and angle is incremented
						//doorMotor1.Set((doorPosition < HOLD) * autoDoorSpeed);
						//doorMotor2.Set(-(doorPosition < HOLD) * autoDoorSpeed2);
						//doorPosition += (doorPosition < HOLD) * doorAnglePerSecond * timeIncrement;

						chainMotor1.Set(autoChainSpeed);
						chainMotor2.Set(autoChainSpeed);

						conveyorTime += timeIncrement;
					}
				}

				if(state == PLACETOTE){
					// if the tray is out and the door is in hold position and the tote has been loaded, switch to rotate
					if(conveyorTime >= 1.1* timeToLoad){
						// turn off all motors
						gearMotor.Set(0);
						doorMotor1.Set(0);
						doorMotor2.Set(0);
						chainMotor1.Set(0);
						chainMotor2.Set(0);
						hasTote = false;
						state = END;
					}
					else{
						// if tray is stowed, speed is 0, otherwise it is -autoTraySpeed
						//gearMotor.Set( -cageOffSwitch.Get() * autoTraySpeed);

						// if door is at release position, speed is 0, otherwise it is opened and angle is incremented
						//doorMotor1.Set((doorPosition < RELEASE) * autoDoorSpeed);
						//doorMotor2.Set(-(doorPosition < RELEASE) * autoDoorSpeed2);
						//doorPosition += (doorPosition < RELEASE) * doorAnglePerSecond * timeIncrement;
						chainMotor1.Set(-autoChainSpeed);
						chainMotor2.Set(-autoChainSpeed);

						conveyorTime += timeIncrement;
					}
				}

				if(state == END){
					// if robot is clear of crate and door is open, close door
					if(backSensor.Get() /** feetPerVolt*/ >= safeStopDistance && doorPosition > HOLD){
						myRobot.ArcadeDrive(0,0,true);
						doorMotor1.Set(-(doorPosition > HOLD) * autoDoorSpeed);
						doorMotor2.Set((doorPosition > HOLD) * autoDoorSpeed2);
						doorPosition -= (doorPosition > HOLD) * doorAnglePerSecond * timeIncrement;
					}
					// otherwise, try to move forward to clear crate
					else if(frontSensor.Get() /** feetPerVolt*/ >= minStopDistance){
						myRobot.ArcadeDrive(autoDriveSpeed,0,true);
					}
					// otherwise, turn everything off until end of autonomous mode
					else{
						myRobot.ArcadeDrive(0,0,true);
						doorMotor1.Set(0);
						doorMotor2.Set(0);
					}
				}

				Wait(timeIncrement);
			}
		}
		else if(debugMode == DANCE){
			/*
			 Scott - 3/13/2015 Competition.  Made new code for the robot to dance.  It moves forward,
			 spins counterclockwise, opens its arms, waves twice, spins clockwise, waves arms twice, then
			 closes the arms.
			 */

			while(IsAutonomous() && IsEnabled()){
				if(state == START)
				{
					// start the robot without the tote, in the autozone, and facing left
					xPosition = 0;
					yPosition = 0;
					hasTote = false;
					inAutoZone = false;
					robotOrientation = LEFT;
					doorPosition = 0;
					state = OPENARMS;
					trueMotorSpeed = 0;
				}

				if(state == DRIVE){

					myRobot.ArcadeDrive(-trueMotorSpeed,-driftCorrect*trueMotorSpeed,true);


					if(xPosition <= (distToCrate-minStopDistance)){
						trueMotorSpeed += (trueMotorSpeed < autoDriveSpeed) * timeIncrement * autoDriveSpeed;
						xPosition += feetPerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
					}
					else{
						if(trueMotorSpeed > 0){
							trueMotorSpeed -= timeIncrement * autoDriveSpeed;
							xPosition += feetPerSecond * timeIncrement * trueMotorSpeed/autoDriveSpeed;
						}
						else{
							myRobot.ArcadeDrive(0,0,true);
							state = ROTATECCW;
						}
					}
				}
				if(state == ROTATECW){
					// if in Auto Zone, rotate until in up position, otherwise rotate to left
					if(robotOrientation > LEFT){
						trueMotorSpeed += (trueMotorSpeed < autoRotateSpeed) * timeIncrement * autoRotateSpeed;
						myRobot.ArcadeDrive(-.1,-trueMotorSpeed,true);
						robotOrientation -= anglePerSecond * timeIncrement * trueMotorSpeed/autoRotateSpeed;
					}
					else{
						if(trueMotorSpeed > 0){
							trueMotorSpeed -= (trueMotorSpeed != autoRotateSpeed) * timeIncrement * autoRotateSpeed;
							myRobot.ArcadeDrive(-.1,-trueMotorSpeed,true);
							robotOrientation -= anglePerSecond * timeIncrement * trueMotorSpeed/autoRotateSpeed;
						}
						else{
							trueMotorSpeed = 0;
							robotOrientation = LEFT;
							myRobot.ArcadeDrive(0,0,true);
							//state = DRIVE;
							state = ROTATECCW;
						}
					}
				}

				if(state == ROTATECCW){
					// rotate until in down position
					if(robotOrientation < DOWN){
						trueMotorSpeed += (trueMotorSpeed < autoRotateSpeed) * timeIncrement * autoRotateSpeed;
						myRobot.ArcadeDrive(-.1,trueMotorSpeed,true);
						robotOrientation += anglePerSecond * timeIncrement * trueMotorSpeed/autoRotateSpeed;
					}
					else{
						if(trueMotorSpeed > 0){
							trueMotorSpeed -= (trueMotorSpeed != autoRotateSpeed) * timeIncrement * autoRotateSpeed;
							myRobot.ArcadeDrive(-.1,trueMotorSpeed,true);
							robotOrientation += anglePerSecond * timeIncrement * trueMotorSpeed/autoRotateSpeed;
						}
						else{
							trueMotorSpeed = 0;
							robotOrientation = DOWN;
							myRobot.ArcadeDrive(0,0,true);
							//state = DRIVE;
							state = ROTATECW;
						}
					}
				}

				if(state == OPENARMS){
					if(doorPosition < 60){
						doorMotor1.Set(autoDoorSpeed);
						doorMotor2.Set(-0 * autoDoorSpeed2);
						doorPosition += doorAnglePerSecond * timeIncrement * .85;
					}
					else{
						doorMotor1.Set(0);
						doorMotor2.Set(0);
						doorPosition = 90;
						state = DRIVE;
					}
				}

				if(state == CLOSEARMS){
					if(doorPosition > 0){
						doorMotor1.Set(-autoDoorSpeed2 * .85);
						doorMotor2.Set(autoDoorSpeed2);
						doorPosition -= doorAnglePerSecond * timeIncrement * .85;
					}
					else{
						doorMotor1.Set(0);
						doorMotor2.Set(0);
						doorPosition = 0;
						state = END;
					}
				}

				if(state == WAVEARMSLEFT){
					if(doorPosition < (135 - 45 * (danceWaves%3==2) )){
						doorMotor1.Set(autoDoorSpeed2 * .85);
						doorMotor2.Set(autoDoorSpeed2);
						doorPosition += doorAnglePerSecond * timeIncrement * .85;
					}
					else{
						doorMotor1.Set(0);
						doorMotor2.Set(0);
						if(danceWaves%3==2){
							doorPosition = 90;
							danceWaves++;
							state = ROTATECW;
						}
						else{
							doorPosition = 135;
							danceWaves++;
							state = WAVEARMSRIGHT;
						}
					}
				}

				if(state == WAVEARMSRIGHT){
					if(doorPosition > (45 + 45 * (danceWaves%3==2))){
						doorMotor1.Set(-autoDoorSpeed2 * .85);
						doorMotor2.Set(-autoDoorSpeed2);
						doorPosition -= doorAnglePerSecond * timeIncrement * .85;
					}
					else{
						doorMotor1.Set(0);
						doorMotor2.Set(0);
						if(danceWaves%3==2){
							doorPosition = 90;
							danceWaves++;
							state = CLOSEARMS;
						}
						else{
							doorPosition = 45;
							danceWaves++;
							state = WAVEARMSLEFT;
						}
					}
				}

				if(state == END){
					// turn off all motors
					myRobot.ArcadeDrive(0,0,true);
					doorMotor1.Set(0);
					doorMotor2.Set(0);
				}


				Wait(timeIncrement);
			}
		}
		else if(debugMode == LINEARSPEED){
			// Accelerate 1 second, drive 1 second, decelerate 1 second
			// Average speed is distance traveled/2

			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				myRobot.ArcadeDrive(autoDriveSpeed*iterator*timeIncrement,0,true);
				Wait(timeIncrement);
			}
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				myRobot.ArcadeDrive(autoDriveSpeed,0,true);
				Wait(timeIncrement);
			}
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				myRobot.ArcadeDrive(autoDriveSpeed*(1-iterator*timeIncrement),0,true);
				Wait(timeIncrement);
			}

			myRobot.ArcadeDrive(0,0,true);

		}
		else if(debugMode == ROTATESPEED){

			// Accelerate 1 second, drive 1 second, decelerate 1 second
			// Average speed is distance traveled/2

			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				myRobot.ArcadeDrive(0,autoDriveSpeed*iterator*timeIncrement,true);
				Wait(timeIncrement);
			}
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				myRobot.ArcadeDrive(0,autoDriveSpeed,true);
				Wait(timeIncrement);
			}
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				myRobot.ArcadeDrive(0,autoDriveSpeed*(1-iterator*timeIncrement),true);
				Wait(timeIncrement);
			}
			myRobot.ArcadeDrive(0,0,true);

		}
		else if(debugMode == DOORSPEED){

			// Open door motors for 100 ms to measure angle traversed to get doorAnglePerSecond value

			/* IMPORTANT: MAKE SURE THESE ARE DEFINED IN THE CORRECT DIRECTIONS FIRST */
			doorMotor1.Set(autoDoorSpeed);
			doorMotor2.Set(-autoDoorSpeed2);
			Wait(1);
			doorMotor1.Set(0);
			doorMotor2.Set(0);

		}
		else if(debugMode == CONVEYORSPEED){

			// operate conveyor to measure how long to load a crate and set conveyorTime value
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				chainMotor1.Set(autoChainSpeed*iterator*timeIncrement,true);
				chainMotor2.Set(autoChainSpeed*iterator*timeIncrement,true);
				Wait(timeIncrement);
			}
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				chainMotor1.Set(autoChainSpeed,true);
				chainMotor2.Set(autoChainSpeed,true);
				Wait(timeIncrement);
			}
			for(iterator = 1; iterator <= 1/timeIncrement; iterator++){
				chainMotor1.Set(autoDriveSpeed*(1-iterator*timeIncrement),true);
				chainMotor2.Set(autoDriveSpeed*(1-iterator*timeIncrement),true);
				Wait(timeIncrement);
			}
			chainMotor1.Set(0);
			chainMotor2.Set(0);
		}

		myRobot.ArcadeDrive(0,0,true);
	}

	void OperatorControl()
	{
		//Holding Door Variables
		double doorState = STANDBY;
		double newDoorState = STANDBY;
		/*Timer *holdingDoorTimer = new Timer(); //Milo - 2/13/15 - I have no idea why, but I can only allocate this variable dynamically
		double holdingDoorTimerLimit; //the time the holding doors should move to get to the desired position
		int holdingDoorDirection; //what direction the holding doors should move in, 1 == clockwise && -1 == counter-clockwise?
		const double holdingDoorSpeed = 1.0; //how fast the holding doors should move*/

		//double for gear motor and chain motor speeds? see line 47
		double chainMotorSpeedLimiter = 0.5;
		double gearMotorSpeed = 0.0;

		// Start Operator Control Mode with cage idle and in stowed position
		bool cageIsExtended = !(cageOnSwitch.Get());

		//Enable Robot
		//myRobot.SetSafetyEnabled(true);

		//Preliminary computations

		// Loop while in operator control
		while (IsOperatorControl() && IsEnabled())
		{
			//----------MOVE ROBOT----------
			myRobot.ArcadeDrive(-driverStick.GetY(), -driverStick.GetX() + driftCorrect,true);

			//------------------------------
			//------CONTROL GEAR MOTOR------
			gearMotor.Set(gearMotorSpeed);
			// Check motor state.
			if(!cageIsExtended)
			{
				if (operatorStick.GetTrigger()) // on trigger press, extend tray if stowed
				{
					// if tray is already stowed, operate gear forward to deploy
					//this->changeDoorState(doorState, HOLD, *holdingDoorTimer, holdingDoorTimerLimit, holdingDoorDirection);
					//newDoorState = HOLD;
					gearMotorSpeed = .75;
				}
				else if (!cageOnSwitch.Get() && gearMotorSpeed != 0)
				{
					// if tray hits outer limit switch, turn off motor and indicate extended
					cageIsExtended = true;
					gearMotorSpeed = 0.0;
				}
			}
			else
			{
				if (operatorStick.GetTrigger()) // on trigger press, stow tray if extended
				{
					// if tray is already deployed, operate gear backward to stow
					//newDoorState = RELEASE;
					gearMotorSpeed = -.75;
				}
				else if (!cageOffSwitch.Get() && gearMotorSpeed != 0)
				{
					// if tray hits inner limit switch, turn off motor and indicate stowed
					// this->changeDoorState(doorState, STANDBY, *holdingDoorTimer, holdingDoorTimerLimit, holdingDoorDirection);
					// IF WE HAVE TOTES, THIS MAY BREAK THE MOTOR. SHOULD BE TESTED WITH CAUTION!
					cageIsExtended = false;
					gearMotorSpeed = 0.0;
				}
			}


			// The below code opens the tray only without operating arms, linked to button 9
			if(!cageIsExtended)
			{
				if (operatorStick.GetRawButton(9)) // on trigger press, extend tray if stowed
				{
					// if tray is already stowed, operate gear forward to deploy
					//this->changeDoorState(doorState, HOLD, *holdingDoorTimer, holdingDoorTimerLimit, holdingDoorDirection);
					gearMotorSpeed = .75;
				}
				else if (!cageOnSwitch.Get() && gearMotorSpeed != 0)
				{
					// if tray hits outer limit switch, turn off motor and indicate extended
					cageIsExtended = true;
					gearMotorSpeed = 0.0;
				}
			}
			else
			{
				if (operatorStick.GetRawButton(9)) // on trigger press, stow tray if extended
				{
					// if tray is already deployed, operate gear backward to stow
					//newDoorState = RELEASE;
					gearMotorSpeed = -.75;
				}
				else if (!cageOffSwitch.Get() && gearMotorSpeed != 0)
				{
					// if tray hits inner limit switch, turn off motor and indicate stowed
					// this->changeDoorState(doorState, STANDBY, *holdingDoorTimer, holdingDoorTimerLimit, holdingDoorDirection);
					// IF WE HAVE TOTES, THIS MAY BREAK THE MOTOR. SHOULD BE TESTED WITH CAUTION!
					cageIsExtended = false;
					gearMotorSpeed = 0.0;
				}
			}
			//------------------------------
			//-----CONTROL CHAIN MOTORS-----
			// Need to move chain motors 1&2 in sync since they are on the same gearbox
			// Change the speed of the chain motors mid-match if necessary
			/*
			 * button 4 == .33 * max speed
			 * button 3 == .5 * max speed
			 * button 5 == .75 * max speed
			 */
			if (operatorStick.GetRawButton(4))
			{
				chainMotorSpeedLimiter = .33;
			}
			else if (operatorStick.GetRawButton(3))
			{
				chainMotorSpeedLimiter = .5;
			}
			else if (operatorStick.GetRawButton(5))
			{
				chainMotorSpeedLimiter = 1; //was .75
			}
			// Limit speed by constant multiplier so we don't overdrive it
			chainMotor1.Set(operatorStick.GetY()*chainMotorSpeedLimiter);
			chainMotor2.Set(operatorStick.GetY()*chainMotorSpeedLimiter);
			//------------------------------
			//---MOVE HOLDING DOOR MOTORS---

			/*if(operatorStick.GetRawButton(2)){
				// move to hold from standby or release
				if(doorState == STANDBY || doorState == RELEASE){
					newDoorState = HOLD;
				}
				// move to release from hold
				else if(doorState == HOLD){
					newDoorState = RELEASE;
				}
			}*/
			// direct position buttons for manual door operation

			if(operatorStick.GetRawButton(7) || operatorStick.GetRawButton(9)){
				//newDoorState = HOLD;
				// manually retract left door
				doorMotor2.Set(-autoDoorSpeed2 * .85);
			}
			else if (operatorStick.GetRawButton(10) || operatorStick.GetRawButton(8))
			{
				//manually extend left door out
				//doorMotor1.Set(-autoDoorSpeed * .85);
				doorMotor2.Set(autoDoorSpeed2 * .85);
				//doorState += timeIncrement * doorAnglePerSecond * .85; //update position
			}
			else{
				doorMotor2.Set(0);
			}

			/*if(operatorStick.GetRawButton(8)){
				newDoorState = STANDBY;
			}*/

			if(operatorStick.GetRawButton(6) || operatorStick.GetRawButton(8)){
				// manually retract right door
				doorMotor1.Set(autoDoorSpeed2 * .85);
				//newDoorState = RELEASE;
			}
			//-MANUALLY OVERRIDE HOLDING DOORS-
			//In order to do this, HOLD DOWN buttons 10 or 11
			else if (operatorStick.GetRawButton(11) || operatorStick.GetRawButton(9))
			{
				//manually extend right door out
				doorMotor1.Set(-autoDoorSpeed * .85);
				//doorMotor2.Set(-autoDoorSpeed2 * .85);
				//doorState -=  timeIncrement * doorAnglePerSecond * .85; //update position
			}
			else
			{
				doorMotor1.Set(0);
				//doorState = changeDoorState(doorState, newDoorState);
			}
			//---------------------------------

			// Update Dashboard with data
			dashboard->PutNumber("Front Clearance (ft)",frontSensor.Get());/**feetPerVolt*/;
			dashboard->PutNumber("Back Clearance (ft)",backSensor.Get());/**feetPerVolt*/;
			dashboard->PutNumber("Drive Speed",driverStick.GetY());
			dashboard->PutNumber("Rotate Speed",driverStick.GetX());
			dashboard->PutNumber("Door Angle",doorState);
			if(newDoorState == STANDBY)
				dashboard->PutString("Target Door Position", "Standby");
			else if(newDoorState == HOLD)
				dashboard->PutString("Target Door Position", "Hold Totes");
			else
				dashboard->PutString("Target Door Position", "Release Totes");
			if(cageIsExtended)
				dashboard->PutString("Tray Position", "Extended");
			else
				dashboard->PutString("Tray Position","Stowed");
			dashboard->PutNumber("Conveyor Speed",chainMotorSpeedLimiter);

			/*if (operatorStick.GetRawButton(2))
			{
				this->changeDoorState(doorState, RELEASE, *holdingDoorTimer, holdingDoorTimerLimit, holdingDoorDirection);
			}
			if (holdingDoorTimer->HasPeriodPassed(holdingDoorTimerLimit))
			{
				holdingDoorTimer->Stop();
				holdingDoorTimer->Reset();
				holdingDoorTimerLimit = 0.01; //This is a failsafe to prevent the above conditional from running after it's resolved
				holdingDoorDirection = 0; //Another failsafe: the arms will not move when this variable is zero
			}
			else
			{
				doorMotor1.Set(-holdingDoorDirection*holdingDoorSpeed);
				doorMotor2.Set(holdingDoorDirection*holdingDoorSpeed);
			}*/
			//------------------------------
			Wait(timeIncrement);
		}
	}

	void Test()
	{
		while(IsTest() && IsEnabled()){
		myRobot.Drive(.1,0);
		Wait(0.005);
		}
	}

};

START_ROBOT_CLASS(Robot);
