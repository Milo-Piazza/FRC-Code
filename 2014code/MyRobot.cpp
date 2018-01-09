#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 


		/*
		//compressor
		Compressor c = new Compressor(4,2);
		c.Start();
		
		
		//solenoid
		Solenoid s[8];
		
		for (int i=0; i<8; i++) {
			s[i] = new Solenoid(i+1); //allocate the Solenoid object
		}
		for (int i=0; i<8; i++) {
			s[i].Set(true); //turn them all on
		}
		for (int i=0; i<8; i++) {
			s[i].Set(false); //turn them off in turn
			Wait(1.0);
		}
		for (int i=0; i<8; i++) {
			s[i].Set(true); //turn them all back on in turn
			Wait(1.0);
			delete s[i]; //delete the objects
		}
		
		
		//relay
		Relay r;
		r = new Relay(2, kBothDirections); //allocate a Relay object
		r.Set(kForward); //turn the relay to forward
		Wait(1.0);
		r.Set(kBackward); //turn the relay to backward
		*/

class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	

public:
	RobotDemo():
		myRobot(1, 2),	// these must be initialized in the same order
		stick(1)		// as they are declared above.
	{
		myRobot.SetExpiration(0.1);
	}

	
	// Drive left & right motors for 2 seconds then stop 
	void Autonomous()
	{
		myRobot.SetSafetyEnabled(false);
		
		NetworkTable *table = NetworkTable::GetTable("datatable");
		uint32_t pressure;
		bool pressed = false;
		Compressor *c = new Compressor(5,1);

	    while (true){

	    	
	    	
	    	pressure = c->GetPressureSwitchValue();
	    	table->PutNumber( "pressure",  (double)pressure);

	    	if ( (int)pressure == 1) {
	    	    c -> Stop();
	    	}
	    	else if ( ! c->Enabled() ){
	    	    c -> Start();
	    	}

	    	Wait(.20);
	    }
	    
		/*myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot*/
	}

	
	// Runs the motors with arcade steering.  
	void OperatorControl()
	{
		myRobot.SetSafetyEnabled(true);

		//.10 makes the robot reeeeallly slooooow
		//myRobot.SetMaxOutput(.40);
		
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	
	// Runs during test mode
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

