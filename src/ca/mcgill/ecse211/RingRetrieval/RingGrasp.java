package ca.mcgill.ecse211.RingRetrieval;

import ca.mcgill.ecse211.Ev3Boot.MotorController;
import ca.mcgill.ecse211.Navigation.Navigator;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is use to grab the ring of off the tree, 
 * once the robot is close enough. To do so,
 * it uses the constructed grabbing mechanism
 *
 */

public class RingGrasp extends MotorController{

//	private static final EV3LargeRegulatedMotor leftMotor  = Ev3BootGrasping.getLeftmotor();
//	private static final EV3LargeRegulatedMotor rightMotor = Ev3BootGrasping.getRightmotor();
//	private static final EV3LargeRegulatedMotor BigArmHook = Ev3BootGrasping.getBigArmHook();
//	private static final EV3LargeRegulatedMotor armHook    = Ev3BootGrasping.getArmHook();
//	
//	private static double wheelrad      = Ev3BootGrasping.getWheelRad();
//	private static double track         = Ev3BootGrasping.getTrack();
	private static double approach_dist = 7.0;
	
	/**
	 * This method is called inside the ring search process,
	 * when a ring is detected, this method is used to reteive it,
	 * It controls the two main components of the grabbing mechaism,
	 * which are the two motors. It rotates ths first motor
	 *  to raise the mechanism and align it with the ring.
	 *  Then it rotates the second motor which controls its arms 
	 *  to grab the ring. 
	 */
	public static void grasp(int color, int elevation, int ring_number)
	{
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		armHook.setSpeed(250);
		//No ring detected, go back to intersection
		if(color == 0) {
			leftMotor.rotate (-Navigator.convertDistance(WHEEL_RAD, approach_dist - 1.0), true);
			rightMotor.rotate(-Navigator.convertDistance(WHEEL_RAD, approach_dist - 1.0), false);
		}
		else {
			//Back off before commencing testing
			leftMotor.rotate (-Navigator.convertDistance(WHEEL_RAD, 7.0), true);
			rightMotor.rotate(-Navigator.convertDistance(WHEEL_RAD, 7.0), true);

			BigArmHook.setSpeed(50);
			//Check the elevation where the color has been detected
			if(elevation == 1)
				BigArmHook.rotateTo(127- (int)(ring_number*0.5));
			else if(elevation == 2)
				BigArmHook.rotateTo(93- (int)(ring_number*0.5));

			armHook.rotateTo(-90,true);
			//Coming in
			leftMotor.rotate (Navigator.convertDistance(WHEEL_RAD, 8.2), true);
			rightMotor.rotate(Navigator.convertDistance(WHEEL_RAD, 8.2), false);
			
			//Hook in
			armHook.rotateTo(-145);
			
			//Snap in
			armHook.setSpeed(80);
			armHook.rotateTo(-205,true);
			leftMotor.rotate (-Navigator.convertDistance(WHEEL_RAD, approach_dist + 0.2), true);
			rightMotor.rotate(-Navigator.convertDistance(WHEEL_RAD, approach_dist + 0.2), false);
			
			// Rotate the big arm down to get momentum to rotate up. 
			// The motor itself is not powerful enough to carry 4 rings normally.
			BigArmHook.setSpeed(200);
			
			int test;
//			while((test = BigArmHook.getTachoCount()) <= 120) 
//				BigArmHook.forward();
			
			//BigArmHook.stop(true);
			// Up we go
			
			while((test = BigArmHook.getTachoCount()) >= 40 && ring_number < 3)
				BigArmHook.backward();
			
			BigArmHook.stop();
			
		}
	}

	public static void removeRing() {
		
		//Lower the hook
		BigArmHook.setSpeed(50);
		BigArmHook.rotateTo(125);
		
		//Ready the small hook
		armHook.setSpeed(200);
		armHook.rotateTo(0);
		
		//Push out
		for(int i = 0; i < 2; i++)
			armHook.rotate(360);
		
	}
}
