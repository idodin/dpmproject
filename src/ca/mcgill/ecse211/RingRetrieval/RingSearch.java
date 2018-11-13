package ca.mcgill.ecse211.RingRetrieval;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.Navigation.Navigator;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This is the classes's constructor. Inside this class is implemented
 * the searching method, which makes the robot turn around the tree
 * until it detects a ring.
 */
public class RingSearch {

	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static int distance;
	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static final int FORWARD_SPEED = Navigator.getForwardSpeed();


	/** 
	 *  This method makes the robot turn around the tree. The switch case decides which side the robot
	 *  travels to first, based on the value of position. 
	 *  Position is computed inside the Ev3Boot class, where the distance between the robot 
	 *  and each side of the tree is computed, and the closest side is recorded in the position field.
	 *  At each side of the tree, we call the colorDetection method from the checkColor class, 
	 *  which uses the light sensor to look for one of the four ring colors.
	 *  Once the colorDetection method has returned, if the detected color is the one with the highest value 
	 *  it calls the grasp method from RingGrasp, otherwise it keeps turning around the tree 
	 *  to find a higher value ring.
	 * 
	 * @param position: Which tree side is the closest side to the robot
	 * @param ringSet_x: The tree's x coordinate
	 * @param ringSet_y: The tree's y coordinate 
	 */
	public static void turnAroundTree(int position,int  ringSet_x,int ringSet_y)
	{
		switch (position) {
		case 0: 
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x-1, ringSet_y, 2, false);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x, ringSet_y+1, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x+1, ringSet_y, 2, false);
				Navigator.turnTo(0);
				break;		

		case 1: Navigator.travelTo(ringSet_x-1, ringSet_y, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x, ringSet_y+1, 2, false);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x+1, ringSet_y, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, false);
				Navigator.turnTo(0);
				break;


		case 2: Navigator.travelTo(ringSet_x, ringSet_y+1, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x+1, ringSet_y, 2, false);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x-1, ringSet_y, 2, false);
				Navigator.turnTo(0);
				break;

		case 3: Navigator.travelTo(ringSet_x+1, ringSet_y, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, false);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x-1, ringSet_y, 2, true);
				Navigator.turnTo(0);
				Navigator.travelTo(ringSet_x, ringSet_y+1, 2, false);
				Navigator.turnTo(0);
				break;

		}

		usAverage.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.00);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.forward();
		rightMotor.forward();

		while (true) {
			usAverage.fetchSample(usData, 0);
			distance = (int) (usData[0] * 100.00);
			if(distance<10) {
				rightMotor.stop(true);
				leftMotor.stop(false);
				break;

		}


	}
}
}
