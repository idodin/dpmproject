package ca.mcgill.ecse211.RingRetrieval;

import java.util.HashMap;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This is the classes's constructor. Inside this class is implemented the
 * searching method, which makes the robot turn around the tree until it detects
 * a ring.
 */
public class RingSearch {

	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static int distance;
	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static final int FORWARD_SPEED = Navigator.getForwardSpeed();
	private static HashMap<Integer, int[]> positionMap;
	private static Odometer odo;

	/**
	 * This method makes the robot turn around the tree. The switch case decides
	 * which side the robot travels to first, based on the value of position.
	 * Position is computed inside the Ev3Boot class, where the distance between the
	 * robot and each side of the tree is computed, and the closest side is recorded
	 * in the position field. At each side of the tree, we call the colorDetection
	 * method from the checkColor class, which uses the light sensor to look for one
	 * of the four ring colors. Once the colorDetection method has returned, if the
	 * detected color is the one with the highest value it calls the grasp method
	 * from RingGrasp, otherwise it keeps turning around the tree to find a higher
	 * value ring.
	 * 
	 * @param position: Which tree side is the closest side to the robot
	 * @param ringSet_x: The tree's x coordinate
	 * @param ringSet_y: The tree's y coordinate
	 */
	public static void turnAroundTree(int position, int ringSet_x, int ringSet_y) {
		// Initialize Hash Map with tree search positions and integer indicating visits.
		
		
		
		try {
			odo = Odometer.getOdometer();
		}
		catch (OdometerExceptions e) {
			//donothing;
		}

		positionMap = new HashMap<Integer, int[]>();
		;
		positionMap.put(0, new int[] { ringSet_x, ringSet_y - 1, 0 });
		positionMap.put(1, new int[] { ringSet_x + 1, ringSet_y, 0 });
		positionMap.put(2, new int[] { ringSet_x, ringSet_y + 1, 0 });
		positionMap.put(3, new int[] { ringSet_x - 1, ringSet_y, 0 });

		int getColor = 0;

		int[] posArray = positionMap.get(position);
		double[] odoPosition = odo.getXYT();
		int[] currentPosition = new int[3];
		currentPosition[0] = (int)Math.round(odoPosition[0] / Ev3Boot.getTileSize());
		currentPosition[1] = (int)Math.round(odoPosition[1] / Ev3Boot.getTileSize());

		if(Math.sqrt(Math.pow(posArray[0] - currentPosition[0], 2) + Math.pow(posArray[1] - currentPosition[1], 2)) > 4) {
			Navigator.travelTo(currentPosition[0] + (posArray[0] - currentPosition[0])/2, currentPosition[1] + (posArray[1] - currentPosition[1])/2, 4, true);
		}
		Navigator.travelTo(posArray[0], posArray[1], 4, true);
		Navigator.turnTo((360 - 90 * position) % 360);
		CheckColor.restartChecker();
		CheckColor.colorDetection();
		getColor = CheckColor.getDetectedColor();

		if (getColor != 0) {
			beepColor(getColor);
			Navigator.turnTo((360 - 90 * position) % 360);
			leftMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 3), true);
			rightMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 3), false);
			Ev3Boot.getArmHook().rotateTo(-205);
			Ev3Boot.getBigArmHook().rotateTo(CheckColor.getElevation() == 1 ? 102 : 32, false);
			leftMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), true);
			rightMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), false);
			Ev3Boot.getArmHook().rotateTo(-230);
			leftMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 20), true);
			rightMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 20), false);
			return;
		}

		if (!travelPosition(position, position, (position + 3) % 4)) {
			travelPosition(position, position, (position + 5) % 4);
			return;
		}

	}

	public static boolean travelPosition(int sequenceStart, int currentPosition, int previousPosition) {
		System.out.println("Travelling");
		int i = ((currentPosition == 0 ? 4 : currentPosition) -  previousPosition) == 1 ? 1 : -1;
		int nextPosition = (currentPosition + 4 + i) % 4;
		int getColor;
		System.out.println("Next position: " + nextPosition);

		int[] posArray = positionMap.get(nextPosition);

		if (nextPosition == sequenceStart) {
			System.out.println(posArray[0] + "," + posArray[1]);
			System.out.println("Back at start");
			return true;
		}

		// Check that not already marked as blocked.;
		if (posArray[2] == 0) {
			if (!(posArray[0] > 0 && posArray[0] < 8 && posArray[1] > 0 && posArray[1] < 8)) {
				System.out.println(posArray[0] + "," + posArray[1]);
				System.out.println("Next position out of bounds: " + nextPosition);
				positionMap.remove(nextPosition);
				positionMap.put(nextPosition, new int[] { posArray[0], posArray[1], -1 });
				return false;
			}
			Navigator.travelTo(posArray[0], posArray[1], 2, false);
			Navigator.turnTo((360 - 90 * nextPosition) % 360);
			CheckColor.restartChecker();
			CheckColor.colorDetection();
			getColor = CheckColor.getDetectedColor();

			if (getColor != 0) {
				beepColor(getColor);
				Navigator.turnTo((360 - 90 * nextPosition) % 360);
				leftMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 3), true);
				rightMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 3), false);
				Ev3Boot.getArmHook().rotateTo(-205);
				Ev3Boot.getBigArmHook().rotateTo(CheckColor.getElevation() == 1 ? 102 :32, false);
				leftMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), true);
				rightMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), false);
				Ev3Boot.getArmHook().rotateTo(-230);
				leftMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 20), true);
				rightMotor.rotate(-Navigator.convertDistance(Ev3Boot.getWheelRad(), 20), false);
				return true;
			}
			if (!travelPosition(sequenceStart, nextPosition, currentPosition)) {
				int[] returnPosArray = positionMap.get(currentPosition);
				Navigator.travelTo(returnPosArray[0], returnPosArray[1], 2, false);
				return false;
			}
			else {
				return true;
			}
	    // If already marked as blocked, we've already attempted to visit all possible positions.
		} else {
			return true;
		}
	}
	
	public static void beepColor(int color) {
		for(int i = 0; i<5-color; i++) Sound.beep();
	}
}
