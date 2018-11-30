package ca.mcgill.ecse211.RingRetrieval;

import java.util.HashMap;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Ev3Boot.MotorController;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Inside this class is implemented the searching method, which makes the robot
 * travel to and around the tree until it detects a ring. Also contains method
 * to make the robot beep according to the color of the ring grabbed.
 * 
 * This class extends "MotorController" to facilitate all the motor logic.
 */
public class RingSearch extends MotorController {

	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static int distance;
	private static HashMap<Integer, double[]> positionMap;
	private static Odometer odo;

	private static int color;
	private static int elevation;
	private static int ring_number = 0;

	/**
	 * This method makes the robot turn around the tree. The switch case decides
	 * which side the robot travels to first, based on the value of position.
	 * Position is computed inside the Ev3Boot class, where the distance between the
	 * robot and each side of the tree is computed, and the closest side is recorded
	 * in the position field. At each side of the tree, we call the colorDetection
	 * method from the checkColor class, which uses the light sensor to look for one
	 * of the four ring colors. Once the colorDetection method has returned, if the
	 * detected color is the one with the highest value it attempts to grasp the
	 * ring, otherwise it keeps turning around the tree to find a higher value ring.
	 * 
	 * @param position: Which tree side is the closest side to the robot
	 * @param ringSet_x: The tree's x coordinate
	 * @param ringSet_y: The tree's y coordinate
	 */
	public static void turnAroundTree(int position, int ringSet_x, int ringSet_y) {
		// Initialize Hash Map with tree search positions and integer indicating visits.
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// donothing;
		}

		positionMap = new HashMap<Integer, double[]>();
		positionMap.put(0, new double[] { ringSet_x, ringSet_y - 1.5, 0 });
		positionMap.put(1, new double[] { ringSet_x + 1.5, ringSet_y, 0 });
		positionMap.put(2, new double[] { ringSet_x, ringSet_y + 1.5, 0 });
		positionMap.put(3, new double[] { ringSet_x - 1.5, ringSet_y, 0 });

		int getColor = 0;

		double[] posArray = positionMap.get(position);
		double[] odoPosition = odo.getXYT();
		int[] currentPosition = new int[3];
		currentPosition[0] = (int) Math.round(odoPosition[0] / TILE_SIZE);
		currentPosition[1] = (int) Math.round(odoPosition[1] / TILE_SIZE);

		System.out.println(position);
		System.out.println("Going To" + posArray[0] + "," + posArray[1]);

		switch (position) {
		case 0:
			Navigator.toStraightNavigator(posArray[0], posArray[1] + 0.5, 8);
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		case 1:
			Navigator.toStraightNavigator(posArray[0] - 0.5, posArray[1], 8);
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		case 2:
			Navigator.toStraightNavigator(posArray[0], posArray[1] - 0.5, 8);
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		case 3:
			Navigator.toStraightNavigator(posArray[0] + 0.5, posArray[1], 8);
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		}

		turnTo((360 - 90 * position) % 360);
		forwardBy(-0.5 * TILE_SIZE);
		Localizer.circleLocalize(posArray[0], posArray[1]);
		turnTo((360 - 90 * position) % 360);
		Navigator.travelUntil();

		CheckColor.colorDetection();

		color = CheckColor.getDetectedColor();
		elevation = CheckColor.getElevation();

		RingGrasp.grasp(color, elevation, ring_number);

		if (color != 0) {
			ring_number++;
		}

		switch (position) {
		case 0:
			Navigator.toStraightNavigator(posArray[0], posArray[1] + 0.5, 8);
			break;
		case 1:
			Navigator.toStraightNavigator(posArray[0] - 0.5, posArray[1], 8);
			break;
		case 2:
			Navigator.toStraightNavigator(posArray[0], posArray[1] - 0.5, 8);
			break;
		case 3:
			Navigator.toStraightNavigator(posArray[0] + 0.5, posArray[1], 8);
			break;
		}
	}

	/**
	 * Method called to travel to the next side of the tree. Calls itself
	 * recursively until all visitable side are visited. It correct itself on each
	 * side of the tree and call the method to detect rings and grab it if there is
	 * one.
	 * 
	 * @param sequenceStart: first side of tree visited
	 * @param currentPosition: current side of the tree
	 * @param previousPosition: previous side of the tree
	 * @return
	 */
	public static boolean travelPosition(int sequenceStart, int currentPosition, int previousPosition) {
		System.out.println("Travelling");
		int i = ((currentPosition == 0 ? 4 : currentPosition) - previousPosition) == 1 ? 1 : -1;
		int nextPosition = (currentPosition + 4 + i) % 4;
		int getColor;
		System.out.println("Next position: " + nextPosition);

		double[] posArray = positionMap.get(nextPosition);

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
				positionMap.put(nextPosition, new double[] { posArray[0], posArray[1], -1 });
				return false;
			}
			System.out.println("Going To" + posArray[0] + "," + posArray[1]);

			// Travel to position
			Navigator.travelTo(posArray[0], posArray[1], 3, false);
			turnTo((360 - 90 * currentPosition) % 360);
			Localizer.circleLocalize(posArray[0], posArray[1]);
			turnTo((360 - 90 * currentPosition) % 360);
			forwardBy(-20);
			Navigator.travelUntil();
			System.out.println("Before turn " + odo.getXYT()[2]);
			System.out.println("Coordinates before :" + odo.getXYT()[0] + "," + odo.getXYT()[1]);
			System.out.println("After turn" + odo.getXYT()[2]);
			System.out.println("Coordinates after :" + odo.getXYT()[0] + "," + odo.getXYT()[1]);

			CheckColor.colorDetection();

			color = CheckColor.getDetectedColor();
			elevation = CheckColor.getElevation();

			RingGrasp.grasp(color, elevation, ring_number);

			if (color != 0) {
				ring_number++;
			}

			if (!travelPosition(sequenceStart, nextPosition, currentPosition)) {
				double[] returnPosArray = positionMap.get(currentPosition);
				Navigator.travelTo(returnPosArray[0], returnPosArray[1], 7, false);
				return false;
			} else {
				return true;
			}
			// If already marked as blocked, we've already attempted to visit all possible
			// positions.
		} else {
			return true;
		}
	}

	/**
	 * beep once we detect a ring. 1: orange 2: yellow 3: green 4: blue
	 * 
	 * @param color: amount of time to beep
	 */
	public static void beepColor(int color) {
		for (int i = 0; i < 5 - color; i++)
			Sound.beep();
	}
}
