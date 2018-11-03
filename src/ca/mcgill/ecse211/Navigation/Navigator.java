package ca.mcgill.ecse211.Navigation;

import ca.mcgill.ecse211.FinalProject.FinalProject;
import ca.mcgill.ecse211.Localization.LightLocalization;
import ca.mcgill.ecse211.odometer.Odometer;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
//import ca.mcgill.ecse211.odometer.Odometer;
//import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class contains methods to allow the robot to navigate to specified
 * waypoints.
 * 
 * This class contains methods to allow the robot to avoid obstacles.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Navigator {

	private static final int FORWARD_SPEED = 200;
	private static final double TILE_SIZE = FinalProject.getTileSize();
	private static final int TURN_ERROR = 3;
	private static final EV3LargeRegulatedMotor leftMotor = FinalProject.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = FinalProject.getRightmotor();
	private static Odometer odo;
	private static double[] currentPosition;
	private static boolean isNavigating;
	private static SampleProvider usDistance = FinalProject.getUSDistance();
	private static float[] usData = FinalProject.getUSData();


	/**
	 * 
	 * First calculate the angle needed to point toward the final destination, turn to that angle and move forward.
	 * 
	 * In a loop, calculate the distance between the current location and final destination, if it is under 2cm exit the loop.
	 * Every 5000 loops, re-orient the heading using "orientateTravel" method.
	 * 
	 * @param x: X-coordinate of the arrival point
	 * @param y: Y-coordinate of the arrival point
	 */
	public static void travelTo(double x, double y) {
		
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		currentPosition = odo.getXYT();

		double deltaX = x * TILE_SIZE - currentPosition[0];
		double deltaY = y * TILE_SIZE - currentPosition[1];
		double totalDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		isNavigating = true;

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		orientateTravel(x, y);

		currentPosition = odo.getXYT();

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		int i = 0;

		while (isNavigating) {
			i++;

			usDistance.fetchSample(usData, 0); // acquire data
			int obstDistance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

			currentPosition = odo.getXYT();

			if (i > 5000) {
				orientateTravel(x, y);
				leftMotor.forward();
				rightMotor.forward();
				i = 0;
			}

			deltaX = x * TILE_SIZE - currentPosition[0];
			deltaY = y * TILE_SIZE - currentPosition[1];
			double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

			if (distance < 2.0) {
				leftMotor.stop(true);
				rightMotor.stop();
				isNavigating = false;
				LightLocalization.lightLocalize(x, y, true, totalDistance);
				return;
			}

		}
	}

	/**
	 * Method called by "travelTo" method.
	 * 
	 * Re-orient the heading of the robot using the difference between current position and desired position
	 * 
	 * @param x: X-coordinate of the arrival point
	 * @param y: Y-coordinate of the arrival point
	 */
	public static void orientateTravel(double x, double y) {
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		currentPosition = odo.getXYT();

		double baseAngle = 0;
		double adjustAngle = 0;

		double deltaX = x * TILE_SIZE - currentPosition[0];
		double deltaY = y * TILE_SIZE - currentPosition[1];
		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		if (deltaX == 0 && deltaY != 0) {
			turnTo(deltaY < 0 ? 180 : 0);
		} else {
			baseAngle = deltaX > 0 ? 90 : 270;
			if ((deltaY > 0 && deltaX > 0) || (deltaY < 0 && deltaX < 0)) {
				adjustAngle = -1 * Math.toDegrees(Math.atan(deltaY / deltaX));
			} else {
				adjustAngle = Math.toDegrees(Math.atan(Math.abs(deltaY) / Math.abs(deltaX)));
			}

			if (Math.abs(currentPosition[2] - (baseAngle + adjustAngle)) > 5)
				turnTo(baseAngle + adjustAngle);

		}

		return;
	}

	/**
	 * This method makes the robot turn to the specified bearing.
	 * 
	 * In a loop, calculate the difference between current heading and desired heaing, 
	 * if smaller than "turnError" stop turning and put motors back on forward.
	 * 
	 * @param theta
	 *            Bearing for the robot to readjust its heading to.
	 */
	public static void turnTo(double theta) {

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);

		leftMotor.setSpeed(60);
		rightMotor.setSpeed(60);

		currentPosition = odo.getXYT();

		double deltaT = (((theta - currentPosition[2]) % 360) + 360) % 360;

		if (deltaT < 180) {

			leftMotor.rotate(convertAngle(FinalProject.getWheelRad(), FinalProject.getTrack(), 1000), true);
			rightMotor.rotate(-convertAngle(FinalProject.getWheelRad(), FinalProject.getTrack(), 1000), true);

			while (Math.abs(odo.getXYT()[2] - theta) > TURN_ERROR) {
				// do nothing
			}
			leftMotor.stop(true);
			rightMotor.stop(false);
		} else {
			leftMotor.rotate(-convertAngle(FinalProject.getWheelRad(), FinalProject.getTrack(), 1000), true);
			rightMotor.rotate(convertAngle(FinalProject.getWheelRad(), FinalProject.getTrack(), 1000), true);
			while (Math.abs(odo.getXYT()[2] - theta) > TURN_ERROR) {
				// do nothing
			}

			leftMotor.stop(true);
			rightMotor.stop(false);
		}

		// OdometerCorrections.correction = true;

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

	}

	/**
	 * Turn by the specified angle theta, can be both positive or negative
	 * 
	 * @param theta: amount of degree the robot has to turn.
	 */
	public static void turnBy(double theta) {

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(-convertAngle(FinalProject.getWheelRad(), FinalProject.getTrack(), theta), true);
		rightMotor.rotate(convertAngle(FinalProject.getWheelRad(), FinalProject.getTrack(), theta), true);

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the rotation of each wheel
	 * needed to rotate that angle
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}