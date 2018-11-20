package ca.mcgill.ecse211.Navigation;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Localization.LightLocalization;
import ca.mcgill.ecse211.odometer.Odometer;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
//import ca.mcgill.ecse211.odometer.Odometer;
//import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class contains methods to allow the robot to navigate to specified waypoint, or to turn to specific angle.
 * 
 * It is mainly called by the Ev3Boot class to navigate to strategic point. It is also called by other LightLocalization class to 
 * navigate to a line intersection.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 * @author Chaimae Fahmi
 *
 */
public class Navigator {

	private static final int FORWARD_SPEED = 200;
	private static final int TURN_SPEED = 75;
	
	public static int getTurnSpeed() {
		return TURN_SPEED;
	}

	public static int getForwardSpeed() {
		return FORWARD_SPEED;
	}

	private static final double TILE_SIZE = Ev3Boot.getTileSize();
	private static final int TURN_ERROR = 1;
	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static Odometer odo;
	private static double[] currentPosition;
	private static double[] lastPosition;
	private static boolean isNavigating;
	private static SampleProvider usDistance = Ev3Boot.getUSDistance();
	private static float[] usData = Ev3Boot.getUSData();

	private static double ReOrientDistance;
	private static double distanceDifference;
	private static double distance;
	
	/**
	 * 
	 * This method execute navigation to a specific point, if wanted it can light localize upon arrival.
	 * 
	 * This method causes the robot to travel to an intermediate waypoint in the case of diagonal travel.
	 * The method continuously polls the 2 rear light sensors. In the case that a line is detected by one of 
	 * them, its respective motor is stopped until the robot corrects its heading (the second light sensor detects 
	 * a line).
	 * In a loop calculate distance between current position and arrival point, if the distance is smaller than treshHold, stop the motors.
	 * When the distance from arrival point is acceptable stop the motors and light localize if specified.
	 * 
	 * @param x: x coordinate to navigate to
	 * @param y: y coordinate to navigate to
	 * @param treshHold: acceptable error distance
	 * @param localizing: should to robot localize upon arrival
	 */
	public static void travelTo(double x, double y, int treshHold, boolean localizing) {

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		currentPosition = odo.getXYT();
		lastPosition = odo.getXYT();

		double deltaX = x * TILE_SIZE - currentPosition[0];
		double deltaY = y * TILE_SIZE - currentPosition[1];
		double totalDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		isNavigating = true;
		ReOrientDistance = 0;

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(500);
		}

		if (deltaX == 0 && deltaY != 0) {
			turnTo(deltaY < 0 ? 180 : 0);
		} else {
			double baseAngle = deltaX > 0 ? 90 : 270;
			double adjustAngle;
			if ((deltaY > 0 && deltaX > 0) || (deltaY < 0 && deltaX < 0)) {
				adjustAngle = -1 * Math.toDegrees(Math.atan(deltaY / deltaX));
			} else {
				adjustAngle = Math.toDegrees(Math.atan(Math.abs(deltaY) / Math.abs(deltaX)));
			}

			turnTo(baseAngle + adjustAngle);
			// System.out.println("Base Angle: " + baseAngle + "\n Adjust Angle: " +
			// adjustAngle);
		}

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		while (isNavigating) {
			currentPosition = odo.getXYT();

			deltaX = x * TILE_SIZE - currentPosition[0];
			deltaY = y * TILE_SIZE - currentPosition[1];
			distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

			if (distance < treshHold) {
				leftMotor.stop(true);
				rightMotor.stop();
				isNavigating = false;
				
				if (treshHold >= 4) {
					travelTo(x, y, 1, false);
				}
				
				Sound.beepSequence();
				if (localizing) {
					LightLocalization.lightLocalize(x, y, true, totalDistance ,4);

				}
				return;
			}

			distanceDifference = Math.sqrt(Math.pow(currentPosition[0] - lastPosition[0], 2)
					+ Math.pow(currentPosition[1] - lastPosition[1], 2));
			ReOrientDistance += distanceDifference;

			lastPosition = currentPosition;

			if (ReOrientDistance > Ev3Boot.getTileSize() * 0.75) {
				Sound.beep();
				leftMotor.stop(true);
				rightMotor.stop(false);
				ReOrientDistance = 0;
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
				leftMotor.forward();
				rightMotor.forward();
			}

		}
	}


	/**
	 * This method makes the robot turn to the specified bearing.
	 * Mainly called by the travelTo() method and LightLocalize() method.
	 * 
	 * In a loop, calculate the difference between current heading and desired
	 * heading, if the difference is smaller than "turnError" stop turning and put
	 * motors back on forward.
	 * 
	 * Always turn the smallest angle.
	 * 
	 * @param theta: Bearing for the robot to readjust its heading to.
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

		leftMotor.setSpeed(TURN_SPEED);
		rightMotor.setSpeed(TURN_SPEED);

		currentPosition = odo.getXYT();

		double deltaT = (((theta - currentPosition[2]) % 360) + 360) % 360;

		if (deltaT < 180) {

			leftMotor.rotate(convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), 1000), true);
			rightMotor.rotate(-convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), 1000), true);

			while (Math.abs(odo.getXYT()[2] - theta) > TURN_ERROR) {
				// do nothing
			}
			leftMotor.stop(true);
			rightMotor.stop(false);
		} else {
			leftMotor.rotate(-convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), 1000), true);
			rightMotor.rotate(convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), 1000), true);
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
	 * @param clockwise: true to turn clockwise and false to turn counter clockwise.
	 * @param theta: amount of degree the robot has to turn.
	 */
	public static void turnBy(double theta, boolean clockwise, boolean blocking) {

		leftMotor.setSpeed(TURN_SPEED+50);
		rightMotor.setSpeed(TURN_SPEED+50);
		if (clockwise == false) {
			leftMotor.rotate(-convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), theta), true);
			rightMotor.rotate(convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), theta), !blocking);
		} else {
			leftMotor.rotate(convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), theta), true);
			rightMotor.rotate(-convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), theta), !blocking);
		}

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
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
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}