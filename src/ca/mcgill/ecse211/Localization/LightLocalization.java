package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

/**
 * This is the constructor, it's used to localize when the robot is at an
 * intersection, using the light sensor.
 */
public class LightLocalization {
	private static double sensorOffset = -11.7;
	private static int counter;
	private static float color;
	private static float lastColor;
	private static double temp;
	private static double yIntersectionplus;
	private static double yIntersectionminus;
	private static double xIntersectionplus;
	private static double xIntersectionminus;
	private static double xOrigin;
	private static double yOrigin;
	private static double TRACK = Ev3Boot.getTrack();
	private static double WHEEL_RAD = Ev3Boot.getWheelRad();
	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static SampleProvider SColor = Ev3Boot.getColorBack();
	private static float[] data = Ev3Boot.getColorBufferBack();
	private static final double TILE_SIZE = Ev3Boot.getTileSize();
	public static EV3GyroSensor gyro = Ev3Boot.gyro;
	public static Odometer odo;

	private static double treshold = 1 * TILE_SIZE;

	/**
	 * Rotate in place until all 4 line are detected, calculations done to determine
	 * what is the current position.Turn by the computed angle, then reset the gyro.
	 * Travel to the origin.
	 * 
	 * Reset once, ~4 sec to reset.
	 * 
	 * Requirement: Needs to be close enough to an intersection so that when the
	 * robot rotates on place it detects all 4 lines. Needs to be in the lower left
	 * quadrant to correct angle, but does not have to be in the lower left quadrant
	 * to correct X and Y.
	 */

	public static void lightLocalize(double x, double y, boolean positionOnly, double traveledDistance) {

		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		if (traveledDistance < treshold) {
			Sound.buzz();
			return;
		}

		SColor.fetchSample(data, 0);
		color = data[0] * 1000;

		counter = 1;

		if (positionOnly) {
			Navigator.turnTo(45);
			Sound.beepSequence();
		}

		rightMotor.setSpeed(75);
		leftMotor.setSpeed(75);

		rightMotor.rotate(Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);
		leftMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);

		while (counter <= 4) {
			lastColor = color;

			SColor.fetchSample(data, 0);
			color = data[0] * 1000;
			if (color - lastColor > 5) {
				temp = Ev3Boot.odo.getXYT()[2];
				// System.out.println(temp);
				Sound.beep();
				if (counter == 4) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					xIntersectionminus = temp;
				}
				else if (counter == 2) {
					xIntersectionplus = temp;
				}
				else if (counter == 3) {
					yIntersectionplus = temp;
				}
				else if (counter == 1) {
					yIntersectionminus = temp;
				}
				counter++;
			}
		}

//		Navigator.turnBy((((xIntersectionplus - xIntersectionminus) + 360) % 360) / 2, false);
		xOrigin = ((x * Ev3Boot.getTileSize()))

				+ (sensorOffset * Math.cos(Math.toRadians(yIntersectionminus - yIntersectionplus) / 2)+1);
		yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(xIntersectionplus - xIntersectionminus) / 2));

		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, false);
		Navigator.turnTo((((xIntersectionplus+xIntersectionminus)/2)+181)%360);
		gyro.reset();
	}
}
