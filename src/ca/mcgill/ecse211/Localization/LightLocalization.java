package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.FinalProject.FinalProject;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class LightLocalization {
	private static double sensorOffset = -13.0;
	private static int counter = 1;
	private static float color;
	private static float lastColor;
	private static double temp;
	private static double yIntersectionplus;
	private static double yIntersectionminus;
	private static double xIntersectionplus;
	private static double xIntersectionminus;
	private static double xOrigin;
	private static double yOrigin;
	private static double TRACK = FinalProject.getTrack();
	private static double WHEEL_RAD = FinalProject.getWheelRad();
	private static final EV3LargeRegulatedMotor leftMotor = FinalProject.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = FinalProject.getRightmotor();
	private static SampleProvider SColor = FinalProject.getColorBack();
	private static float[] data = FinalProject.getColorBufferBack();
	private static final double TILE_SIZE = FinalProject.getTileSize();
	public static EV3GyroSensor gyro = FinalProject.gyro;
	public static Odometer odo;

	private static double treshold = 2 * TILE_SIZE;

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
			return;
		}

		rightMotor.setSpeed(150);
		leftMotor.setSpeed(150);

		SColor.fetchSample(data, 0);
		color = data[0] * 1000;

		rightMotor.rotate(Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);
		leftMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);

		while (counter <= 4) {
			lastColor = color;

			SColor.fetchSample(data, 0);
			color = data[0] * 1000;
			if (color - lastColor > 5) {
				temp = FinalProject.odo.getXYT()[2];
				System.out.println(temp);
				if (counter == 1) {
					yIntersectionminus = temp;
				}
				if (counter == 2) {
					xIntersectionplus = temp;
				}
				if (counter == 3) {
					yIntersectionplus = temp;
				}
				if (counter == 4) {
					leftMotor.stop(true);
					rightMotor.stop(false);

					xIntersectionminus = temp;
				}
				counter++;
			}
		}

		xOrigin = x + sensorOffset * Math.cos(Math.toRadians(yIntersectionminus - yIntersectionplus) / 2);
		yOrigin = y + sensorOffset * Math.cos(Math.toRadians(xIntersectionplus - xIntersectionminus) / 2);

		odo.setX(xOrigin);
		odo.setY(yOrigin);
		
		if (!positionOnly) {
			Navigator.turnBy((((xIntersectionplus - xIntersectionminus) + 360) % 360) / 2);
			gyro.reset();
		}
		
		Navigator.travelTo(0, 0);
		Button.waitForAnyPress();
		System.exit(0);
	}
}
