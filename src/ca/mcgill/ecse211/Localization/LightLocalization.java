package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.FinalProject.FinalProject;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
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
	private static final EV3LargeRegulatedMotor leftMotor = FinalProject.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = FinalProject.rightMotor;
	private static final Odometer odo = FinalProject.odo;
	private static SampleProvider SColor = FinalProject.getColorBack();
	private static float[] data = FinalProject.getColorBufferBack();
	public static EV3GyroSensor gyro = FinalProject.gyro;

	
	public static void lightLocalize() {

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

		xOrigin = sensorOffset * Math.cos(Math.toRadians((yIntersectionminus - yIntersectionplus) / 2));
		yOrigin = sensorOffset * Math.cos(Math.toRadians(xIntersectionplus - xIntersectionminus) / 2);

		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(0, 0);

		Navigator.turnTo(xIntersectionminus);

		gyro.reset();

		Navigator.turnTo((180 + (xIntersectionplus - xIntersectionminus) / 2) % 360);

		gyro.reset();

		Button.waitForAnyPress();
		System.exit(0);
	}
}
