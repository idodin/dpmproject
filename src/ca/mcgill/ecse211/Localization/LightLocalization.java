package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
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
	private static double sensorOffset = -11.6;
	private static int counter;
	private static float color;
	private static float lastColor;
	private static double temp;

	private static double firstLine;
	private static double secondLine;
	private static double thirdLine;
	private static double fourthLine;

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
	private static final int TURN_SPEED = Navigator.getTurnSpeed();
	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static int distance;

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

	public static void lightLocalize(double x, double y, boolean positionOnly, double traveledDistance, int corner) {

		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		//		if (traveledDistance < treshold) {
		//			Sound.buzz();
		//			return;
		//		}

		SColor.fetchSample(data, 0);
		color = data[0] * 1000;

		counter = 1;

		if (positionOnly) {
			Navigator.turnTo(45);
			Sound.beepSequence();
		}

		rightMotor.setSpeed(TURN_SPEED);
		leftMotor.setSpeed(TURN_SPEED);

		rightMotor.rotate(Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);
		leftMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);

		while (counter <= 4) {
			lastColor = color;

			SColor.fetchSample(data, 0);
			color = data[0] * 1000;
			if (color - lastColor > 3.5) {
				temp = Ev3Boot.odo.getXYT()[2];
				// System.out.println(temp);
				Sound.beep();
				if (counter == 1) {
					firstLine = temp;
				}
				else if (counter == 2) {
					secondLine = temp;
				}
				else if (counter == 3) {
					thirdLine = temp;
				}
				else if (counter == 4) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					fourthLine = temp;
				}
				counter++;
			}
		}

		switch (corner) {
		case 0:xOrigin = ((x * Ev3Boot.getTileSize()))
				+ (sensorOffset * Math.cos(Math.toRadians(firstLine - thirdLine) / 2)+3.0);
		yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(secondLine - fourthLine) / 2));

		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, 1, false);
		Navigator.turnTo((((secondLine+fourthLine)/2)+179)%360);
		
		usAverage.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.00);
		System.out.println("Distance"+distance);
		if(distance< 1.5 * TILE_SIZE) {
			Navigator.turnTo((((secondLine+fourthLine)/2)% 360));
		}
		
		break;


		case 1:	xOrigin = ((x * Ev3Boot.getTileSize()))
				+ (sensorOffset * Math.cos(Math.toRadians(fourthLine - secondLine) / 2)+ 3);
		yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(firstLine - thirdLine) / 2));
		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, 1, false);
		Navigator.turnTo((((firstLine+thirdLine)/2)+178)%360);
		
		usAverage.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.00);
		System.out.println("Distance"+distance);
		if(distance < 1.5 * TILE_SIZE) {
			Navigator.turnTo((((firstLine+thirdLine)/2)% 360));
		}
		break;

		case 2:xOrigin = ((x * Ev3Boot.getTileSize()))
				+ (sensorOffset * Math.cos(Math.toRadians(thirdLine - firstLine) / 2)-3.0);
		yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(fourthLine - secondLine) / 2));
		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, 1, false);
		Navigator.turnTo((((secondLine+fourthLine)/2)+178)%360);
		
		usAverage.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.00);
		System.out.println("Distance"+distance);
		if(distance > 1 * TILE_SIZE) {
			Navigator.turnTo((((secondLine+fourthLine)/2)% 360));
		}
		
		break;

		case 3:xOrigin = ((x * Ev3Boot.getTileSize()))
				+ (sensorOffset * Math.cos(Math.toRadians(secondLine - fourthLine) / 2)-3.0);
			yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(thirdLine - firstLine) / 2));
			odo.setX(xOrigin);
			odo.setY(yOrigin);

			Navigator.travelTo(x, y, 1, false);
			Navigator.turnTo((((firstLine+thirdLine)/2)+178)%360);
			
			usAverage.fetchSample(usData, 0);
			distance = (int) (usData[0] * 100.00);
			System.out.println("Distance"+distance);
			if(distance > 1.2 * TILE_SIZE) {
				Navigator.turnTo((((firstLine+thirdLine)/2)% 360));
			}
			break;



		case 4: xOrigin = ((x * Ev3Boot.getTileSize()))
				+ (sensorOffset * Math.cos(Math.toRadians(firstLine - thirdLine) / 2)+3.0);
				yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(secondLine - fourthLine) / 2));

		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, 1, false);
		Navigator.turnTo((((secondLine+fourthLine)/2)+178)%360);
		break;
		}

	gyro.reset();
}
}
