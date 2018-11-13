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
 * This class role is to localize the robots position using the light sensor 
 * assuming it is close to a line intersection.
 * 
 * This class contains methods to execute Circle Light Localization.
 * 
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
	 * This method execute circle light localization using the light sensor.
	 * 
	 * It is called by the Ev3Boot class at the start of the run after UltraSonic Localization to perfect the heading correction and
	 * to correct x and y. It is also called from the navigation class at the end of travelTo() to recorrect position values.
	 * 
	 * The robot rotates counter clockwise while scanning using a light sensor.
	 * Rotates in place until all 4 line are detected. Stores angle which we detected every lines.
	 * Using the 4 angles, compute where the real 0 degree heading is and where the intersection is.
	 * Navigate to the line intersection, turn to the real 0 degree heading and reset the gyroscope
	 * and set the new value of x and y.
	 * 
	 * @param x: x coordinate of the intersection the robot wants to localize
	 * @param y: y coordinate of the intersection the robot wants to localize
	 * @param positionOnly: boolean to determine whether to correct both heading and position or position only.
	 */
	public static void lightLocalize(double x, double y, boolean positionOnly) {

		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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
			if (color - lastColor > 3.5) {
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
				+ (sensorOffset * Math.cos(Math.toRadians(yIntersectionminus - yIntersectionplus) / 2)+2.0);
		yOrigin = (y * Ev3Boot.getTileSize())
				+ (sensorOffset * Math.cos(Math.toRadians(xIntersectionplus - xIntersectionminus) / 2));

		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, 1, false);
		Navigator.turnTo((((xIntersectionplus+xIntersectionminus)/2)+180)%360);
		gyro.reset();
	}
}
