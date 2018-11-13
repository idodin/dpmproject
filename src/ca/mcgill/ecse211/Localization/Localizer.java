package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class role is to localize the robots position using the ultrasonic sensor assuming it is in a corner.
 * 
 * This class contains methods to execute Falling Edge Ultrasonic.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Localizer {

	private static final int FORWARD_SPEED = 100;
	private static final int TURN_SPEED = 60;
	private static final int FORWARD_ACCELERATION = 2000;
	private static final int TURN_ACCELERATION = 2000;

	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static Odometer odo;
	private static double[] currentPosition;
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider color = Ev3Boot.getColorBack();
	private static float[] colorBuffer = Ev3Boot.getColorBufferBack();

	private static int d = 33;
	private static double k = 3.5;
	private static int rrising = 20;
	private static int rfalling = 30;

	/**
	 * This method execute Falling Edge Localization using the Ultrasonic Sensor.
	 * 
	 * It is called by the Ev3Boot class at the start of the run to roughly correct the heading of the robot.
	 * A more precise heading correction will be done after.
	 * 
	 * First, the robot turns clockwise until distance is greater than d + k + rfalling.
	 * This ensure that if the robot start facing the wall, it only start looking for falling edge after turning away from the wall.
	 * Then the robot continue turning clockwise until 2 consecutive readings of distance is smaller than d + 5 + k.
	 * Stop the motors when the first falling edge is found and record the angle as angle "a".
	 * Then the robot start turning counter clockwise until 2 consecutive readings of distance is smaller than d + 5 + k.
	 * Stop the motors and record the angle as angle "b".
	 * Using "a" and "b", compute the angle correction to determine current angle.
	 * Finally, turn to the 0 degree heading.
	 * 
	 * @throws OdometerExceptions
	 */
	public static void localizeFE() throws OdometerExceptions {
		// Initialize variables
		double a, b, correction;
		a = b = 0;
//		boolean a1set, a2set, b1set, b2set;
//		a1set = a2set = b1set = b2set = false;
		int dist, lastdist;
		dist = lastdist = Integer.MAX_VALUE;

		// Get Odometer Instance
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		// Start turning, this will be stopped when Falling Edges are detected.
		Navigator.turnBy(1000, true);

		// Sample from Ultrasonic Sensor
		usAverage.fetchSample(usData, 0);
		dist = (int) (usData[0] * 100.00);

		// If we're already underneath the threshold, rotate until we are no longer
		// underneath it
		while (dist < d + k + rfalling || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (back wall) is found
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			
			if (dist > 3 && dist <= d + 5 + k && lastdist <= d + 5 + k) {
				Sound.beep();
				a = odo.getXYT()[2];
				break;
			}
			
//			if (dist > 3 && dist <= d + 10 - k && lastdist <= d + 10- k && a1set && !a2set) {
//				Sound.beep();
//				a2 = odo.getXYT()[2];
//				a2set = true;
//			}
//			if (a1set && a2set) {
//				a = (a1 + a2) / 2;
//				break;
//			}
			
			lastdist = dist;
		}

		stopMotors();
		Navigator.turnBy(1000, false);

		while (dist < d + k + rfalling || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (right wall) is found.
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist > 4 && dist < 35 && dist <= d + k -1 && lastdist <= d + k) {
				Sound.beep();
				b = odo.getXYT()[2];
				break;
			}
//			if (dist > 3 && dist <= d - k && lastdist <= d + k && b1set && !b2set) {
//				Sound.beep();
//				b2 = odo.getXYT()[2];
//				b2set = true;
//			}
//			if (b1set && b2set) {
//				b = (b1 + b2) / 2;
//				break;
//			}
			lastdist = dist;
		}

		stopMotors();

		// Correct theta and orientate to 0.
		if (a < b) {
			correction = 45 - (a + b) / 2;

			// turnTo(180-(correction+odo.getXYT()[2]));
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			Navigator.turnTo(0);
			Navigator.turnBy(4, true);
			Ev3Boot.gyro.reset();
			odo.setTheta(0);
			Navigator.turnTo(0);

		}
		if (a >= b) {
			correction = 225 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			Navigator.turnTo(0);
			Navigator.turnBy(4, true);
			Ev3Boot.gyro.reset();
			odo.setTheta(0);
			Navigator.turnTo(0);
			// odo.setTheta(odo.getXYT()[2] - (225 - (a+b)/2));
		}

	}

	/**
	 * Stop both motors at the same time.
	 */
	private static void stopMotors() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * Set Speed and Acceleration for both motors.
	 * @param speed
	 * @param accel
	 */
	private static void setSpeedAccel(int speed, int accel) {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(accel);
			motor.setSpeed(speed);
		}
	}
}
