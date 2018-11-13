package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class contains methods for Rising Edge and Falling Edge Ultrasonic
 * Localization as well as Color Sensor Localization.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Localizer {

	private static final int FORWARD_SPEED = Navigator.getForwardSpeed();
	private static final int TURN_SPEED = Navigator.getTurnSpeed();
	private static final int FORWARD_ACCELERATION = 2000;
	private static final int TURN_ACCELERATION = 2000;
	private static double WHEEL_RADIUS = Ev3Boot.getWheelRad();

	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static Odometer odo;
	private static double[] currentPosition;
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider SColor = Ev3Boot.getColorBack();
	private static float[] data = Ev3Boot.getColorBufferBack();
	private static float color;
	private static float lastColor;

	private static int fallingDistance = 40;
	private static int faceToTheWallDistance = 80;

	/**
	 * Falling Edge Localization with the Ultrasonic Sensor.
	 * 
	 * Turn until first falling edge is met, record angle. then turn clockwise until
	 * second falling edge is met, record angle.
	 * 
	 * Calculations done to determine where the 0 degrees heading is. Turn to 0
	 * degrees
	 * 
	 * @throws OdometerExceptions
	 */
	public static void localizeFE() throws OdometerExceptions {
		// Initialize variables
		double a, b, correction;
		a = b = 0;
		// boolean a1set, a2set, b1set, b2set;
		// a1set = a2set = b1set = b2set = false;
		int dist, lastdist;
		dist = lastdist = Integer.MAX_VALUE;

		// Get Odometer Instance
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		// Start turning, this will be stopped when Falling Edges are detected.
		Navigator.turnBy(1000, true, false);

		// Sample from Ultrasonic Sensor
		usAverage.fetchSample(usData, 0);
		dist = (int) (usData[0] * 100.00);

		// If we're already underneath the threshold, rotate until we are no longer
		// underneath it
		while (dist < faceToTheWallDistance || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (back wall) is found
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);

			if (dist > 3 && dist <= fallingDistance && lastdist <= fallingDistance) {
				Sound.beep();
				a = odo.getXYT()[2];
				break;
			}
			lastdist = dist;
		}

		stopMotors();
		Navigator.turnBy(1000, false, false);

		while (dist < faceToTheWallDistance || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (right wall) is found.
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
			if (dist > 3 && dist < fallingDistance && lastdist <= fallingDistance) {
				Sound.beep();
				b = odo.getXYT()[2];
				break;
			}
			lastdist = dist;
		}

		stopMotors();

		// Correct theta and orientate to 0.
		if (a < b) {
			correction = 45 - (a + b) / 2;

			// turnTo(180-(correction+odo.getXYT()[2]));
			Navigator.turnBy(180 + correction + odo.getXYT()[2], true, true);

		}
		if (a >= b) {
			correction = 225 - (a + b) / 2;
			Navigator.turnBy(180 + correction + odo.getXYT()[2], true, true);
		}
		
		Sound.beepSequence();
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.forward();
		rightMotor.forward();

		SColor.fetchSample(data, 0);
		color = data[0] * 1000;
		
		while (true) {
			lastColor = color;
			SColor.fetchSample(data, 0);
			color = data[0] * 1000;
			if (color - lastColor > 4) {
				leftMotor.stop(true);
				rightMotor.stop();
				leftMotor.rotate(-Navigator.convertDistance(WHEEL_RADIUS, 10),true);
				rightMotor.rotate(-Navigator.convertDistance(WHEEL_RADIUS, 10),false);
				break;
			}
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
	 * 
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
