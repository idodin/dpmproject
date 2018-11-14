package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
<<<<<<< HEAD
 * This class contains methods for Rising Edge and Falling Edge Ultrasonic
 * Localization as well as Color Sensor Localization.
=======
 * This class role is to localize the robots position using the ultrasonic sensor assuming it is in a corner.
 * 
 * This class contains methods to execute Falling Edge Ultrasonic.
>>>>>>> 5488394... java doc improvement
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Localizer {

	private static final int FORWARD_SPEED = Navigator.getForwardSpeed();
	private static final int TURN_SPEED = Navigator.getTurnSpeed();
	private static final int FORWARD_ACCELERATION = 600;
	private static final int TURN_ACCELERATION = 600;
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
<<<<<<< HEAD
>>>>>>> 5488394... java doc improvement
=======
>>>>>>> 16c10c7... Update
	 * 
	 * @throws OdometerExceptions
	 */
	public static void localizeFE() throws OdometerExceptions {
		// Initialize variables
		double a, b, correction;
		a = b = 0;

		int dist, lastdist;
		dist = lastdist = Integer.MAX_VALUE;
		
		setSpeedAccel(TURN_SPEED, TURN_ACCELERATION);

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
//		Navigator.turnBy(1000, false, false);
//
//		while (dist < faceToTheWallDistance || dist == 0) {
//			usAverage.fetchSample(usData, 0);
//			dist = (int) (usData[0] * 100.00);
//		}
//
//		// Keep rotating until falling edge (right wall) is found.
//		while (true) {
//			usAverage.fetchSample(usData, 0);
//			dist = (int) (usData[0] * 100.00);
//			if (dist > 3 && dist < fallingDistance && lastdist <= fallingDistance) {
//				Sound.beep();
//				b = odo.getXYT()[2];
//				break;
//			}
//			lastdist = dist;
//		}
//
//		stopMotors();
//
//		// Correct theta and orientate to 0.
//		if (a < b) {
//			correction = 45 - (a + b) / 2;
//
//			// turnTo(180-(correction+odo.getXYT()[2]));
//			Navigator.turnBy(180 + correction + odo.getXYT()[2], true, true);
//
//		}
//		if (a >= b) {
//			correction = 225 - (a + b) / 2;
//			Navigator.turnBy(180 + correction + odo.getXYT()[2], true, true);
//		}
		
		Navigator.turnBy(75, false, true);
		
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
			System.out.println(color);
			if (color - lastColor > 6500) {
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
