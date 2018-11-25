package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class role is to localize the robots position using the ultrasonic
 * sensor assuming it is in a corner.
 * 
 * This class contains methods to execute Falling Edge Ultrasonic.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Localizer {

	private static final int FORWARD_SPEED = Navigator.getForwardSpeed();
	private static final int TURN_SPEED = Navigator.getTurnSpeed();
	private static final int TURN_ACCELERATION = 600;
	public static final int SENSOR_OFFSET = 13;
	private static final int CORRECTOR_SPEED = Navigator.getForwardSpeed() / 3;

	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static Odometer odo;
	private static SampleProvider usAverage = Ev3Boot.getUSAverage();
	private static float[] usData = Ev3Boot.getUSData();
	private static SampleProvider colorLeft = Ev3Boot.getColorLeft();
	private static float[] colorBufferLeft = Ev3Boot.getColorLeftBuffer();
	private static float currentColorLeft;
	private static float oldColorLeft;
	private static SampleProvider colorRight = Ev3Boot.getColorRight();
	private static float[] colorBufferRight = Ev3Boot.getColorRightBuffer();
	private static float currentColorRight;
	private static float oldColorRight;

	private static int fallingDistance = 40;
	private static int faceToTheWallDistance = 80;

	/**
	 * This method execute Falling Edge Localization using the Ultrasonic Sensor.
	 * 
	 * It is called by the Ev3Boot class at the start of the run to roughly correct
	 * the heading of the robot. A more precise heading correction will be done
	 * after.
	 * 
	 * First, the robot turns clockwise until distance is greater than d + k +
	 * rfalling. This ensure that if the robot start facing the wall, it only start
	 * looking for falling edge after turning away from the wall. Then the robot
	 * continue turning clockwise until 2 consecutive readings of distance is
	 * smaller than d + 5 + k. Stop the motors when the first falling edge is found
	 * and record the angle as angle "a". Then the robot start turning counter
	 * clockwise until 2 consecutive readings of distance is smaller than d + 5 + k.
	 * Stop the motors and record the angle as angle "b". Using "a" and "b", compute
	 * the angle correction to determine current angle. Finally, turn to the 0
	 * degree heading.
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

		// Turn in opposite direction.
		stopMotors();
		Navigator.turnBy(1000, false, false);

		// Do nothing until we re-pass previously detected wall.
		while (dist < faceToTheWallDistance || dist == 0) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);
		}

		// Keep rotating until falling edge (right wall) is found.
		while (true) {
			usAverage.fetchSample(usData, 0);
			dist = (int) (usData[0] * 100.00);

			if (dist > 3 && dist <= fallingDistance && lastdist <= fallingDistance) {
				if (angleDiff(odo.getXYT()[2], a) < 60)
					continue;
				Sound.beep();
				b = odo.getXYT()[2];
				break;
			}
			lastdist = dist;
		}

		Sound.beepSequence();

		stopMotors();

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		// leftMotor.forward();
		// rightMotor.forward();

		// SColor.fetchSample(data, 0);
		// color = data[0] * 1000;

		// while (true) {
		// lastColor = color;
		// SColor.fetchSample(data, 0);
		// color = data[0] * 1000;
		// System.out.println(color);
		// if (color - lastColor > 6500) {
		// leftMotor.stop(true);
		// rightMotor.stop();
		// leftMotor.rotate(-Navigator.convertDistance(WHEEL_RADIUS, 10), true);
		// rightMotor.rotate(-Navigator.convertDistance(WHEEL_RADIUS, 10), false);
		// break;
		// }
		// }

		// Correct theta and orientate to 0.
		if (a < b) {
			correction = 45 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			Navigator.turnTo(0);

		} else if (a >= b) {
			correction = 225 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			Navigator.turnTo(0);
		}

	}

	public static void localizeColor() throws OdometerExceptions {
		boolean ySet = false;
		boolean xSet = false;

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		leftMotor.forward();
		rightMotor.forward();

		colorLeft.fetchSample(colorBufferLeft, 0);
		currentColorLeft = colorBufferLeft[0] * 1000;
		oldColorLeft = currentColorLeft;

		colorRight.fetchSample(colorBufferRight, 0);
		currentColorRight = colorBufferRight[0] * 1000;
		oldColorRight = currentColorRight;

		while (true) {

			oldColorLeft = currentColorLeft;
			colorLeft.fetchSample(colorBufferLeft, 0);
			currentColorLeft = colorBufferLeft[0] * 1000;

			oldColorRight = currentColorRight;
			colorRight.fetchSample(colorBufferRight, 0);
			currentColorRight = colorBufferRight[0] * 1000;

			// COPY START

			if (currentColorLeft - oldColorLeft > 19 && Navigator.pollMultiple(false)) {

				leftMotor.stop(true);
				double theta = odo.getXYT()[2];
				while (currentColorRight - oldColorRight <= 19) {
					if (rightMotor.getAcceleration() != CORRECTOR_SPEED
							|| leftMotor.getAcceleration() != CORRECTOR_SPEED) {
						setSpeedAccel(CORRECTOR_SPEED, TURN_ACCELERATION);
					}
					if (angleDiff(odo.getXYT()[2], theta) > 25) {
						rightMotor.stop(true);
						leftMotor.stop();
						leftMotor.rotate(-1 * Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), true);
						rightMotor.rotate(-1 * Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), false);
						leftMotor.forward();
						rightMotor.forward();
						break;
					}
					oldColorRight = currentColorRight;
					colorRight.fetchSample(colorBufferRight, 0);
					currentColorRight = colorBufferRight[0] * 1000;
				}

				setSpeedAccel(FORWARD_SPEED, TURN_ACCELERATION);

				if (ySet) {
					odo.setX(Ev3Boot.getTileSize() + SENSOR_OFFSET);
					odo.setTheta(90);
					xSet = true;
					stopMotors();
					break;
				} else {
					odo.setY(Ev3Boot.getTileSize() + SENSOR_OFFSET);
					odo.setTheta(0);
					ySet = true;
				}
				stopMotors();

				leftMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), true);
				rightMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), false);
				Navigator.turnTo(90);
				leftMotor.forward();
				rightMotor.forward();
				oldColorLeft = currentColorLeft;

				leftMotor.setSpeed(Navigator.getForwardSpeed());
				rightMotor.setSpeed(Navigator.getForwardSpeed());
				leftMotor.forward();
				rightMotor.forward();

				try {
					Thread.sleep(400);
					colorRight.fetchSample(colorBufferRight, 0);
					currentColorRight = colorBufferRight[0] * 1000;
					colorLeft.fetchSample(colorBufferLeft, 0);
					currentColorLeft = colorBufferLeft[0] * 1000;
					oldColorLeft = currentColorLeft;
					oldColorRight = currentColorRight;

				} catch (InterruptedException e) {
				}
				continue;

			} else if (currentColorRight - oldColorRight > 19 && Navigator.pollMultiple(true)) {
				// System.out.println("Right line detected:\n" + (currentColorRight -
				// oldColorRight));

				rightMotor.stop(true);
				double theta = odo.getXYT()[2];
				while (currentColorLeft - oldColorLeft <= 19) {
					if (rightMotor.getAcceleration() != CORRECTOR_SPEED
							|| leftMotor.getAcceleration() != CORRECTOR_SPEED) {
						setSpeedAccel(CORRECTOR_SPEED, TURN_ACCELERATION);
					}

					if (angleDiff(odo.getXYT()[2], theta) > 25) {
						rightMotor.stop(true);
						leftMotor.stop();
						leftMotor.rotate(-1 * Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), true);
						rightMotor.rotate(-1 * Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), false);
						leftMotor.forward();
						rightMotor.forward();
						break;
					}
					oldColorLeft = currentColorLeft;
					colorLeft.fetchSample(colorBufferLeft, 0);
					currentColorLeft = colorBufferLeft[0] * 1000;
					// System.out.println(colorLeft-oldColorLeft);
				}

				setSpeedAccel(FORWARD_SPEED, TURN_ACCELERATION);

				if (ySet) {
					odo.setX(Ev3Boot.getTileSize() + SENSOR_OFFSET);
					odo.setTheta(90);
					xSet = true;
					stopMotors();
					break;
				} else {
					odo.setY(Ev3Boot.getTileSize() + SENSOR_OFFSET);
					odo.setTheta(0);
					ySet = true;
				}
				stopMotors();

				leftMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), true);
				rightMotor.rotate(Navigator.convertDistance(Ev3Boot.getWheelRad(), 5), false);
				Navigator.turnTo(90);
				leftMotor.forward();
				rightMotor.forward();
				oldColorLeft = currentColorLeft;

				leftMotor.setSpeed(Navigator.getForwardSpeed());
				rightMotor.setSpeed(Navigator.getForwardSpeed());
				leftMotor.forward();
				rightMotor.forward();

				try {
					Thread.sleep(400);
					colorRight.fetchSample(colorBufferRight, 0);
					currentColorRight = colorBufferRight[0] * 1000;
					colorLeft.fetchSample(colorBufferLeft, 0);
					currentColorLeft = colorBufferLeft[0] * 1000;
					oldColorLeft = currentColorLeft;
					oldColorRight = currentColorRight;

				} catch (InterruptedException e) {
				}
				continue;
			}

		}
		Navigator.travelTo(1.5, 1.5, 3, false);
		Navigator.turnTo(0);

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

	private static double angleDiff(double first, double second) {
		double phi = Math.abs(first - second) % 360;
		phi = phi > 180 ? 360 - phi : phi;
		return phi;
	}
}
