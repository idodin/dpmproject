package ca.mcgill.ecse211.Ev3Boot;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * Abstract class that takes care of all motors basic operations. These
 * operations include, setting both motors on forward, moving forward by
 * specified angular rotation or distance, setting both motors accelerations,
 * setting both motors speeds, stopping both motors, turning by specified angle
 * and turning to a specified angle.
 * 
 * Motors, speed, acceleration, track size, wheel radius and tile size are all
 * initialize and should only be modified in this class.
 * 
 * This class should be extended by all class that needs to make operations on
 * any motors.
 */
public abstract class MotorController {

	public static Odometer odo = null;
	public static final int FORWARD_ACCEL = 350;
	public static final int FORWARD_SPEED = 200;
	public static final int TURN_SPEED = 170;
	public static final int LOCALIZER_SPEED = 250;

	public static final int CORRECTOR_SPEED = 30;
	public static final int LOCALIZER_ACCELERATION = 1000;
	public static final double SENSOR_OFFSET = 13.3;

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor BigArmHook = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3MediumRegulatedMotor armHook = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// Configuration Objects
	public static final double WHEEL_RAD = 2.103;
	public static final double TRACK = 9.92;
	public static final double TILE_SIZE = 30.48;

	/**
	 * Stops the right and left motors at the same time
	 */
	public static void stopBoth() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * Set speed of right and left motors to specified value at the same time. To
	 * execute this task it is safer to stop the motors before changing speed.
	 * 
	 * @param speed: the speed which the user desires the motor to be set in wheel
	 *        angle per second
	 */
	public static void setSpeeds(int speed) {
		if (leftMotor.getSpeed() == TURN_SPEED && rightMotor.getSpeed() == TURN_SPEED) {
			return;
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			m.setSpeed(speed);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// do nothing
		}
	}

	/**
	 * Set acceleration of right and left motors to specified value at the same
	 * time. To execute this task it is safer to stop the motors before changing
	 * acceleration.
	 * 
	 * @param accel: the acceleration which the user desires the motor to be set in
	 *        wheel angle per second^2
	 */
	public static void setAccels(int accel) {
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			m.setAcceleration(accel);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// do nothing
		}
	}

	/**
	 * Move forward or backward in straight line by specified distance.
	 * 
	 * @param dist: Distance the user desire the robot to move, positive to move
	 *        forward and negative to move backward
	 */
	public static void forwardBy(double dist) {
		leftMotor.rotate((dist < 0 ? -1 : 1) * convertDistance(WHEEL_RAD, dist > 0 ? dist : -1 * dist), true);
		rightMotor.rotate((dist < 0 ? -1 : 1) * convertDistance(WHEEL_RAD, dist > 0 ? dist : -1 * dist), false);
	}

	/**
	 * Makes right and left motor rotate forward.
	 */
	public static void bothForwards() {
		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * Set Speed and Acceleration for both motors at the same time.
	 * 
	 * @param speed
	 * @param accel
	 */
	public static void setSpeedAccel(int speed, int accel) {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(accel);
			motor.setSpeed(speed);
		}
		try {
			Thread.sleep(300);
		} catch (InterruptedException e) {
			// do nothing
		}
	}

	/**
	 * This method makes the robot turn to the specified bearing.
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

		setSpeedAccel(TURN_SPEED, FORWARD_ACCEL);

		double[] currentPosition = odo.getXYT();

		double deltaT = (((theta - currentPosition[2]) % 360) + 360) % 360;

		if (deltaT < 180) {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaT), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaT), false);
		} else {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 360 - deltaT), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 360 - deltaT), false);
		}

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

	}

	/**
	 * Turn by the specified angle theta, can be both positive or negative
	 * 
	 * @param theta: amount of degree the robot has to turn.
	 * @param clockwise: turn clockwise if theta is positive
	 * @param blocking: should we wait for the turn to finish to do other operations
	 *        (immediate return). True for non immediate return and false for
	 *        immediate return.
	 * @param SPEED: speed of motor in angle/sec
	 */
	public static void turnBy(double theta, boolean clockwise, boolean blocking, int SPEED) {

		setSpeeds(SPEED);

		if (clockwise == false) {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), !blocking);
		} else {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), !blocking);
		}

	}

	/**
	 * Turn by the specified angle theta, can be both positive or negative
	 * 
	 * @param theta: amount of degree the robot has to turn.
	 * @param clockwise: turn clockwise if theta is positive
	 * @param blocking: should we wait for the turn to finish to do other operations
	 *        (immediate return). True for non immediate return and false for
	 *        immediate return.
	 */
	public static void turnBy(double theta, boolean clockwise, boolean blocking) {

		if (clockwise == false) {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), !blocking);
		} else {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), !blocking);
		}

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius: wheel radius
	 * @param distance: distance to be converted to total rotation
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the rotation of each wheel
	 * needed to rotate that angle
	 * 
	 * @param radius: wheel radius
	 * @param width: Track width
	 * @param angle: angle to be converted to wheel rotation
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Difference between 2 angles. Always return a value between 0 and 359 degrees
	 * 
	 * @param first: first angle
	 * @param second: second angle
	 * @return angle difference
	 */
	public static double angleDiff(double first, double second) {
		double phi = Math.abs(first - second) % 360;
		phi = phi > 180 ? 360 - phi : phi;
		return phi;
	}

}
