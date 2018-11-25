package ca.mcgill.ecse211.Ev3Boot;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public abstract class MotorController {
	public static Odometer odo = null;
	public static final int FORWARD_SPEED = 115;
	public static final int TURN_SPEED = 75;
	public static final int CORRECTOR_SPEED = 30;

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor BigArmHook = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3MediumRegulatedMotor armHook = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// Configuration Objects
	public static final double WHEEL_RAD = 2.103;
	public static final double TRACK = 9.45;
	public static final double TILE_SIZE = 30.48;
	
	public static void stopBoth() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	public static void setSpeeds(int speed) {
//		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
//			m.stop();
//		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			m.setSpeed(speed);
		}
		try {
			Thread.sleep(300);
		} catch (InterruptedException e) {
			// do nothing
		}
	}

	public static void setAccels(int accel) {
//		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
//			m.stop();
//		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			m.setAcceleration(accel);
		}
		try {
			Thread.sleep(300);
		} catch (InterruptedException e) {
			// do nothing
		}
	}

	public static void forwardBy(double dist) {
		leftMotor.rotate((dist < 0 ? -1 : 1) * convertDistance(WHEEL_RAD, dist > 0 ? dist : -1 * dist), true);
		rightMotor.rotate((dist < 0 ? -1 : 1) * convertDistance(WHEEL_RAD, dist > 0 ? dist : -1 * dist), false);
	}

	public static void bothForwards() {
//		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
//			m.stop();
//		}
		leftMotor.stop(true);
		rightMotor.stop(false);

		try {
			Thread.sleep(300);
		} catch (InterruptedException e) {

		}

		for (EV3LargeRegulatedMotor m : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			m.forward();
		}
	}

	/**
	 * Set Speed and Acceleration for both motors.
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
	 * This method makes the robot turn to the specified bearing. Mainly called by
	 * the travelTo() method and LightLocalize() method.
	 * 
	 * In a loop, calculate the difference between current heading and desired
	 * heading, if the difference is smaller than "turnError" stop turning and put
	 * motors back on forward.
	 * 
	 * Always turn the smallest angle.
	 * 
	 * @param theta: Bearing for the robot to readjust its heading to.
	 */
	/**
	 * This method makes the robot turn to the specified bearing.
	 * 
	 * @param theta Bearing for the robot to readjust its heading to.
	 */
	public static void turnTo(double theta) {

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		double[] currentPosition = odo.getXYT();

		double deltaT = (((theta - currentPosition[2]) % 360) + 360) % 360;

//		System.out.println("TURNING \nCurrent theta: " + currentPosition[2] + "\nDesired theta: " + theta
//				+ "\nDelta theta (clockwise): " + deltaT);

		if (deltaT < 180) {
			leftMotor.rotate(convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), deltaT), true);
			rightMotor.rotate(-convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), deltaT), false);
		} else {
			leftMotor.rotate(-convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), 360 - deltaT), true);
			rightMotor.rotate(convertAngle(Ev3Boot.getWheelRad(), Ev3Boot.getTrack(), 360 - deltaT), false);
		}

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the rotation of each wheel
	 * needed to rotate that angle
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
