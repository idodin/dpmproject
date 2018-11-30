package ca.mcgill.ecse211.Navigation;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Ev3Boot.MotorController;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
//import ca.mcgill.ecse211.odometer.Odometer;
//import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class contains methods to allow the robot to navigate to specified
 * waypoint, to turn to specific angle, travel until a line is detected and
 * correct its orientation.
 * 
 * It is mainly called by the Ev3Boot and GameLogic class to navigate to
 * strategic point. It is also called by other LightLocalization class to
 * navigate to a line intersection.
 * 
 * This class extends "MotorController" to facilitate all the motor logic.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 * @author Chaimae Fahmi
 *
 */
public class Navigator extends MotorController {

	private static final int BLACK = 300;
	private static final int errorMargin = 150;

	private static final int TURN_ERROR = 1;
	private static SampleProvider colorSensorLeft = Ev3Boot.getColorLeft();
	private static SampleProvider colorSensorRight = Ev3Boot.getColorRight();
	private static float[] colorLeftBuffer = Ev3Boot.getColorLeftBuffer();
	private static float[] colorRightBuffer = Ev3Boot.getColorRightBuffer();

	private static float colorLeft;
	private static float colorRight;
	private static float oldColorLeft;
	private static float oldColorRight;
	private static final long CORRECTION_PERIOD = 10;
	static long correctionStart;
	static long correctionEnd;

	private static Odometer odo;
	private static double[] currentPosition;
	private static double[] lastPosition;
	private static boolean isNavigating;
	private static SampleProvider usDistance = Ev3Boot.getUSDistance();
	private static float[] usData = Ev3Boot.getUSData();

	private static double ReOrientDistance;
	private static double distanceDifference;
	private static double distance;

	private static boolean updatePosition;
	private static double deltaX;
	private static double deltaY;

	private static double baseAngle;
	private static double adjustAngle;

	private static double newX;
	private static double newY;
	private static double newTheta;

	private static double phi;

	/**
	 * This method makes the robot go forward until it detects a line. To detect the
	 * line it uses the checkLines method. Once it has detected a line, it uses the
	 * cosine and sine of the robot’s heading to decide which coordinate has to be
	 * modified. We only modify one coordinate at a time because our robot only
	 * performs straight traveling. It corrects the x or y coordinate to the closest
	 * line.
	 */
	public static void travelUntil() {
		long currentTime = System.currentTimeMillis();
		if (currentTime - Ev3Boot.demoStart >= 60000) {
			// Sound.buzz();
		}

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		// Set wheel accelerations.
		setSpeedAccel(FORWARD_SPEED / 2, 500);

		isNavigating = true;

		System.out.println("Start forward");

		bothForwards();

		while (isNavigating) {

			correctionStart = System.currentTimeMillis();

			if (checkForLines()) {
				System.out.println("Lines checked!");
				stopBoth();

				int yInc = Math.round((float) Math.cos(Math.toRadians(currentPosition[2])));
				int xInc = Math.round((float) Math.sin(Math.toRadians(currentPosition[2])));

				if (updatePosition) {

					if (xInc < 0) {
						newX = roundToNearestTileSize(currentPosition[0] + Localizer.SENSOR_OFFSET)
								- Localizer.SENSOR_OFFSET;
						newTheta = 270;
					} else if (xInc > 0) {
						newX = roundToNearestTileSize(currentPosition[0] - Localizer.SENSOR_OFFSET)
								+ Localizer.SENSOR_OFFSET;
						newTheta = 90;
					} else {
						newX = currentPosition[0];
					}

					if (yInc < 0) {
						newY = roundToNearestTileSize(currentPosition[1] + Localizer.SENSOR_OFFSET)
								- Localizer.SENSOR_OFFSET;
						newTheta = 180;
					} else if (yInc > 0) {
						newY = roundToNearestTileSize(currentPosition[1] - Localizer.SENSOR_OFFSET)
								+ Localizer.SENSOR_OFFSET;
						newTheta = 0;
					} else {
						newY = currentPosition[1];
					}

					odo.setXYT(newX, newY, newTheta);

					isNavigating = false;
				}
			}

			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {

				}
			}

		}

	}

	/**
	 * 
	 * This method execute navigation to a specific point. The method continuously
	 * polls the 2 rear light sensors. In the case that a line is detected by one of
	 * them, its respective motor is stopped until the robot corrects its heading
	 * (the second light sensor detects a line). In a loop calculate distance
	 * between current position and arrival point, if the distance is smaller than
	 * treshHold, stop the motors. When the distance from arrival point is
	 * acceptable stop the motors and light localize if specified. When a line is
	 * crossed the robot corrects its odometry,
	 * 
	 * @param x: x coordinate to navigate to
	 * @param y: y coordinate to navigate to
	 * @param treshHold: acceptable error distance
	 * @param checkLine: should the robot detect line to correct itself or not
	 */
	public static void travelTo(double x, double y, int treshHold, boolean checkLine) {
		long currentTime = System.currentTimeMillis();
		if (currentTime - Ev3Boot.demoStart >= 60000) {
			// Sound.buzz();
		}

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		currentPosition = odo.getXYT();
		lastPosition = odo.getXYT();

		// Set wheel accelerations.
		if (!checkLine) {
			setSpeedAccel(FORWARD_SPEED * 2, 500);
		} else {
			setSpeedAccel(FORWARD_SPEED, 500);
		}

		isNavigating = true;

		// Calculate Turn Angle

		deltaX = x * TILE_SIZE - currentPosition[0];
		deltaY = y * TILE_SIZE - currentPosition[1];
		distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		if (distance < treshHold) {
			leftMotor.stop(true);
			rightMotor.stop();
			isNavigating = false;

			return;
		}

		if (deltaX == 0 && deltaY != 0) {
			turnTo(deltaY < 0 ? 180 : 0);
		} else {
			baseAngle = deltaX > 0 ? 90 : 270;
			if ((deltaY > 0 && deltaX > 0) || (deltaY < 0 && deltaX < 0)) {
				adjustAngle = -1 * Math.toDegrees(Math.atan(deltaY / deltaX));
			} else {
				adjustAngle = Math.toDegrees(Math.atan(Math.abs(deltaY) / Math.abs(deltaX)));
			}

			// OdometerCorrection.isCorrecting = false;
			turnTo((((baseAngle + adjustAngle + (distance > TILE_SIZE ? 5 : 0)) % 360) + 360) % 360);
			odo.setTheta(baseAngle + adjustAngle);
			// OdometerCorrection.isCorrecting = true;

		}

		bothForwards();

		while (isNavigating) {

			correctionStart = System.currentTimeMillis();

			if (checkLine)
				checkForLines();

			currentPosition = odo.getXYT();

			deltaX = x * TILE_SIZE - currentPosition[0];
			deltaY = y * TILE_SIZE - currentPosition[1];
			distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

			if (distance < treshHold) {
				leftMotor.stop(true);
				rightMotor.stop();
				isNavigating = false;
				if (treshHold > 4) {
					travelTo(x, y, 2, false);
				}

				return;
			}

			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {

				}
			}

		}

	}

	/**
	 * This method starts by fetching samples from the two light sensors, for each
	 * sensor it uses a differential filter, it fetches two samples and computes
	 * their difference which it then compares to a threshold. When crossing a line,
	 * If the two sensors are i.e both sensors detect the line, the method returns
	 * true. Otherwise it attempts to correct the robot’s heading. When only one of
	 * the sensors detects a line it stops the corresponding wheel, which makes the
	 * motor rotate in place. The robot keeps rotating until the other sensor
	 * detects the line as well, which brings the robot to a 0 degree heading. To
	 * limit the errors caused by crossing a vertical side line, the method stops
	 * both motors after turning more than 25 degrees and rotates back to the
	 * initial heading.
	 * 
	 * @return
	 */
	public static boolean checkForLines() {
		// Take previously retrieved values for colours as old colour values.
		oldColorLeft = colorLeft;
		oldColorRight = colorRight;

		// Fetch new colours.
		colorSensorLeft.fetchSample(colorLeftBuffer, 0);
		colorLeft = colorLeftBuffer[0] * 1000;

		colorSensorRight.fetchSample(colorRightBuffer, 0);
		colorRight = colorRightBuffer[0] * 1000;

		if (colorLeft - oldColorLeft > 19) {
			// If both sensors on line or black line was falsely detected.

			if (colorRight - oldColorRight > 19 || !pollMultiple(false, 15)) {
				return false;
			}

			leftMotor.stop(true);
			double theta = odo.getXYT()[2];

			// Keep turning Right Motor in cases where no line was detected AND/OR a black
			// line was
			// falsely detected.
			while (colorRight - oldColorRight <= 19) {
				updatePosition = true;
				double omega = odo.getXYT()[2];
				phi = angleDiff(theta, omega);
				// If we've been turning for more than degrees, stop, turn in opposite direction
				// and go back.
				if (phi > 25) {
					System.out.println(omega);
					rightMotor.stop(true);
					leftMotor.stop();
					turnBy(26, true, true);
					forwardBy(-7);
					updatePosition = false;
					break;
				}

				// Poll new colors
				oldColorRight = colorRight;
				colorSensorRight.fetchSample(colorRightBuffer, 0);
				colorRight = colorRightBuffer[0] * 1000;
			}

			bothForwards();
			currentPosition = odo.getXYT();

			// How much do you increment by?
			int yInc = Math.round((float) Math.cos(Math.toRadians(currentPosition[2])));
			int xInc = Math.round((float) Math.sin(Math.toRadians(currentPosition[2])));

			if (updatePosition) {

				if (xInc < 0) {
					newX = roundToNearestTileSize(currentPosition[0] + Localizer.SENSOR_OFFSET)
							- Localizer.SENSOR_OFFSET;
					newTheta = 270;
				} else if (xInc > 0) {
					newX = roundToNearestTileSize(currentPosition[0] - Localizer.SENSOR_OFFSET)
							+ Localizer.SENSOR_OFFSET;
					newTheta = 90;
				} else {
					newX = currentPosition[0];
				}

				if (yInc < 0) {
					newY = roundToNearestTileSize(currentPosition[1] + Localizer.SENSOR_OFFSET)
							- Localizer.SENSOR_OFFSET;
					newTheta = 180;
				} else if (yInc > 0) {
					newY = roundToNearestTileSize(currentPosition[1] - Localizer.SENSOR_OFFSET)
							+ Localizer.SENSOR_OFFSET;
					newTheta = 0;
				} else {
					newY = currentPosition[1];
				}

				odo.setXYT(newX, newY, newTheta);

				return true;
			}

		} else if (colorRight - oldColorRight > 19) {

			if (colorLeft - oldColorLeft > 19 || !pollMultiple(true, 15)) {
				return false;
			}

			rightMotor.stop(true);
			double theta = odo.getXYT()[2];
			while (colorLeft - oldColorLeft <= 19) {
				updatePosition = true;
				if (leftMotor.getSpeed() != CORRECTOR_SPEED) {
					// setSpeeds(CORRECTOR_SPEED);
				}

				double omega = odo.getXYT()[2];
				phi = angleDiff(theta, omega);
				// If we've been turning for more than degrees, stop, turn in opposite direction
				// and go back.
				if (phi > 25) {
					System.out.println(omega);
					rightMotor.stop(true);
					leftMotor.stop();
					turnBy(26, false, true);
					forwardBy(-7);
					updatePosition = false;
					break;
				}
				oldColorLeft = colorLeft;
				colorSensorLeft.fetchSample(colorLeftBuffer, 0);
				colorLeft = colorLeftBuffer[0] * 1000;
			}

			bothForwards();
			currentPosition = odo.getXYT();

			// How much do you increment by?
			int yInc = Math.round((float) Math.cos(Math.toRadians(currentPosition[2])));
			int xInc = Math.round((float) Math.sin(Math.toRadians(currentPosition[2])));

			if (updatePosition) {
				// yCount += yInc;
				// xCount += xInc;

				if (xInc < 0) {
					newX = roundToNearestTileSize(currentPosition[0] + Localizer.SENSOR_OFFSET)
							- Localizer.SENSOR_OFFSET;
					newTheta = 270;
				} else if (xInc > 0) {
					newX = roundToNearestTileSize(currentPosition[0] - Localizer.SENSOR_OFFSET)
							+ Localizer.SENSOR_OFFSET;
					newTheta = 90;
				} else {
					newX = currentPosition[0];
				}

				if (yInc < 0) {
					newY = roundToNearestTileSize(currentPosition[1] + Localizer.SENSOR_OFFSET)
							- Localizer.SENSOR_OFFSET;
					newTheta = 180;
				} else if (yInc > 0) {
					newY = roundToNearestTileSize(currentPosition[1] - Localizer.SENSOR_OFFSET)
							+ Localizer.SENSOR_OFFSET;
					newTheta = 0;
				} else {
					newY = currentPosition[1];
				}

				odo.setXYT(newX, newY, newTheta);

				return true;
			}

		}
		return false;
	}

	/**
	 * This method is used to make the robot travel in straight lines. It calls the
	 * travelTo method twice to first make the robot cover the distance on the y
	 * axis, then it calls the same method to cover the needed distance on the x
	 * axis.
	 * 
	 * @param x: x coordinate to navigate to
	 * @param y: y coordinate to navigate to
	 * @param threshold: acceptable distance error from final point
	 */
	public static void toStraightNavigator(double x, double y, int threshold) {
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}
		currentPosition = odo.getXYT();
		travelTo(currentPosition[0] / TILE_SIZE, y, threshold, true);
		travelTo(x, y, threshold, true);
	}

	/**
	 * This method is used to sample the sensors. It takes in an input boolean
	 * isRight which decides which sensor to sample. Then it uses the second
	 * parameter passed, the sampleCount, inside a loop to sample the sensors. After
	 * performing all the samples, it computes their average and compares it to
	 * check if it’s within the error margin. This method returns true if a line has
	 * been detected, false otherwise.
	 * 
	 * @param isRight: true to poll right sensor, false to poll left sensor
	 * @param sampleCount: sample amount to be polled
	 * @returntrue if a line has been detected, false otherwise.
	 */
	public static boolean pollMultiple(Boolean isRight, int sampleCount) {
		// int sampleCount = 15;
		float sum = 0;
		SampleProvider sample = isRight ? colorSensorRight : colorSensorLeft;
		float[] buffer = isRight ? colorRightBuffer : colorLeftBuffer;

		for (int i = 0; i < sampleCount; i++) {
			sample.fetchSample(buffer, 0);
			sum += buffer[0] * 1000;
		}

		float avg = sum / sampleCount;
		if (avg > BLACK - errorMargin && avg < BLACK + errorMargin) {
			return true;
		} else {
			return false;
		}

	}

	/**
	 * This method takes in a parameter value which it rounds to the closer tile
	 * size value.
	 * 
	 * @param value: value to be rounded
	 * @return value of closest tile size
	 */
	public static double roundToNearestTileSize(double value) {
		return TILE_SIZE * (Math.round(value / TILE_SIZE));
	}

}