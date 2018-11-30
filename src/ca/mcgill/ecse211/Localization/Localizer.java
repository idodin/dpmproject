package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Ev3Boot.MotorController;
import ca.mcgill.ecse211.Ev3Boot.Wifi;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class role is to localize the robots position using the ultrasonic
 * sensor and light sensor assuming it is in a corner. This class is also used
 * to correct heading and postion assuming it is close from a line intersection.
 * 
 * This class contains methods to execute Falling Edge Ultrasonic, light
 * localization and circle localization.
 * 
 * This class extends "MotorController" to facilitate all the motor logic.
 * 
 * Methods in this class are called by the Ev3Boot class to execute
 * localization. Also called by turnAroundTree method to relocalize at an
 * intersection before trying to detect rings to make sure it is in a good
 * position.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 *
 */
public class Localizer extends MotorController {

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

	private static int corner;

	private static int fallingDistance = 40;
	private static int faceToTheWallDistance = 80;

	private static double phi;

	// var for cirlce localization
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

	/**
	 * This method execute Falling Edge Localization using the Ultrasonic Sensor.
	 * 
	 * It is called by the Ev3Boot class at the start of the run to roughly correct
	 * the heading of the robot. A more precise heading correction will be done
	 * after.
	 * 
	 * First, the robot turns clockwise until distance is greater than faceToWallD *
	 * distance. This ensure that if the robot start facing the wall, it only start
	 * looking for falling edge after turning away from the wall. Then the robot
	 * continue turning clockwise until 2 consecutive readings of distance is
	 * smaller than fallingDistance. Stop the motors when the first falling edge is
	 * found and record the angle as angle "a". Then the robot start turning counter
	 * clockwise until 2 consecutive readings of distance is smaller than
	 * fallingDistance. Stop the motors and record the angle as angle "b". Using "a"
	 * and "b", compute the angle correction to determine current angle. Finally,
	 * turn to the 0 degree heading.
	 * 
	 * @throws OdometerExceptions
	 */
	public static void localizeFE() throws OdometerExceptions {

		// Initialize variables
		double a, b, correction;
		a = b = 0;

		int dist, lastdist;
		dist = lastdist = Integer.MAX_VALUE;

		setSpeedAccel(LOCALIZER_SPEED, LOCALIZER_ACCELERATION);

		// Get Odometer Instance
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		// Start turning, this will be stopped when Falling Edges are detected.
		turnBy(1000, true, false, LOCALIZER_SPEED);

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
		stopBoth();
		turnBy(1000, false, false, LOCALIZER_SPEED);

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
				if (angleDiff(odo.getXYT()[2], a) < 60 || angleDiff(odo.getXYT()[2], a) > 140)
					continue;
				Sound.beep();
				b = odo.getXYT()[2];
				break;
			}
			lastdist = dist;
		}

		Sound.beepSequence();

		stopBoth();

		setSpeeds(FORWARD_SPEED);

		// Correct theta and orientate to 0.
		if (a < b) {
			correction = 45 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			turnTo(0);

		} else if (a >= b) {
			correction = 225 - (a + b) / 2;
			odo.setTheta(180 + correction + odo.getXYT()[2]);
			turnTo(0);
		}

	}

	/**
	 * This method is used to correct the robots x and y. The robot starts at a 0
	 * degree angle and it goes forward until it detects a line. The odometer's y
	 * value is then corrected using the offset between the sensors and the
	 * wheelbase as well as the starting corner. Once the first line is detected the
	 * method rotates the robot by 90 degree using the turnBy method from the
	 * Navigator class and makes the robot go forward again until it detects a line,
	 * then the odometer's x is then corrected using the offset between the sensors
	 * and the wheelbase as well as the starting corner.
	 * 
	 * @throws OdometerExceptions
	 */
	public static void localizeColor() throws OdometerExceptions {
		int map_x = 14;
		int map_y = 8;

		boolean firstSet = false;
		boolean secondSet = false;
		corner = Wifi.getCorner();
		setSpeedAccel(FORWARD_SPEED, FORWARD_ACCEL);

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			throw e;
		}

		bothForwards();

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

			if (currentColorLeft - oldColorLeft > 19 && Navigator.pollMultiple(false, 15)) {

				leftMotor.stop(true);
				double theta = odo.getXYT()[2];
				while (currentColorRight - oldColorRight <= 19) {

					if (angleDiff(odo.getXYT()[2], theta) > 25) {
						stopBoth();
						turnBy(15, true, true, CORRECTOR_SPEED);
						bothForwards();
						break;
					}
					oldColorRight = currentColorRight;
					colorRight.fetchSample(colorBufferRight, 0);
					currentColorRight = colorBufferRight[0] * 1000;
				}

				if (firstSet) {
					stopBoth();
					setSpeedAccel(FORWARD_SPEED, FORWARD_ACCEL);
					switch (corner) {
					case 0:
						odo.setX(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(90);
						Navigator.travelTo(1, 1, 2, false);
						Navigator.turnTo(0);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(1.5, 1.5, 2, false);
						break;
					case 1:
						odo.setY(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(0);
						Navigator.travelTo(map_x, 1, 2, false);
						Navigator.turnTo(0);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(map_x - 0.5, 1.5, 2, false);
						break;
					case 2:
						odo.setX(map_x * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(270);
						Navigator.travelTo(map_x, map_y, 2, false);
						Navigator.turnTo(180);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(map_x - 0.5, map_y - 0.5, 2, false);
						break;
					case 3:
						odo.setY(map_y * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(180);
						Navigator.travelTo(1, map_y, 2, false);
						Navigator.turnTo(180);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(1.5, map_y - 0.5, 2, false);
						break;
					}

					secondSet = true;
					break;
				} else {
					stopBoth();
					setSpeedAccel(FORWARD_SPEED, FORWARD_ACCEL);
					switch (corner) {
					case 0:
						odo.setY(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(0);
						break;
					case 1:
						odo.setX(map_x * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(270);
						break;
					case 2:
						odo.setY(map_y * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(180);
						break;
					case 3:
						odo.setX(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(90);
						break;
					}
					firstSet = true;
				}
				stopBoth();

				forwardBy(-25);
				turnBy(90, true, true);
				bothForwards();
				oldColorLeft = currentColorLeft;

				setSpeeds(FORWARD_SPEED);
				bothForwards();

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

			} else if (currentColorRight - oldColorRight > 19 && Navigator.pollMultiple(true, 15)) {

				rightMotor.stop(true);
				double theta = odo.getXYT()[2];
				while (currentColorLeft - oldColorLeft <= 19) {

					if (angleDiff(odo.getXYT()[2], theta) > 25) {
						stopBoth();
						turnBy(15, false, true, CORRECTOR_SPEED);
						forwardBy(-5);
						bothForwards();
						break;
					}

					oldColorLeft = currentColorLeft;
					colorLeft.fetchSample(colorBufferLeft, 0);
					currentColorLeft = colorBufferLeft[0] * 1000;
					// tem.out.println(colorLeft-oldColorLeft);
				}

				if (firstSet) {
					stopBoth();
					setSpeedAccel(FORWARD_SPEED, FORWARD_ACCEL);
					switch (corner) {
					case 0:
						odo.setX(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(90);
						Navigator.travelTo(1, 1, 2, false);
						Navigator.turnTo(0);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(1.5, 1.5, 2, false);
						break;
					case 1:
						odo.setY(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(0);
						Navigator.travelTo(map_x, 1, 2, false);
						Navigator.turnTo(0);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(map_x - 0.5, 1.5, 2, false);
						break;
					case 2:
						odo.setX(map_x * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(270);
						Navigator.travelTo(map_x, map_y, 2, false);
						Navigator.turnTo(180);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(map_x - 0.5, map_y - 0.5, 2, false);
						break;
					case 3:
						odo.setY(map_y * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(180);
						Navigator.travelTo(1, map_y, 2, false);
						Navigator.turnTo(180);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						Navigator.travelTo(1.5, map_y - 0.5, 2, false);
						break;
					}

					secondSet = true;
					break;
				} else {
					stopBoth();
					setSpeedAccel(FORWARD_SPEED, FORWARD_ACCEL);
					switch (corner) {
					case 0:
						odo.setY(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(0);
						break;
					case 1:
						odo.setX(map_x * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(270);
						break;
					case 2:
						odo.setY(map_y * TILE_SIZE - SENSOR_OFFSET);
						odo.setTheta(180);
						break;
					case 3:
						odo.setX(TILE_SIZE + SENSOR_OFFSET);
						odo.setTheta(90);
						break;
					}
					firstSet = true;
				}
				stopBoth();

				forwardBy(-20);
				turnBy(90, true, true);
				bothForwards();
				oldColorLeft = currentColorLeft;

				setSpeeds(FORWARD_SPEED);
				bothForwards();

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

	}

	/**
	 * This method is used to perform the circle localization. For this type of
	 * localization to succeed, the method assumes that the robot is close enough to
	 * a grid intersection. First, the robot rotates 360 degrees while detecting the
	 * black line it crosses. At each crossed line, it records its angle using the
	 * theta value from the odometer. It stores the angles in four different
	 * variables, firstLine, secondLine, thirdLine, fourhtLine, which each represent
	 * the intersection with the positive and negative x and y axis. For the logic
	 * of this method to work the order of the encountered lines is important, so we
	 * chose to rotate counter clockwise and start at a 45-degree heading
	 * consistently. Once it has encountered all four lines, it corrects the
	 * coordinates of the robot using the x and y passed in to the method and the
	 * recorded angles
	 * 
	 * @param x: x-coordinate of the intersection we are correcting to
	 * @param y: x-coordinate of the intersection we are correcting to
	 */
	public static void circleLocalize(double x, double y) {

		double newSensorOffset = -1 * SENSOR_OFFSET;

		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		turnTo(60);

		colorRight.fetchSample(colorBufferRight, 0);
		color = colorBufferRight[0] * 1000;

		counter = 1;

		setSpeeds(TURN_SPEED);
		rightMotor.rotate(Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);
		leftMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);

		while (counter <= 4) {
			lastColor = color;

			colorRight.fetchSample(colorBufferRight, 0);
			color = colorBufferRight[0] * 1000;
			if (color - lastColor > 5 && Navigator.pollMultiple(true, 1)) {
				temp = Ev3Boot.odo.getXYT()[2];
				Sound.beep();
				if (counter == 1) {
					firstLine = temp;
				} else if (counter == 2) {
					secondLine = temp;
				} else if (counter == 3) {
					thirdLine = temp;
				} else if (counter == 4) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					fourthLine = temp;
				}
				counter++;
			}
		}

		// we are to the right
		if (Math.abs(secondLine - firstLine) > Math.abs(fourthLine - firstLine)) {
			newSensorOffset = -1 * newSensorOffset;
		}

		xOrigin = ((x * TILE_SIZE)) + (newSensorOffset * Math.cos(Math.toRadians(Math.abs(firstLine - thirdLine)) / 2));
		yOrigin = (y * TILE_SIZE)
				+ (-1 * SENSOR_OFFSET * Math.cos(Math.toRadians(Math.abs(secondLine - fourthLine)) / 2));

		odo.setX(xOrigin);
		odo.setY(yOrigin);

		Navigator.travelTo(x, y, 2, false);
	}
}
