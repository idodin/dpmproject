
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to keep track of the robot's position
 *  and heading, by using it's motors' tacho counts.
 * This class implements runnable, ans is run in parallel as long
 * as the motor is travelling.
 * 
 */
public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private int lastTachoL;
	private int lastTachoR;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double distL;
	private double distR;

	private final double WHEEL_RAD;

	private double dX;
	private double dY;

	private double deltaD;
	private double deltaT;

	private double theta = Math.toRadians(OdometerData.getOdometerData().getXYT()[2]);
	private double newTheta;
	
	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK      the wheel base of the robot
	 * @param WHEEL_RAD  the wheel radius
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
													// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK      the wheel base of the robot
	 * @param WHEEL_RAD  the wheel radius
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * To update the robot's heading, this method uses the gyroscope.
	 * To update the x and y coordinates it uses the change in the motor's 
	 * tacho counts to find the distance covered by each wheel. 
	 * Using those two distances it computes the displacement of the robot.
	 * Afterwards, the x and y  variations are recorded
	 *  using the displacement and the variation in theta,
	 * Finally, it uses the update method from OdometerData to keep track of the new values.
	 */
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			theta = odo.getXYT()[2];
			updateStart = System.currentTimeMillis();
			
			Ev3Boot.gyroAngle.fetchSample(Ev3Boot.getGyroBuffer(), 0);
			newTheta = ((Ev3Boot.getGyroBuffer()[0] % 360) + 360) % 360;

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// Calculate Left and Right Wheel Distances.
			distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastTachoL) / 180;
			distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastTachoR) / 180;

			lastTachoL = leftMotorTachoCount;
			lastTachoR = rightMotorTachoCount;

			
			deltaD = (distL + distR) * 0.5;
			deltaT = newTheta - theta;

			
			dX = deltaD * Math.sin(Math.toRadians(newTheta));
			dY = deltaD * Math.cos(Math.toRadians(newTheta));
			
			odo.update(dX, dY, deltaT);
			
			
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}
