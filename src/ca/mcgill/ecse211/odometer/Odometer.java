/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.FinalProject.FinalProject;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
	 * variables once.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
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
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
//			theta = Math.toRadians(odo.getXYT()[2]);
			theta = odo.getXYT()[2];
			updateStart = System.currentTimeMillis();
			
			FinalProject.gyroAngle.fetchSample(FinalProject.gyroBuffer, 0);
			newTheta = ((FinalProject.gyroBuffer[0] % 360) + 360) % 360;

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// Calculate Left and Right Wheel Distances.
			distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastTachoL) / 180;
			distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastTachoR) / 180;

			lastTachoL = leftMotorTachoCount;
			lastTachoR = rightMotorTachoCount;

			
			deltaD = (distL + distR) * 0.5;
//			deltaT = (distL - distR) / TRACK;
			deltaT = newTheta - theta;
			

//			theta += deltaT;


//			dX = deltaD * Math.sin(theta);
//			dY = deltaD * Math.cos(theta);
			
			dX = deltaD * Math.sin(Math.toRadians(newTheta));
			dY = deltaD * Math.cos(Math.toRadians(newTheta));
			
			
//			odo.update(dX, dY, 180 * deltaT / Math.PI); // Convert back to degrees.
			odo.update(dX, dY, deltaT);
			
//			FinalProject.gyroAngle.fetchSample(FinalProject.gyroBuffer, 0);
//			FinalProject.odometer.setTheta((double) FinalProject.gyroBuffer[0]);

//			odo.update(dX, dY, 0);
//			odo.setTheta(FinalProject.angle);
			
			
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
