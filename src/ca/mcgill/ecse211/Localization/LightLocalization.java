package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.FinalProject.FinalProject;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class LightLocalization {
	private static double sensorOffset = -13.0;
	static int counter = 1;
	static float color;
	static float lastColor;
	static double temp;
	static double yIntersectionplus;
	static double yIntersectionminus;
	static double xIntersectionplus;
	static double xIntersectionminus;
	static double deltatheta1;
	static double deltatheta2;
	static double xOrigin;
	static double yOrigin;
	static double y;
	static double x;
	static double tempDist;
	private static double TRACK = FinalProject.getTrack();
	private static double WHEEL_RAD = FinalProject.getWheelRad();
	private static final EV3LargeRegulatedMotor leftMotor = FinalProject.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = FinalProject.getRightmotor();
	private static SampleProvider SColor = FinalProject.getColorBack();
	static float[] data = FinalProject.getColorBufferBack();
	public static Odometer odo;
	
	/**
	 * Rotate on place until all 4 line are detected, calculations done to determine what is the current position.
	 * Navigate to the line intersection and  rotate to the 4th line heading and reset the gyro, 
	 * this has to be done to calculate where is the 0 degree heading.
	 * 
	 * Reset 2 times, ~4 sec to reset.
	 * Would be nice to find a way to reset only once and not twice.
	 * 
	 * Requirement: Needs to be close enough to an intersection so that when the robot rotates on place it detects all 4 lines.
	 * 				Needs to be in the lower left quadrant to correct angle, but does not have to be in the lower left quadrant to correct X and Y.
	 */
	public static void lightLocalize() {

		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		rightMotor.setSpeed(150);
		leftMotor.setSpeed(150);

		SColor.fetchSample(data, 0);
		color = data[0] * 1000;

		rightMotor.rotate(Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);
		leftMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 450), true);

		while (counter <= 4) {
			// System.out.println(color);
			lastColor = color;

			SColor.fetchSample(data, 0);
			color = data[0] * 1000;
			// System.out.println(color);
			if (color - lastColor > 5) {
			//	Sound.beep();
				temp = FinalProject.odo.getXYT()[2];
				System.out.println(temp);
				if (counter == 1) {
					yIntersectionminus = temp;
				}
				if (counter == 2) {
					xIntersectionplus = temp;
				}
				if (counter == 3) {
					yIntersectionplus = temp;	
				}
				if (counter == 4) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					
					xIntersectionminus = temp;
				}
				counter++;
			}
		}
		
		xOrigin = sensorOffset * Math.cos(Math.toRadians((yIntersectionminus - yIntersectionplus) / 2));
		yOrigin = sensorOffset * Math.cos(Math.toRadians(xIntersectionplus - xIntersectionminus) / 2);
		
		FinalProject.odo.setX(xOrigin);
		FinalProject.odo.setY(yOrigin);
		
		Navigator.travelTo(0, 0);	
		
		Navigator.turnTo(xIntersectionminus);
		
		FinalProject.gyro.reset();
	
		Navigator.turnTo((180+(xIntersectionplus-xIntersectionminus)/2)%360);
		
		FinalProject.gyro.reset();
		
//		System.out.println("x final:" + odo.getXYT()[0]);
//		System.out.println("y final:" + odo.getXYT()[1]);
//		System.out.println("t final:" + odo.getXYT()[2]);

		Button.waitForAnyPress();
		System.exit(0);
	}
}
