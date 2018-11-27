package ca.mcgill.ecse211.RingRetrieval;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Ev3Boot.MotorController;
import ca.mcgill.ecse211.Navigation.Navigator;
import lejos.hardware.Sound;

/**
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * 
 * Once the motor has reached the ring, this method uses
 *  a computed gaussian distribution and the light sensor
 *   to determine the ring color
 * @author Hieu Chau Nguyen
 *
 */
public class CheckColor extends MotorController{

//	private static final EV3LargeRegulatedMotor leftMotor  = Ev3BootGrasping.getLeftmotor();
//	private static final EV3LargeRegulatedMotor rightMotor = Ev3BootGrasping.getRightmotor();
//	private static final EV3LargeRegulatedMotor BigArmHook = Ev3BootGrasping.getBigArmHook();
//	private static final EV3LargeRegulatedMotor armHook    = Ev3BootGrasping.getArmHook();
//	
//	private static double wheelrad = Ev3BootGrasping.getWheelRad();
//	private static double track    = Ev3BootGrasping.getTrack();
	private static SampleProvider colorSample = Ev3Boot.getColorFront();
	private static float[] colorBuffer = Ev3Boot.getColorBufferFront();
	private static double minimum_value = Ev3Boot.getColorMin();
	private static double approach_dist = -5.0;
	//How often a colour is detected, and at which elevation? 
	private static int[][] colorCount;

	// Normalized mean, std*modifier_for_detection
	static double[][][] COLOR = {
			//ORANGE
			{{0.912223734,0.2896962706, 0.2896962706},{0.018521652*3,0.045192704*3,0.030158541*3}},
			//YELLOW
			{{0.865878045,0.405933976,0.287293334},{0.009333934*3,0.031746029*3,0.046546512*3}},
			//GREEN
			{{0.449623982,0.8484381058,0.2279268786},{0.057517323*4,0.048271431*4,0.033977313*4}},
			//BLUE
			{{0.155323995,0.6446263948,0.7485527822},{0.024216451*4,0.094028999*4,0.056205047*4}}
	};
	
	
	// Which color is detected.
	// 0: NOTHING
	// 1: ORANGE
	// 2: YELLOW 
	// 3: GREEN 
	// 4: BLUE 
	private static int detect = 0;
	// At what elevation
	// 0: NOTHING
	// 1: LOW
	// 2: HIGH
	private static int elevation;
	
	
	private static boolean detected = false;

	
	/**
	 * Check if the robot finished detecting the color
	 * @return Boolean value true if color detection is finished, false otherwise.
	 */
	public static boolean finishDetect() {
		return detected;
	}
	
	/**
	 * Restart the checker for stability reason
	 */
	public static void restartChecker() {
		detected = false; 
		colorCount = new int[][] 
				{{0, 0, 0, 0},
				 {0, 0, 0, 0}};
	}
	
	/**
	 * Get the value of the detected Color
	 * @return Integer value of the colour, 0 if no colour has been detected.
	 */
	public static int getDetectedColor() {
		return detect;
	}
	
	
	/**
	 * Get the elevation of the detected colour
	 * @return 1 for low elevation, 2 for high elevation, 0 if no colour has been detected.
	 */
	public static int getElevation() {
		return elevation;
	}
	
	/**
	 * This method assumes the robot is close enough to the ring,
	 * It makes the robot slightly move around to record multiple samples
	 * using the fetchColor method.
	 * It sets the variable detect to the value of the detected ring
	 */
	public static void colorDetection() {
		
		restartChecker();
		
		
		//approach the tree
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(Navigator.convertDistance(WHEEL_RAD, approach_dist),true);
		rightMotor.rotate(Navigator.convertDistance(WHEEL_RAD, approach_dist),false);
		
		//Detect higher elevation first.
		BigArmHook.setSpeed(100);
		BigArmHook.rotateTo(40);
		
		int step = 4; // color detection steps for the robot
		int elev = 1; // elevation being checked
		

		int max[]       = {0,0};
		int maxHolder[] = {0,0};
		int sample[][] = colorCount;
		
		do {

			fetchColor(elev);
			
			// Budging the robot
			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);
			
			if(step >= 4) {
				leftMotor.rotate (-Navigator.convertAngle(WHEEL_RAD, TRACK, 5.0), true);
				rightMotor.rotate (Navigator.convertAngle(WHEEL_RAD, TRACK, 5.0), false);
			}
			else if(step >= 3) {
				leftMotor.rotate  (Navigator.convertAngle(WHEEL_RAD, TRACK, 30.0), true);
				rightMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 30.0), false);
			}
			else if(step >= 1) {
				leftMotor.rotate  (Navigator.convertAngle(WHEEL_RAD, TRACK, 5.0), true);
				rightMotor.rotate(-Navigator.convertAngle(WHEEL_RAD, TRACK, 5.0), false);
			}
			// Switch to detecting low elevation.
			else {

				for(int j = 0; j < 4; j++) 
					if(colorCount[elev][j] > max[elev]) {
						maxHolder[elev] = j+1;
						max[elev] = colorCount[elev][j];
					}
				
				if(maxHolder[elev] != 0) {
					detect = maxHolder[elev];
					elevation = elev + 1;
					break;
				}
				if(elev == 1){
					leftMotor.rotate (-Navigator.convertAngle(WHEEL_RAD, TRACK, 35.0), true);
					rightMotor.rotate (Navigator.convertAngle(WHEEL_RAD, TRACK, 35.0), false);
					

					leftMotor.rotate (-Navigator.convertDistance(WHEEL_RAD, 1.0), true);
					rightMotor.rotate(-Navigator.convertDistance(WHEEL_RAD, 1.0), true);
					BigArmHook.rotateTo(75);
					
					step = 5; 
					elev = 0;
				}
				else {
					detect = 0;
					elevation = 0;
				}
			}
			
			step--;
		}while(step >= 0);
		
		if(detect != 0) 
			for(int i = 0; i < (5 - detect); i++)
				Sound.beep();
		detected = true;
		
		//Realign the robot back to initial position
		leftMotor.rotate (-Navigator.convertAngle(WHEEL_RAD, TRACK, 35.0), true);
		rightMotor.rotate (Navigator.convertAngle(WHEEL_RAD, TRACK, 35.0), false);
	}
	
	
	/**
	 * This method records multiple samples, and implements a filter 
	 * to ignore values too low caused by ambient lighting. 
	 */
	
	private static void fetchColor(int elevation) {
		//Number of samples per fetch
		int count = 3;
		int colorFound = 0;
		long tStart, tEnd;
		do {
			tStart = System.currentTimeMillis();

			colorSample.fetchSample(colorBuffer, 0);
			
			double distance = Math.sqrt(colorBuffer[0]*colorBuffer[0] + colorBuffer[1]*colorBuffer[1] + colorBuffer[2]*colorBuffer[2]);
			// The color data must be above the minimum value to be qualified for checking.
			colorFound = 0;
			if(distance > minimum_value) {
				
				double previousError = 0;
				
				double[] data = new double[3];
				for(int i = 0; i < 3; i++)
					data[i] = colorBuffer[i]/distance;
				
				for(int i = 0; i < 4; i++) {

					double error = Math.pow(data[0] - COLOR[i][0][0],2) + 
							Math.pow(data[1] - COLOR[i][0][1],2) + 
							Math.pow(data[1] - COLOR[i][0][1],2);
					
					//If overlaps happens, we check which color is the closest
					if(colorFound == 0 || error < previousError) {
						colorFound = i + 1;
						previousError = error;
					}
				}
				
				colorCount[elevation][colorFound-1]++;
			}
			
			count--;
			tEnd   = System.currentTimeMillis();
			if (tEnd - tStart < 20) {
		        try {
		          Thread.sleep(20 - (tEnd - tStart));
		        } catch (InterruptedException e) {
		          e.printStackTrace();
		        }
		      }
		}while(count > 0);
		
	}

}
