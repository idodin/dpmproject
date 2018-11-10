package ca.mcgill.ecse211.RingRetrieval;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.navigation.Navigator;

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
public class CheckColor{

	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();
	private static double wheelrad= Ev3Boot.getWheelRad();
	private static SampleProvider colorSample = Ev3Boot.getColorFront();
	private static float[] colorBuffer = Ev3Boot.getColorBufferFront();
	
	//How often a colour is detected? 
	private static int[] colorCount;

	// Normalized mean, std*modifier_for_detection
	static double[][][] COLOR = {
			//ORANGE
			{{0.940487,0.270557, 0.198284},{0.018521652*2,0.045192704*2,0.030158541*2}},
			//YELLOW
			{{0.893274575,0.377327128,0.244015957},{0.009333934*4,0.031746029*2,0.046546512*2}},
			//GREEN
			{{0.517744752,0.773791369,0.35647404},{0.057517323*4,0.048271431*4,0.033977313*4}},
			//BLUE
			{{0.138686903,0.44275977,0.8794325},{0.024216451*4,0.094028999*4,0.056205047*4}}
	};
	
	
	// Which color is detected.
	// 0: ORANGE (1)
	// 1: YELLOW (2)
	// 2: GREEN (3)
	// 3: BLUE (4)
	private static int detect = 0;
	private static boolean detected = false;
	private static double minimum_value;

	
	/**
	 * Check if the robot finished detecting the color
	 * @return Boolean value true if the color was detected false otherwise.
	 */
	public static boolean finishDetect() {
		return detected;
	}
	
	/**
	 * Restart the checker for stability reason
	 */
	public void restartChecker() {
		detected = false; 
		colorCount = new int[] {0, 0, 0, 0 };
	}
	
	/**
	 * Get the value of the detected Color
	 * @return Integer value of the colour
	 */
	public static int getDetectedColor() {
		return detect;
	}
	
	
	/**
	 * This method assumes the robot is close enough to the ring,
	 * It makes the robot slightly move around to record multiple samples
	 * using the fetchColor method.
	 * It sets the variable detect to the value of the detected ring
	 */
	public static void colorDetection() {
		
		colorCount = new int[] {0, 0, 0, 0};
		
		// Collect environment's light data 
		//to set the minimum color required
		colorSample.fetchSample(colorBuffer, 0); 
		try {
			Thread.sleep(100);
		} catch (Exception e) {}
		colorSample.fetchSample(colorBuffer, 0);
		minimum_value = Math.sqrt(colorBuffer[0]*colorBuffer[0] + colorBuffer[1]*colorBuffer[1] + colorBuffer[2]*colorBuffer[2]);
		//Add 10% to the minimum value;
		minimum_value = minimum_value * 11/10;
		
		int step = 4;
		do {

			fetchColor();
			
			// Budging the robot
			leftMotor.setSpeed(30);
			rightMotor.setSpeed(30);
			
			if(step >2) {
				leftMotor.rotate (Navigator.convertDistance(wheelrad, 1.0), true);
				rightMotor.rotate(Navigator.convertDistance(wheelrad, 1.0), false);
			}
			else {
				leftMotor.rotate (-Navigator.convertDistance(wheelrad, 1.0), true);
				rightMotor.rotate(-Navigator.convertDistance(wheelrad, 1.0), false);
			}
			
			step--;
		}while(step >= 0);
		
		int max = 0;
		int maxHolder = 0;
		for(int i = 0; i < 4; i++) {
			if(colorCount[i] > max) {
				maxHolder = i+1;
				max = colorCount[i];
			}
		}
		
		detect   = maxHolder;
		detected = true;
	}
	
	
	/**
	 * This method records multiple samples, and implements a filter 
	 * to ignore values too low caused by ambient lighting. 
	 */
	
	private static void fetchColor() {
		//Number of samples per fetch
		int count = 5;
		int colorFound = 0;
		long tStart, tEnd;
		do {
			tStart = System.currentTimeMillis();

			colorSample.fetchSample(colorBuffer, 0);
			
			double distance = Math.sqrt(colorBuffer[0]*colorBuffer[0] + colorBuffer[1]*colorBuffer[1] + colorBuffer[2]*colorBuffer[2]);
			
			// The color data must be above the minimum value to be qualified for checking.
			if(distance > minimum_value) {
				
				double previousError = 0;
				
				double[] data = new double[3];
				for(int i = 0; i < 3; i++)
					data[i] = colorBuffer[i]/distance;
				
				boolean foundColor = false;
				for(int i = 0; i < 4; i++) {
					boolean fail = false;
					for(int j = 0; j < 3; j++)
						if(data[j] <= (COLOR[i][0][j] - COLOR[i][1][j]) || data[j] >= (COLOR[i][0][j] + COLOR[i][1][j])) {
							fail = true;
							break;
						}
					
					if(!fail) {
						foundColor = true;

						double error = Math.pow(data[0] - COLOR[i][0][0],2) + 
								Math.pow(data[1] - COLOR[i][0][1],2) + 
								Math.pow(data[1] - COLOR[i][0][1],2);
						
						//If overlaps happens, we check which color is the closest
						if(colorFound == 0 || error < previousError) {
							colorFound = i + 1;
							previousError = error;
						}
						
					}
				}
			
				if(foundColor) 
					colorCount[colorFound-1]++;
				
			}
			
			count--;
			tEnd   = System.currentTimeMillis();
			if (tEnd - tStart < 50) {
		        try {
		          Thread.sleep(50 - (tEnd - tStart));
		        } catch (InterruptedException e) {
		          e.printStackTrace();
		        }
		      }
		}while(count > 0);
		
	}

}
