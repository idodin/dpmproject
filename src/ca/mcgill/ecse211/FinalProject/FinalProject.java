package ca.mcgill.ecse211.FinalProject;

import ca.mcgill.ecse211.Localization.LightLocalization;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.RingRetrieval.RingGrasp;
import ca.mcgill.ecse211.RingRetrieval.RingSearch;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;


/**
 * This class is the boot class for the Search and Localize Demo It initializes
 * and runs threads for odometry, localization and LCD Display.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 * @author Chaimae Fahmi
 * @author Ai Di Wang
 * @author Umar Tahir
 * @author Hieu Chau Nguyen 
 *
 */

public class FinalProject {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	

	// Configuration Objects
	private static final double WHEEL_RAD = 2.09;
	private static final double TRACK = 15.80;
	private static final double TILE_SIZE = 30.48;
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port colorPortBack = LocalEV3.get().getPort("S2");
	private static final Port colorPortFront = LocalEV3.get().getPort("S3");
	private static final Port gyroPort = LocalEV3.get().getPort("S1");

	// Sensor Objects
	public static EV3GyroSensor gyro = new EV3GyroSensor(gyroPort);
	public static SampleProvider gyroAngle = gyro.getAngleMode();
	private static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	private static SampleProvider usAverage = new MeanFilter(usDistance, 5);
	private static float[] usData = new float[usAverage.sampleSize()];
	private static SampleProvider colorBack = new EV3ColorSensor(colorPortBack).getMode("ColorID");
	private static float[] colorBufferBack = new float[colorBack.sampleSize()];
	private static float[] gyroBuffer = new float[gyroAngle.sampleSize()];
	private static SampleProvider colorFront = new EV3ColorSensor(colorPortFront).getMode("ColorID");
	private static float[] colorBufferFront = new float[colorBack.sampleSize()];
	

	public static Odometer odo;
	public static Navigator navigator;
	public static RingGrasp grasping;
	public static RingSearch searching;
	public static LightLocalization lightLocalization;
	public static Localizer localizer;
	public static Display display;
	// public static double angle;

	public static void main(String[] args) throws OdometerExceptions {
		
		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}
		
		Display display= new Display(lcd);

		Thread odoThread = new Thread(odo);
		odoThread.start();
		Thread odoDisplayThread = new Thread(display);
		odoDisplayThread.start();

		
		Navigator.turnTo(360);
		
		
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);

	}

	/**
	 * Return the Wheel Radius of the Robot
	 * 
	 * @return: Wheel radius
	 */
	public static double getWheelRad() {
		return WHEEL_RAD;
	}

	/**
	 * Return the Ultrasonic Sample Provider
	 * 
	 * @return: US sample provider
	 */
	public static SampleProvider getUSDistance() {
		return usDistance;
	}

	/**
	 * Return the Ultrasonic Distance Buffer
	 * 
	 * @return: US distance buffer 
	 */
	public static float[] getUSData() {
		return usData;
	}

	/**
	 * Return the Track (Wheelbase) of the Robot
	 * 
	 * @return: wheel base
	 */
	public static double getTrack() {
		return TRACK;
	}

	/**
	 * Return the Ultrasonic Average Sample Provider
	 * 
	 * @return: US Sample Provider 
	 */
	public static SampleProvider getUSAverage() {
		return usAverage;
	}

	/**
	 * Return the Color Sensor Sample Provider
	 * 
	 * @return
	 */
	public static SampleProvider getColorBack() {
		return colorBack;
	}

	/**
	 * Return the Color Sensor Data Buffer
	 * 
	 * @return
	 */
	public static float[] getColorBufferBack() {
		return colorBufferBack;
	}

	/**
	 * Return the left motor
	 * 
	 * @return: left motor
	 */
	public static EV3LargeRegulatedMotor getLeftmotor() {
		return leftMotor;
	}

	/**
	 * Return right motor
	 * 
	 * @return: right motor
	 */
	public static EV3LargeRegulatedMotor getRightmotor() {
		return rightMotor;
	}
	
	/**
	 * Return gyro buffer
	 * 
	 * @return: Gyro buffer 
	 */
	public static float[] getGyroBuffer() {
		return gyroBuffer;
	}

	/**
	 * Return tile size
	 * 
	 * @return: Tile size
	 */
	public static double getTileSize() {
		return TILE_SIZE;
	}

	/**
	 * Returns the sample provider of the front light sensor
	 * 
	 * @return: Front light sensor sample provider
	 */
	public static SampleProvider getColorFront() {
		return colorFront;
	}

	/**
	 * Return the color buffer of the front light sensor
	 * 
	 * @return: Front light sensor's color buffer
	 */
	public static float[] getColorBufferFront() {
		return colorBufferFront;
	}

}
