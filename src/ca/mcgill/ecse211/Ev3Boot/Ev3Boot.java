package ca.mcgill.ecse211.Ev3Boot;

import ca.mcgill.ecse211.Localization.LightLocalization;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

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

public class Ev3Boot {

	// ** Set these as appropriate for your team and current situation **

	private static final String SERVER_IP = "192.168.2.13";
	private static final int TEAM_NUMBER = 21;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

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

	// game values
	private static int redTeam;
	private static int greenTeam;
	
	private static int red_Corner;
	private static int green_Corner;
	
	private static int red_Corner_LL_x;
	private static int red_Corner_LL_y;
	
	private static int red_Corner_UR_x;
	private static int red_Corner_UR_y;
	
	private static int green_Corner_LL_x;
	private static int green_Corner_LL_y;
	
	private static int green_Corner_UR_x;
	private static int green_Corner_UR_y;
	
	private static int island_LL_x;
	private static int island_LL_y;
	
	private static int island_UR_x;
	private static int island_UR_y;
	
	private static int red_Tunnel_LL_x;
	private static int red_Tunnel_LL_y;
	
	private static int red_Tunnel_UR_x;
	private static int red_Tunnel_UR_y;
	
	private static int green_Tunnel_LL_x;
	private static int green_Tunnel_LL_y;
	
	private static int green_Tunnel_UR_x;
	private static int green_Tunnel_UR_y;
	
	private static int red_Ring_Set_x;
	private static int red_Ring_Set_y;

	private static int green_Ring_Set_x;
	private static int green_Ring_Set_y;

	@SuppressWarnings("rawtypes")
	public static void main(String[] args) throws OdometerExceptions {

		System.out.println("Running..");

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			
			/*
			 * getData() will connect to the server and wait until the user/TA presses the
			 * "Start" button in the GUI on their laptop with the data filled in. Once it's
			 * waiting, you can kill it by pressing the upper left hand corner button
			 * (back/escape) on the EV3. getData() will throw exceptions if it can't connect
			 * to the server (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception if it
			 * connects but receives corrupted data or a message from the server saying
			 * something went wrong. For example, if TEAM_NUMBER is set to 1 above but the
			 * server expects teams 17 and 5, this robot will receive a message saying an
			 * invalid team number was specified and getData() will throw an exception
			 * letting you know.
			 */
			
			Map data = conn.getData();
			
			redTeam = ((Long) data.get("RedTeam")).intValue();
			greenTeam = ((Long) data.get("GreenTeam")).intValue();
			
			red_Corner = ((Long) data.get("RedCorner")).intValue();
			green_Corner = ((Long) data.get("GreenCorner")).intValue();
			
			red_Corner_LL_x = ((Long) data.get("Red_LL_x")).intValue();
			red_Corner_LL_y = ((Long) data.get("Red_LL_y")).intValue();
			
			red_Corner_UR_x = ((Long) data.get("Red_UR_x")).intValue();
			red_Corner_UR_y = ((Long) data.get("Red_UR_y")).intValue();
			
			green_Corner_LL_x = ((Long) data.get("Green_LL_x")).intValue();
			green_Corner_LL_y = ((Long) data.get("Green_LL_y")).intValue();
			
			green_Corner_UR_x = ((Long) data.get("Green_UR_x")).intValue();
			green_Corner_UR_y = ((Long) data.get("Green_UR_y")).intValue();
			
			island_LL_x = ((Long) data.get("Island_LL_x")).intValue();
			island_LL_y = ((Long) data.get("Island_LL_y")).intValue();
			
			island_UR_x = ((Long) data.get("Island_UR_x")).intValue();
			island_UR_y = ((Long) data.get("Island_UR_y")).intValue();
			
			red_Tunnel_LL_x = ((Long) data.get("TNR_LL_x")).intValue();
			red_Tunnel_LL_y = ((Long) data.get("TNR_LL_y")).intValue();
			
			red_Tunnel_UR_x = ((Long) data.get("TNR_UR_x")).intValue();
			red_Tunnel_UR_y = ((Long) data.get("TNR_UR_y")).intValue();
			
			green_Tunnel_LL_x = ((Long) data.get("TNG_LL_x")).intValue();
			green_Tunnel_LL_y = ((Long) data.get("TNG_LL_y")).intValue();
			
			green_Tunnel_UR_x = ((Long) data.get("TNG_UR_x")).intValue();
			green_Tunnel_UR_y = ((Long) data.get("TNG_UR_y")).intValue();
			
			red_Ring_Set_x = ((Long) data.get("TR_x")).intValue();
			red_Ring_Set_y = ((Long) data.get("TR_y")).intValue();

			green_Ring_Set_x = ((Long) data.get("TG_x")).intValue();
			green_Ring_Set_y = ((Long) data.get("TG_y")).intValue();
			

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// Wait until user decides to end program
		Button.waitForAnyPress();

		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		Display display = new Display(lcd);

		Thread odoThread = new Thread(odo);
		odoThread.start();
		Thread odoDisplayThread = new Thread(display);
		odoDisplayThread.start();

		Navigator.travelTo(red_Tunnel_LL_x, red_Tunnel_LL_y, true);
		Navigator.travelTo(1, 3, true);
		Navigator.travelTo(3, 1, true);
		Navigator.travelTo(0, 0, true);

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
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
