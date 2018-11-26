package ca.mcgill.ecse211.Ev3Boot;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.GameLogic.GameLogic;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.RingRetrieval.CheckColor;
import ca.mcgill.ecse211.RingRetrieval.RingSearch;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This is the constructor of the boot class,it intializes the needed fields for
 * the used motors and sensors. This class's main method contains the method
 * calls which define the game logic, including lightLocalize, localizeFE and
 * travelTo.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 * @author Chaimae Fahmi
 * @author Ai Di Wang
 * @author Umar Tahir
 * @author Hieu Chau Nguyen
 *
 */

public class Ev3Boot extends MotorController {

	// ** Set these as appropriate for your team and current situation **

	

	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port colorPortLeft = LocalEV3.get().getPort("S1");
	private static final Port colorPortFront = LocalEV3.get().getPort("S3");

	private static final Port colorPortRight = LocalEV3.get().getPort("S2");

	// Sensor Objects
	// public static EV3GyroSensor colorRightSensor = new
	// EV3GyroSensor(colorPortRight);
	// public static EV3GyroSensor colorLeftSensor = new
	// EV3GyroSensor(colorPortLeft);
	// public static EV3GyroSensor colorfrontSensor = new
	// EV3GyroSensor(colorPortFront);

	private static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	private static SampleProvider colorLeft = new EV3ColorSensor(colorPortLeft).getMode("Red");
	private static SampleProvider colorFront = new EV3ColorSensor(colorPortFront).getMode("RGB");
	private static SampleProvider colorRight = new EV3ColorSensor(colorPortRight).getMode("Red");
	private static SampleProvider usAverage = new MeanFilter(usDistance, 5);

	private static float[] usData = new float[usAverage.sampleSize()];
	private static float[] colorLeftBuffer = new float[colorLeft.sampleSize()];
	private static float[] colorRightBuffer = new float[colorRight.sampleSize()];
	private static float[] colorFrontBuffer = new float[colorFront.sampleSize()];

	public static Odometer odo;
	public static Navigator navigator;
	// public static RingGrasp grasping;
	// public static RingSearch searching;
	public static Localizer localizer;
	public static Display display;

	// game values
	private static int redTeam;
	private static int greenTeam;

	private static int red_Corner;
	private static int green_Corner;

	private static int red_Zone_LL_x;
	private static int red_Zone_LL_y;

	private static int red_Zone_UR_x;
	private static int red_Zone_UR_y;

	private static int green_Zone_LL_x;
	private static int green_Zone_LL_y;

	private static int green_Zone_UR_x;
	private static int green_Zone_UR_y;

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

	// values color dependent
	private static int corner;

	private static int tunnel_LL_x;
	private static int tunnel_LL_y;
	private static int tunnel_UR_x;
	private static int tunnel_UR_y;

	private static int ringSet_x;
	private static int ringSet_y;
	private static boolean tunnelEntryIsLL;

	private static float color;
	private static float oldColorRight;

	private static final double COLOR_MIN = 0.0113; // minimum total colour for ring detection
	public static long demoStart;

	/**
	 * The main method establishes the connection with the Dpm server using the
	 * WifiConnection class, and gets the necessary information about the field
	 * (corner, red_Team...) It also initializes and runs threads for odometry and
	 * LCD Display, Inside this method, the localizeFE and lightLocalize methods are
	 * called which make the robot localize. It also calls the travelTo method from
	 * the Navigator class, to make the robot first travel to the tunnel, through it
	 * and to the ring set. Once the robot is at the ring set, a call is made to the
	 * turnAroundTree from the RingSearch class to detect and retreive the rings.
	 */

	// @SuppressWarnings("rawtypes")
	public static void main(String[] args) throws OdometerExceptions {
		try {
			odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		Display display = new Display(lcd);

		// Start odometry
		Thread odoThread = new Thread(Odometer.getOdometer());
		odoThread.start();

		Thread odoDisplayThread = new Thread(display);
		odoDisplayThread.start();

		// start of random shit
		red_Zone_LL_x = 2;
		red_Zone_LL_y = 0;

		red_Zone_UR_x = 8;
		red_Zone_UR_y = 3;

		tunnel_LL_x = 2;
		tunnel_LL_y = 3;

		tunnel_UR_x = 3;
		tunnel_UR_y = 5;
		ringSet_x = 5;
		ringSet_y = 6;
		// end of random shit
		
		(new Thread() {
			public void run() {
				Wifi.getInfo();
			}
		}).start();;

		(new Thread() {
			public void run() {

				int buttonChoice;
				do {
					// // clear the display
					// lcd.clear();
					//
					// // ask the user whether the motors should drive in a square or float
					// lcd.drawString("Awaiting Input ", 0, 0);

					buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
				} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

				try {
					demoStart = System.currentTimeMillis();
//					Navigator.toStraightNavigator(4.5, 4.5, 7);
//					Navigator.toStraightNavigator(0.5, 0.5, 7);
//					turnBy(90, true, false, TURN_SPEED);
//					try {
//						Thread.sleep(2000);
//					} catch (InterruptedException e) {
//					}
//					turnBy(90, true, false, TURN_SPEED);
//					try {
//						Thread.sleep(2000);
//					} catch (InterruptedException e) {
//					}
//					turnBy(90, true, false, TURN_SPEED);
//					try {
//						Thread.sleep(2000);
//					} catch (InterruptedException e) {
//					}
//					turnBy(90, true, false, TURN_SPEED);
//					try {
//						Thread.sleep(2000);
//					} catch (InterruptedException e) {
//					}
//					forwardBy(TILE_SIZE);
//					try {
//						Thread.sleep(2000);
//					} catch (InterruptedException e) {
//					}
					Localizer.localizeFE();
					Localizer.localizeColor();
					GameLogic.travelToTunnel(true);
					int position = GameLogic.closestSideOfTree(ringSet_x, ringSet_y, odo.getXYT()[0], odo.getXYT()[1]);
					RingSearch.turnAroundTree(position, ringSet_x, ringSet_y);
				} catch (OdometerExceptions e) {
					System.out.println("hello");
					// donothing;
				}
			}
		}).start();


	}

	/**
	 * Returns the Wheel Radius of the Robot
	 * 
	 * @return: Wheel radius
	 */
	public static double getWheelRad() {
		return WHEEL_RAD;
	}

	/**
	 * Returns the Ultrasonic Sample Provider
	 * 
	 * @return: US sample provider
	 */
	public static SampleProvider getUSDistance() {
		return usDistance;
	}

	/**
	 * Returns the Ultrasonic Distance Buffer
	 * 
	 * @return: US distance buffer
	 */
	public static float[] getUSData() {
		return usData;
	}

	/**
	 * Returns the Track (Wheelbase) of the Robot
	 * 
	 * @return: wheel base
	 */
	public static double getTrack() {
		return TRACK;
	}

	/**
	 * Returns the Ultrasonic Sample Provider
	 * 
	 * @return: US Sample Provider
	 */
	public static SampleProvider getUSAverage() {
		return usAverage;
	}

	/**
	 * Returns the Color Sensor's Sample Provider
	 * 
	 * @return
	 */
	public static SampleProvider getColorLeft() {
		return colorLeft;
	}

	/**
	 * Returns the Color Sensor's Data Buffer
	 * 
	 * @return
	 */
	public static float[] getColorLeftBuffer() {
		return colorLeftBuffer;
	}

	/**
	 * Returns the left motor
	 * 
	 * @return: left motor
	 */
	public static EV3LargeRegulatedMotor getLeftmotor() {
		return leftMotor;
	}

	/**
	 * Returns right motor
	 * 
	 * @return: right motor
	 */
	public static EV3LargeRegulatedMotor getRightmotor() {
		return rightMotor;
	}

	/**
	 * Returns the tile size
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
	 * Returns the color buffer of the front light sensor
	 * 
	 * @return: Front light sensor's color buffer
	 */
	public static float[] getColorBufferFront() {
		return colorFrontBuffer;
	}

	/**
	 * Get the minimum total color distance for the rind detection
	 * 
	 * @return minimum color distance
	 */
	public static double getColorMin() {
		return COLOR_MIN;
	}

	/**
	 * Return big arm motor
	 * 
	 * @return
	 */
	public static EV3LargeRegulatedMotor getBigArmHook() {
		return BigArmHook;
	}

	/**
	 * Return small arm motor
	 * 
	 * @return
	 */
	public static EV3MediumRegulatedMotor getArmHook() {
		return armHook;
	}

	public static float[] getColorRightBuffer() {
		return colorRightBuffer;
	}

	public static void setColorRightBuffer(float[] colorRightBuffer) {
		Ev3Boot.colorRightBuffer = colorRightBuffer;
	}

	public static SampleProvider getColorRight() {
		return colorRight;
	}

	public static void setColorRight(SampleProvider colorRight) {
		Ev3Boot.colorRight = colorRight;
	}

}
