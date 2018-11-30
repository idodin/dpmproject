package ca.mcgill.ecse211.Ev3Boot;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.GameLogic.GameLogic;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.RingRetrieval.CheckColor;
import ca.mcgill.ecse211.RingRetrieval.RingGrasp;
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
 * This is the constructor of the boot class,it initializes the needed fields
 * for the used sensors. This class's main method contains the method calls to
 * receive the game parameters, localize, travel to specified point, detect
 * rings, grab ring and drop rings
 * 
 * This class extends "MotorController" to facilitate all the motor logic.
 * 
 * This class also creates all the 4 threads, one for the display on
 * information, one for the odometry, one that calls sequentially all the method
 * used to accomplish the game logic.
 * 
 * Furthermore, sensor ports initializations are located in this class along
 * with their getters.
 * 
 * @author Imad Dodin
 * @author An Khang Chau
 * @author Chaimae Fahmi
 * @author Ai Di Wang
 * @author Umar Tahir
 * @author Hieu Chau Nguyen
 */

public class Ev3Boot extends MotorController {

	// ** Set these as appropriate for your team and current situation **
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port colorPortLeft = LocalEV3.get().getPort("S1");
	private static final Port colorPortFront = LocalEV3.get().getPort("S3");
	private static final Port colorPortRight = LocalEV3.get().getPort("S2");

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
	public static Localizer localizer;
	public static Display display;

	public static long demoStart;

	/**
	 * The main method initializes and runs threads for odometry and LCD Display.
	 * This method also create a thread that calls all the method needed to
	 * accomplish the tasks. First it calls "getInfo()" to receive the game
	 * parameters, then calls "localizeFE()" and "localizeColor()" to make the robot
	 * localize, it also calls "travelToTunnel()" to that makes the robot cross the
	 * tunnel from starting zone to land or the other way around. This tread also
	 * calls "turnAroundTree()" which navigate to and around the tree, detects and
	 * grab rings. Finally it calls "removeRing()" to drop the ring at the end.
	 */
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

		(new Thread() {
			public void run() {
				Wifi.getInfo();

				try {
					demoStart = System.currentTimeMillis();

					Localizer.localizeFE();
					Localizer.localizeColor();
					GameLogic.travelToTunnel(true);
					int position = GameLogic.closestSideOfTree(Wifi.getRingSet_x(), Wifi.getRingSet_y(),
							odo.getXYT()[0], odo.getXYT()[1]);
					RingSearch.turnAroundTree(position, Wifi.getRingSet_x(), Wifi.getRingSet_y());

					GameLogic.travelToTunnel(false);

					int map_x = 14;
					int map_y = 8;

					switch (Wifi.getCorner()) {
					case 0:
						Navigator.toStraightNavigator(1, 1, 8);
						for (int i = 0; i < 5; i++) {
							Sound.beep();
						}
						break;
					case 1:
						Navigator.toStraightNavigator(map_x, 1, 8);
						for (int i = 0; i < 5; i++) {
							Sound.beep();
						}
						break;
					case 2:
						Navigator.toStraightNavigator(map_x, map_y, 8);
						for (int i = 0; i < 5; i++) {
							Sound.beep();
						}
						break;
					case 3:
						Navigator.toStraightNavigator(1, map_y, 8);
						for (int i = 0; i < 5; i++) {
							Sound.beep();
						}
						break;

					}

					RingGrasp.removeRing();

				} catch (Exception e) {
					System.out.println("hello");
					// donothing;
				}
			}
		}).start();

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
	 * @return Color Sensor's Sample Provider
	 */
	public static SampleProvider getColorLeft() {
		return colorLeft;
	}

	/**
	 * Returns the Color Sensor's Data Buffer
	 * 
	 * @return Color Sensor's Data Buffer
	 */
	public static float[] getColorLeftBuffer() {
		return colorLeftBuffer;
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
	 * Returns the color buffer of the light sensor located on the right
	 * 
	 * @return  color buffer of the light sensor located on the right
	 */
	public static float[] getColorRightBuffer() {
		return colorRightBuffer;
	}

	/**
	 * return the sample provider of the right light sensor
	 * 
	 * @return sample provider of the right light sensor
	 */
	public static SampleProvider getColorRight() {
		return colorRight;
	}
}
