package ca.mcgill.ecse211.Ev3Boot;

import ca.mcgill.ecse211.Localization.LightLocalization;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.GameLogic.GameLogic;
import ca.mcgill.ecse211.Localization.LightLocalization;
import ca.mcgill.ecse211.Localization.Localizer;
import ca.mcgill.ecse211.RingRetrieval.CheckColor;
import ca.mcgill.ecse211.RingRetrieval.RingGrasp;
import ca.mcgill.ecse211.RingRetrieval.RingSearch;
import ca.mcgill.ecse211.Navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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

public class Ev3Boot {

	// ** Set these as appropriate for your team and current situation **

	private static final String SERVER_IP = "192.168.2.2";
	private static final int TEAM_NUMBER = 21;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor BigArmHook = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor armHook = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// Configuration Objects
	private static final double WHEEL_RAD = 2.106;
	private static final double TRACK = 15.80;
	private static final double TILE_SIZE = 30.48;
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port colorPortBack = LocalEV3.get().getPort("S2");
	private static final Port colorPortFront = LocalEV3.get().getPort("S3");

	private static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	private static SampleProvider usAverage = new MeanFilter(usDistance, 5);
	private static float[] usData = new float[usAverage.sampleSize()];
	private static SampleProvider colorBack = new EV3ColorSensor(colorPortBack).getMode("ColorID");
	private static float[] colorBufferBack = new float[colorBack.sampleSize()];
	private static SampleProvider colorFront = new EV3ColorSensor(colorPortFront).getMode("RGB");
	private static float[] colorBufferFront = new float[colorFront.sampleSize()];

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

	private static final double COLOR_MIN = 0.0113; // minimum total colour for ring detection

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

	@SuppressWarnings("rawtypes")
	public static void main(String[] args) throws OdometerExceptions {

		(new Thread() {
			public void run() {
				System.out.println("Running..");

				// armHook.rotateTo(-205);

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

					red_Zone_LL_x = ((Long) data.get("Red_LL_x")).intValue();
					red_Zone_LL_y = ((Long) data.get("Red_LL_y")).intValue();

					red_Zone_UR_x = ((Long) data.get("Red_UR_x")).intValue();
					red_Zone_UR_y = ((Long) data.get("Red_UR_y")).intValue();

					green_Zone_LL_x = ((Long) data.get("Green_LL_x")).intValue();
					green_Zone_LL_y = ((Long) data.get("Green_LL_y")).intValue();

					green_Zone_UR_x = ((Long) data.get("Green_UR_x")).intValue();
					green_Zone_UR_y = ((Long) data.get("Green_UR_y")).intValue();

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
				// Button.waitForAnyPress();

				try {
					odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
				} catch (OdometerExceptions e) {
					e.printStackTrace();
					return;
				}
				//
//				Display display = new Display(lcd);

				Thread odoThread = new Thread(odo);
				odoThread.start();
//				Thread odoDisplayThread = new Thread(display);
//				odoDisplayThread.start();

				try{
					Localizer.localizeFE();
				} catch (OdometerExceptions e) {
					//do nothing;
				}

				if (redTeam == 21) {
					corner = red_Corner;
					tunnel_LL_x = red_Tunnel_LL_x;
					tunnel_LL_y = red_Tunnel_LL_y;
					tunnel_UR_x = red_Tunnel_UR_x;
					tunnel_UR_y = red_Tunnel_UR_y;
					ringSet_x = red_Ring_Set_x;
					ringSet_y = red_Ring_Set_y;
					tunnelEntryIsLL = GameLogic.isTunnelEntryLL(tunnel_LL_x, tunnel_LL_y, tunnel_UR_x, tunnel_UR_y,
							red_Zone_LL_x, red_Zone_LL_y, red_Zone_UR_x, red_Zone_UR_y);
				} else {
					corner = green_Corner;
					tunnel_LL_x = green_Tunnel_LL_x;
					tunnel_LL_y = green_Tunnel_LL_y;
					tunnel_UR_x = green_Tunnel_UR_x;
					tunnel_UR_y = green_Tunnel_UR_y;
					ringSet_x = green_Ring_Set_x;
					ringSet_y = green_Ring_Set_y;
					tunnelEntryIsLL = GameLogic.isTunnelEntryLL(tunnel_LL_x, tunnel_LL_y, tunnel_UR_x, tunnel_UR_y,
							green_Zone_LL_x, green_Zone_LL_y, green_Zone_UR_x, green_Zone_UR_y);
				}

				switch (corner) {

				case 0:
					LightLocalization.lightLocalize(1, 1, false, 100, 0);
					break;
				case 1:
					LightLocalization.lightLocalize(7, 1, false, 100, 1);
					break;
				case 2:
					LightLocalization.lightLocalize(7, 7, false, 100, 2);
					break;
				case 3:
					LightLocalization.lightLocalize(1, 7, false, 100, 3);
					break;
				}
				double[] odoPosition = odo.getXYT();
				int[] currentPosition = new int[3];
				currentPosition[0] = (int) Math.round(odoPosition[0] / TILE_SIZE);
				currentPosition[1] = (int) Math.round(odoPosition[1] / TILE_SIZE);

				if (tunnelEntryIsLL) {
					if (tunnel_LL_x - tunnel_UR_x > 1) {
						if (Math.sqrt(Math.pow(tunnel_LL_x - currentPosition[0], 2)
								+ Math.pow(tunnel_LL_y - currentPosition[1], 2)) > 4) {
							System.out.println(
									"Travelling to: " + currentPosition[0] + (tunnel_LL_x - currentPosition[0]) / 2
											+ "," + currentPosition[0] + (tunnel_LL_x - currentPosition[0]) / 2);
							Navigator.travelTo(currentPosition[0] + (tunnel_LL_x - currentPosition[0]) / 2,
									currentPosition[1] + (tunnel_LL_y - currentPosition[1]) / 2, 4, true);
						}
						Navigator.travelTo(tunnel_LL_x - 1, tunnel_LL_y, 5, true);
						Navigator.travelTo(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5, 2, false);
						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y - 0.5, 5, false);
						Navigator.travelTo(tunnel_UR_x, tunnel_UR_y - 0.5, 2, false);
						Navigator.travelTo(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5, 2, false);
						Navigator.travelTo(tunnel_UR_x + 1, tunnel_UR_y , 2, true);

					} else {
						if (Math.sqrt(Math.pow(tunnel_LL_x - currentPosition[0], 2)
								+ Math.pow(tunnel_LL_y - currentPosition[1], 2)) > 4) {
							Navigator.travelTo(currentPosition[0] + (tunnel_LL_x - currentPosition[0]) / 2,
									currentPosition[1] + (tunnel_LL_y - currentPosition[1]) / 2, 4, true);
						}
						Navigator.travelTo(tunnel_LL_x, tunnel_LL_y - 1, 5, true);
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 2, false);
						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y - 0.5, 5, false);
						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y, 2, false);
						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5, 2, false);
						Navigator.travelTo(tunnel_UR_x , tunnel_UR_y + 1, 2, true);
					}
				} else {
					if (tunnel_LL_x - tunnel_UR_x > 1) {
						if (Math.sqrt(Math.pow(tunnel_UR_x - currentPosition[0], 2)
								+ Math.pow(tunnel_UR_y - currentPosition[1], 2)) > 4) {
							Navigator.travelTo(currentPosition[0] + (tunnel_UR_x - currentPosition[0]) / 2,
									currentPosition[1] + (tunnel_UR_y - currentPosition[1]) / 2, 4, true);
						}
						Navigator.travelTo(tunnel_UR_x + 1, tunnel_UR_y, 5, true);
						Navigator.travelTo(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5, 2, false);
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y + 0.5, 5, false);
						Navigator.travelTo(tunnel_LL_x, tunnel_LL_y + 0.5, 2, false);
						Navigator.travelTo(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5, 2, false);
						Navigator.travelTo(tunnel_LL_x - 1, tunnel_LL_y , 2, true);

					} else {
						if (Math.sqrt(Math.pow(tunnel_UR_x - currentPosition[0], 2)
								+ Math.pow(tunnel_UR_y - currentPosition[1], 2)) > 4) {
							Navigator.travelTo(currentPosition[0] + (tunnel_UR_x - currentPosition[0]) / 2,
									currentPosition[1] + (tunnel_UR_y - currentPosition[1]) / 2, 4, true);
						}
						Navigator.travelTo(tunnel_UR_x, tunnel_UR_y + 1, 5, true);
						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5, 2, false);
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y + 0.5, 5, false);
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y, 2, false);
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 2, false);
						Navigator.travelTo(tunnel_LL_x , tunnel_LL_y - 1, 2, true);
					}
				}

				int position = GameLogic.closestSideOfTree(ringSet_x, ringSet_y, odo.getXYT()[0], odo.getXYT()[1]);
				System.out.println("Case " + position);
				System.out.println("ring x: " + ringSet_x);
				System.out.println("ring y: " + ringSet_y);

				RingSearch.turnAroundTree(position, ringSet_x, ringSet_y);

//				if(tunnelEntryIsLL) {
//					if(tunnel_LL_x - tunnel_UR_x > 1) {
//						Navigator.travelTo(tunnel_UR_x+1, tunnel_UR_y, 5, true);
//						Navigator.travelTo(tunnel_UR_x+1, tunnel_UR_y -0.5, 2, false);
//						Navigator.travelTo(tunnel_LL_x-0.5, tunnel_LL_y + 0.5, 5, false);
//						
//					}
//					else {
//						Navigator.travelTo(tunnel_UR_x, tunnel_UR_y + 1 , 5, true);
//						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y+0.5, 2, false);
//						Navigator.travelTo(tunnel_LL_x+0.5, tunnel_LL_y-0.5, 5, false);
//					}	
//				}
//				else {
//					if(tunnel_LL_x - tunnel_UR_x > 1) {
//						Navigator.travelTo(tunnel_LL_x-1, tunnel_LL_y, 5, true);
//						Navigator.travelTo(tunnel_LL_x-1, tunnel_LL_y + 0.5, 2, false);
//						Navigator.travelTo(tunnel_UR_x+0.5, tunnel_UR_y-0.5, 5, false);
//					}
//					else {
//						Navigator.travelTo(tunnel_LL_x, tunnel_LL_y - 1 , 5, true);
//						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 2, false);
//						Navigator.travelTo(tunnel_UR_x-0.5, tunnel_UR_y+0.5, 5, false);
//					}	
//				}
//				
//				System.out.println(CheckColor.getDetectedColor());
//				System.out.println(CheckColor.getElevation());
//				
//				switch (corner) {
				//
//				case 0:
//					if(Math.sqrt(Math.pow(1 - currentPosition[0], 2) + Math.pow(1 - currentPosition[1], 2)) > 4) {
//						Navigator.travelTo(currentPosition[0] + (1 - currentPosition[0])/2, currentPosition[1] + (1 - currentPosition[1])/2, 4, true);
//					}
//					Navigator.travelTo(1, 1, 5, false);
//					break;
//				case 1:
//					if(Math.sqrt(Math.pow(7 - currentPosition[0], 2) + Math.pow(1 - currentPosition[1], 2)) > 4) {
//						Navigator.travelTo(currentPosition[0] + (7 - currentPosition[0])/2, currentPosition[1] + (1 - currentPosition[1])/2, 4, true);
//					}
//					Navigator.travelTo(7, 1, 5, false);
//					break;
//				case 2:
//					if(Math.sqrt(Math.pow(7 - currentPosition[0], 2) + Math.pow(7 - currentPosition[1], 2)) > 4) {
//						Navigator.travelTo(currentPosition[0] + (7 - currentPosition[0])/2, currentPosition[1] + (7 - currentPosition[1])/2, 4, true);
//					}
//					Navigator.travelTo(7, 7, 5, false);
//					break;
//				case 3:
//					if(Math.sqrt(Math.pow(1 - currentPosition[0], 2) + Math.pow(7 - currentPosition[1], 2)) > 4) {
//						Navigator.travelTo(currentPosition[0] + (1 - currentPosition[0])/2, currentPosition[1] + (7 - currentPosition[1])/2, 4, true);
//					}
//					Navigator.travelTo(1, 7, 5, false);
//					break;
//				}

				while (Button.waitForAnyPress() != Button.ID_ESCAPE)
					System.exit(0);
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
	public static SampleProvider getColorBack() {
		return colorBack;
	}

	/**
	 * Returns the Color Sensor's Data Buffer
	 * 
	 * @return
	 */
	public static float[] getColorBufferBack() {
		return colorBufferBack;
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
		return colorBufferFront;
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
	public static EV3LargeRegulatedMotor getArmHook() {
		return armHook;
	}

}
