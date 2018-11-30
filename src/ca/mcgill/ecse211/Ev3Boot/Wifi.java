package ca.mcgill.ecse211.Ev3Boot;

import java.util.Map;

import ca.mcgill.ecse211.GameLogic.GameLogic;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * Class which takes care of receiving game parameters and interpret them.
 * 
 * Initialize every game parameters and set them to the value needed according
 * to the data received Contains getter for all game parameters.
 * 
 * Contains variable which are dependent on the team you are.
 *
 * Class called by Ev3Boot class to receive useful parameters.
 */
public class Wifi {

	private static final String SERVER_IP = "192.168.2.2";
	private static final int TEAM_NUMBER = 21;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

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

	private static int team_zone_LL_x;
	private static int team_zone_LL_y;
	private static int team_zone_UR_x;
	private static int team_zone_UR_y;

	private static int ringSet_x;
	private static int ringSet_y;

	private static boolean tunnelEntryIsLL;

	/**
	 * Class that establish connection with the wifi with corresponding "SERVER_IP".
	 * Then get the data from the connection. Then it assigns data to game
	 * parameters used. Finally, depending on the color the robot's team is, assign
	 * color dependent values to corresponding data.
	 * 
	 * This method is the first method called by Ev3Boot class.
	 */
	public static void getInfo() {

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

			if (redTeam == 21) {
				corner = red_Corner;
				tunnel_LL_x = red_Tunnel_LL_x;
				tunnel_LL_y = red_Tunnel_LL_y;
				tunnel_UR_x = red_Tunnel_UR_x;
				tunnel_UR_y = red_Tunnel_UR_y;
				ringSet_x = red_Ring_Set_x;
				ringSet_y = red_Ring_Set_y;
				team_zone_LL_x = red_Zone_LL_x;
				team_zone_LL_y = red_Zone_LL_y;
				team_zone_UR_x = red_Zone_UR_x;
				team_zone_UR_y = red_Zone_UR_y;
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
				team_zone_LL_x = green_Zone_LL_x;
				team_zone_LL_y = green_Zone_LL_y;
				team_zone_UR_x = green_Zone_UR_x;
				team_zone_UR_y = green_Zone_UR_y;
				tunnelEntryIsLL = GameLogic.isTunnelEntryLL(tunnel_LL_x, tunnel_LL_y, tunnel_UR_x, tunnel_UR_y,
						green_Zone_LL_x, green_Zone_LL_y, green_Zone_UR_x, green_Zone_UR_y);
			}

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
	}

	/**
	 * returns robot's team zone lower left x-coordinate
	 * 
	 * @return robot's team zone lower left x-coordinate
	 */
	public static int getTeam_zone_LL_x() {
		return team_zone_LL_x;
	}

	/**
	 * returns robot's team zone lower left y-coordinate
	 * 
	 * @return robot's team zone lower left y-coordinate
	 */
	public static int getTeam_zone_LL_y() {
		return team_zone_LL_y;
	}

	/**
	 * returns robot's team zone upper right x-coordinate
	 * 
	 * @return robot's team zone upper right x-coordinate
	 */
	public static int getTeam_zone_UR_x() {
		return team_zone_UR_x;
	}

	/**
	 * returns robot's team zone upper right y-coordinate
	 * 
	 * @return robot's team zone upper right y-coordinate
	 */
	public static int getTeam_zone_UR_y() {
		return team_zone_UR_y;
	}

	/**
	 * returns robot's starting corner
	 * 
	 * @return robot's starting corner
	 */
	public static int getCorner() {
		return corner;
	}

	/**
	 * return island lower left x-coordinate
	 * 
	 * @return island lower left x-coordinate
	 */
	public static int getIsland_LL_x() {
		return island_LL_x;
	}

	/**
	 * return island lower left y-coordinate
	 * 
	 * @return island lower left y-coordinate
	 */
	public static int getIsland_LL_y() {
		return island_LL_y;
	}

	/**
	 * return island upper right x-coordinate
	 * 
	 * @return island upper right x-coordinate
	 */
	public static int getIsland_UR_x() {
		return island_UR_x;
	}

	/**
	 * return island upper right y-coordinate
	 * 
	 * @return island upper right y-coordinate
	 */
	public static int getIsland_UR_y() {
		return island_UR_y;
	}

	/**
	 * return robot's team lower left x-coordinate of the tunnel
	 * 
	 * @return
	 */
	public static int getTunnel_LL_x() {
		return tunnel_LL_x;
	}

	/**
	 * return robot's team lower left y-coordinate of the tunnel
	 * 
	 * @return
	 */
	public static int getTunnel_LL_y() {
		return tunnel_LL_y;
	}

	/**
	 * return robot's team upper right x-coordinate of the tunnel
	 * 
	 * @return robot's team upper right x-coordinate of the tunnel
	 */
	public static int getTunnel_UR_x() {
		return tunnel_UR_x;
	}

	/**
	 * return robot's team upper right y-coordinate of the tunnel
	 * 
	 * @return robot's team upper right y-coordinate of the tunnel
	 */
	public static int getTunnel_UR_y() {
		return tunnel_UR_y;
	}

	/**
	 * return robot's team ring set x-coordinate
	 * 
	 * @returnrobot's team ring set x-coordinate
	 */
	public static int getRingSet_x() {
		return ringSet_x;
	}

	/**
	 * return robot's team ring set y-coordinate
	 * 
	 * @return robot's team ring set y-coordinate
	 */
	public static int getRingSet_y() {
		return ringSet_y;
	}

	/**
	 * return true if lower left is part of the tunnel entry return false if lower
	 * left is not part of the tunnel entry
	 * 
	 * @return true if lower left is part of the tunnel entry return false if lower
	 *         left is not part of the tunnel entry
	 */
	public static boolean isTunnelEntryIsLL() {
		return tunnelEntryIsLL;
	}

}
