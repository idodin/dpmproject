package ca.mcgill.ecse211.Ev3Boot;

import java.util.Map;

import ca.mcgill.ecse211.GameLogic.GameLogic;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;

public class Wifi {

	private static final String SERVER_IP = "192.168.2.50";
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

	public static int getTeam_zone_LL_x() {
		return team_zone_LL_x;
	}

	public static int getTeam_zone_LL_y() {
		return team_zone_LL_y;
	}

	public static int getTeam_zone_UR_x() {
		return team_zone_UR_x;
	}

	public static int getTeam_zone_UR_y() {
		return team_zone_UR_y;
	}

	public static int getCorner() {
		return corner;
	}

	public static void setCorner(int corner) {
		Wifi.corner = corner;
	}

	public static int getRedTeam() {
		return redTeam;
	}

	public static int getGreenTeam() {
		return greenTeam;
	}

	public static int getRed_Corner() {
		return red_Corner;
	}

	public static int getGreen_Corner() {
		return green_Corner;
	}

	public static int getRed_Zone_LL_x() {
		return red_Zone_LL_x;
	}

	public static int getRed_Zone_LL_y() {
		return red_Zone_LL_y;
	}

	public static int getRed_Zone_UR_x() {
		return red_Zone_UR_x;
	}

	public static int getRed_Zone_UR_y() {
		return red_Zone_UR_y;
	}

	public static int getGreen_Zone_LL_x() {
		return green_Zone_LL_x;
	}

	public static int getGreen_Zone_LL_y() {
		return green_Zone_LL_y;
	}

	public static int getGreen_Zone_UR_x() {
		return green_Zone_UR_x;
	}

	public static int getGreen_Zone_UR_y() {
		return green_Zone_UR_y;
	}

	public static int getIsland_LL_x() {
		return island_LL_x;
	}

	public static int getIsland_LL_y() {
		return island_LL_y;
	}

	public static int getIsland_UR_x() {
		return island_UR_x;
	}

	public static int getIsland_UR_y() {
		return island_UR_y;
	}

	public static int getRed_Tunnel_LL_x() {
		return red_Tunnel_LL_x;
	}

	public static int getRed_Tunnel_LL_y() {
		return red_Tunnel_LL_y;
	}

	public static int getRed_Tunnel_UR_x() {
		return red_Tunnel_UR_x;
	}

	public static int getRed_Tunnel_UR_y() {
		return red_Tunnel_UR_y;
	}

	public static int getGreen_Tunnel_LL_x() {
		return green_Tunnel_LL_x;
	}

	public static int getGreen_Tunnel_LL_y() {
		return green_Tunnel_LL_y;
	}

	public static int getGreen_Tunnel_UR_x() {
		return green_Tunnel_UR_x;
	}

	public static int getGreen_Tunnel_UR_y() {
		return green_Tunnel_UR_y;
	}

	public static int getRed_Ring_Set_x() {
		return red_Ring_Set_x;
	}

	public static int getRed_Ring_Set_y() {
		return red_Ring_Set_y;
	}

	public static int getGreen_Ring_Set_x() {
		return green_Ring_Set_x;
	}

	public static int getGreen_Ring_Set_y() {
		return green_Ring_Set_y;
	}

	public static int getTunnel_LL_x() {
		return tunnel_LL_x;
	}

	public static int getTunnel_LL_y() {
		return tunnel_LL_y;
	}

	public static int getTunnel_UR_x() {
		return tunnel_UR_x;
	}

	public static int getTunnel_UR_y() {
		return tunnel_UR_y;
	}

	public static int getRingSet_x() {
		return ringSet_x;
	}

	public static int getRingSet_y() {
		return ringSet_y;
	}

	public static boolean isTunnelEntryIsLL() {
		return tunnelEntryIsLL;
	}

}
