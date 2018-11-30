package ca.mcgill.ecse211.GameLogic;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Ev3Boot.MotorController;
import ca.mcgill.ecse211.Ev3Boot.Wifi;
import ca.mcgill.ecse211.Navigation.Navigator;

/**
 * This class contains helper method to facilitate game logic. Contains methods
 * to determine which tunnel entry to choose and which side of the ring set to
 * navigate to first. Also contains methods that calls successive travel method
 * to go to the bridge and cross it.
 * 
 * Methods inside this class are called by Ev3Boot class to execute game tasks
 * 
 * This class extends "MotorController" to facilitate all the motor logic.
 */
public class GameLogic extends MotorController {

	/**
	 * Method used to travel from current position to the tunnel and cross it. It
	 * can be use to cross the bridge from starting zone or from island. It also
	 * make sure the arm is in a position where it is possible to cross the bridge
	 * 
	 * Methods get tunnel, starting zone and island coordinates to determine the
	 * dimension of the tunnel, its orientation and where is the entry.
	 * 
	 * @param going: true if the robot wants to cross the bridge from staring land
	 *        false if the robot wants to cross the bridge from island
	 */
	public static void travelToTunnel(boolean going) {

		int tunnel_LL_x = Wifi.getTunnel_LL_x();
		int tunnel_LL_y = Wifi.getTunnel_LL_y();
		int tunnel_UR_x = Wifi.getTunnel_UR_x();
		int tunnel_UR_y = Wifi.getTunnel_UR_y();
		int team_Zone_LL_x = Wifi.getTeam_zone_LL_x();
		int team_Zone_LL_y = Wifi.getTeam_zone_LL_y();
		int team_Zone_UR_x = Wifi.getTeam_zone_UR_x();
		int team_Zone_UR_y = Wifi.getTeam_zone_UR_y();

		boolean tunnelEntryIsLL = isTunnelEntryLL(tunnel_LL_x, tunnel_LL_y, tunnel_UR_x, tunnel_UR_y, team_Zone_LL_x,
				team_Zone_LL_y, team_Zone_UR_x, team_Zone_UR_y);

		if (going) {
			if (tunnelEntryIsLL) {
				// If traversing tunnel horizontally to the right
				if (tunnel_UR_x - tunnel_LL_x > 1) {
					System.out.println("Horizontal Right");
					System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
					// Go to entry
					Navigator.toStraightNavigator(tunnel_LL_x - 0.2, tunnel_LL_y + 0.5, 8);
					// Readjust and prepare arm
					turnTo(100);
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.travelUntil();
					// Go to exit
					Navigator.travelTo(tunnel_UR_x + 0.7, tunnel_UR_y - 0.5, 7, false);
					BigArmHook.rotate(-120);

				} else if (tunnel_UR_x - tunnel_LL_x == 1 && tunnel_UR_y - tunnel_LL_y == 1) {
					// entry is vertical
					if (isPointOnZone(tunnel_LL_x + 1, tunnel_LL_y, team_Zone_LL_x, team_Zone_LL_y, team_Zone_UR_x,
							team_Zone_UR_y)) {
						System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
						// Go to entry
						Navigator.toStraightNavigator(tunnel_LL_x + 0.5, tunnel_LL_y - 0.2, 8);
						turnTo(0);
						// Readjust and prepare arm
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.turnTo(10);
						Navigator.travelUntil();
						// Go to exit
						Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y + 0.7, 7, false);
						BigArmHook.rotate(-120);
						// Traverse tunnel horizontally to the right
					} else {
						System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
						// Go to Entry
						Navigator.toStraightNavigator(tunnel_LL_x - 0.2, tunnel_LL_y + 0.5, 8);
						turnTo(100);
						// Readjust and prepare arm
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						turnTo(100);
						Navigator.travelUntil();
						// Go to exit
						Navigator.travelTo(tunnel_UR_x + 0.7, tunnel_UR_y - 0.5, 7, false);
						BigArmHook.rotate(-120);
					}
					// traverse vertically up
				} else {
					System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
					System.out.println("Vertically Up");
					// Go to Entry
					Navigator.toStraightNavigator(tunnel_LL_x + 0.5, tunnel_LL_y - 0.2, 8);
					turnTo(0);
					// Readjust and prepare arm
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.turnTo(10);
					Navigator.travelUntil();
					// Go to exit
					Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y + 0.7, 7, false);
					System.out.println("Current:" + odo.getXYT()[0] + "," + odo.getXYT()[1] + "," + odo.getXYT()[2]);
					BigArmHook.rotate(-120);
				}
			} else {
				// Traverse horizontally to the left
				if (tunnel_UR_x - tunnel_LL_x > 1) {
					System.out.println("Horizontal Left");
					System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
					// Go to entry
					Navigator.toStraightNavigator(tunnel_UR_x + 0.2, tunnel_UR_y - 0.5, 8);
					turnTo(280);
					// Readjust and prepare arm
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.turnTo(10);
					Navigator.travelUntil();
					// Go to exit
					Navigator.travelTo(tunnel_LL_x - 0.7, tunnel_LL_y + 0.5, 7, false);
					BigArmHook.rotate(-120);
				} else if (tunnel_UR_x - tunnel_LL_x == 1 && tunnel_UR_y - tunnel_LL_y == 1) {
					// entry is vertical
					if (isPointOnZone(tunnel_UR_x - 1, tunnel_UR_y, team_Zone_LL_x, team_Zone_LL_y, team_Zone_UR_x,
							team_Zone_UR_y)) {
						System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
						// Go to entry
						Navigator.toStraightNavigator(tunnel_UR_x - 0.5, tunnel_UR_y + 0.2, 8);
						turnTo(180);
						// Readjust and prepare arm
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.turnTo(190);
						Navigator.travelUntil();
						// Go to exit
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.7, 7, false);
						BigArmHook.rotate(-120);
						// entry is horizontal
					} else {
						System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
						// Go to entry
						Navigator.toStraightNavigator(tunnel_UR_x + 0.2, tunnel_UR_y - 0.5, 8);
						turnTo(280);
						// Readjust and prepare arm
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.turnTo(280);
						Navigator.travelUntil();
						// Navigator.travelTo(tunnel_UR_x , tunnel_UR_y - 0.5, 7, true);
						// Go to exit
						Navigator.travelTo(tunnel_LL_x - 0.7, tunnel_LL_y + 0.5, 7, false);
						BigArmHook.rotate(-120);
					}
					// Traverse tunnel vertically downwards
				} else {
					System.out.println("Vertical Down");
					System.out.println("Going to:" + tunnel_LL_x + "," + tunnel_LL_y);
					// Go to entry
					Navigator.toStraightNavigator(tunnel_UR_x - 0.5, tunnel_UR_y + 0.2, 8);
					turnTo(190);
					// Readjust and prepare arm
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.turnTo(190);
					Navigator.travelUntil();
					// Go to exit
					Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.7, 7, false);
					BigArmHook.rotate(-120);
				}
			}
		}

		// coming back
		else {
			// go to ur then go to ll
			if (tunnelEntryIsLL) {
				if (tunnel_UR_x - tunnel_LL_x > 1) {
					Navigator.toStraightNavigator(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5, 8);
					turnTo(270);
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.travelUntil();
					Navigator.travelTo(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5, 5, false);

				} else if (tunnel_UR_x - tunnel_LL_x == 1 && tunnel_UR_y - tunnel_LL_y == 1) {
					// entry is vertical
					if (isPointOnZone(tunnel_LL_x + 1, tunnel_LL_y, team_Zone_LL_x, team_Zone_LL_y, team_Zone_UR_x,
							team_Zone_UR_y)) {
						Navigator.toStraightNavigator(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5, 8);
						turnTo(180);
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.travelUntil();
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 5, false);
					} else {
						Navigator.toStraightNavigator(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5, 8);
						turnTo(270);
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.travelUntil();
						Navigator.travelTo(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5, 5, false);
					}
				}

				else {
					Navigator.toStraightNavigator(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5, 8);
					turnTo(180);
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.travelUntil();
					Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 5, false);
				}
			}
			// go to ll thhen go to ll
			else {
				if (tunnel_UR_x - tunnel_LL_x > 1) {
					Navigator.toStraightNavigator(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5, 5);
					turnTo(90);
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.travelUntil();
					Navigator.travelTo(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5, 5, false);
				} else if (tunnel_UR_x - tunnel_LL_x == 1 && tunnel_UR_y - tunnel_LL_y == 1) {
					// entry is vertical
					if (isPointOnZone(tunnel_UR_x - 1, tunnel_UR_y, team_Zone_LL_x, team_Zone_LL_y, team_Zone_UR_x,
							team_Zone_UR_y)) {
						Navigator.toStraightNavigator(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5, 8);
						turnTo(180);
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.travelUntil();
						Navigator.travelTo(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 5, false);
					} else {
						Navigator.toStraightNavigator(tunnel_UR_x + 0.5, tunnel_UR_y - 0.5, 8);
						turnTo(270);
						forwardBy(-15);
						BigArmHook.setSpeed(80);
						BigArmHook.rotate(120);
						Navigator.travelUntil();
						Navigator.travelTo(tunnel_LL_x - 0.5, tunnel_LL_y + 0.5, 5, false);
					}
				} else {
					Navigator.toStraightNavigator(tunnel_LL_x + 0.5, tunnel_LL_y - 0.5, 5);
					turnTo(0);
					forwardBy(-15);
					BigArmHook.setSpeed(80);
					BigArmHook.rotate(120);
					Navigator.travelUntil();
					Navigator.travelTo(tunnel_UR_x - 0.5, tunnel_UR_y + 0.5, 5, false);
				}
			}
		}
	}

	/**
	 * 
	 * This method determines which tunnel entry to take depending on the current
	 * land the robot is on.
	 * 
	 * Compute which entry is on or within the border of the current land to decide
	 * which entry to take.
	 *
	 * Method is called by travelToTunnel method.
	 * 
	 * @param tunnel_ll_x: tunnel lower left x coordinate
	 * @param tunnel_ll_y: tunnel lower left y coordinate
	 * @param tunnel_ur_x: tunnel upper right x coordinate
	 * @param tunnel_ur_y: tunnel upper right y coordinate
	 * @param zone_ll_x: zone lower left x coordinate
	 * @param zone_ll_y: zone lower left y coordinate
	 * @param zone_ur_x: zone upper right x coordinate
	 * @param zone_ur_y: zone upper right y coordinate
	 * @return true if entry to take is lower left, false if entry to take is upper
	 *         right
	 */
	public static boolean isTunnelEntryLL(int tunnel_ll_x, int tunnel_ll_y, int tunnel_ur_x, int tunnel_ur_y,
			int zone_ll_x, int zone_ll_y, int zone_ur_x, int zone_ur_y) {
		// lower left x is in the zone
		if (tunnel_ll_x >= zone_ll_x && tunnel_ll_x <= zone_ur_x) {
			// lower left y is in the zone
			if (tunnel_ll_y >= zone_ll_y && tunnel_ll_y <= zone_ur_y) {
				return true;
			}
		}
		return false;
	}

	/**
	 * 
	 * This method determines which side of the tree to navigate to first.
	 * 
	 * Using the coordinate of the tree and current location, compute the distance
	 * of every line intersection one tile away from tree position. Return the side
	 * of the tree with the smallest distance.
	 * 
	 * Method is called by the Ev3Boot class.
	 * 
	 * @param ring_x: ring x coordinate
	 * @param ring_y: ring y coordinate
	 * @param robot_x: current x position
	 * @param robot_y: current y position
	 * @return 0 or 1 or 2 or 3, 0 = bottom 1 = right 2 = top 3 = left
	 */
	public static int closestSideOfTree(int ring_x, int ring_y, double robot_x, double robot_y) {
		double smallestDist = 999999;
		int position = -1;

		double x0 = ring_x * TILE_SIZE;
		double y0 = (ring_y - 1) * TILE_SIZE;
		if (Math.hypot(x0 - robot_x, y0 - robot_y) < smallestDist) {
			smallestDist = Math.hypot(x0 - robot_x, y0 - robot_y);
			position = 0;
		}

		int x1 = ring_x + 1;
		int y1 = ring_y;
		if (Math.hypot(x1 - robot_x, y1 - robot_y) < smallestDist) {
			smallestDist = Math.hypot(x1 - robot_x, y1 - robot_y);
			position = 1;
		}

		int x2 = ring_x;
		int y2 = ring_y + 1;
		if (Math.hypot(x2 - robot_x, y2 - robot_y) < smallestDist) {
			smallestDist = Math.hypot(x2 - robot_x, y2 - robot_y);
			position = 2;
		}

		int x3 = ring_x - 1;
		int y3 = ring_y;
		if (Math.hypot(x3 - robot_x, y3 - robot_y) < smallestDist) {
			smallestDist = Math.hypot(x3 - robot_x, y3 - robot_y);
			position = 3;
		}

		return position;
	}

	/**
	 * Method that look if a point is on the border of island. Used to determine if
	 * a side of a tree is visitable, the only cas it is not if it is on island
	 * border.
	 * 
	 * return true if it is on island border. return false if it is not on island.
	 * border
	 * 
	 * @param x: x coordinate of point to check if it is on island border
	 * @param y: y coordinate of point to check if it is on island border
	 * @return
	 */
	public static boolean isLandBorder(int x, int y) {
		int x1 = Wifi.getIsland_LL_x();
		int y1 = Wifi.getIsland_LL_y();
		int x2 = Wifi.getIsland_UR_x();
		int y2 = Wifi.getIsland_UR_y();

		// on x border
		if (x == x1 || x == x2) {
			return true;
		}
		if (y == y1 || y == y2) {
			return true;
		}
		return false;
	}

	/**
	 * Method used to check if a point is located inside or on the border of the
	 * stating island. Used by travelToTunnel method to know where is the entry
	 * located in the case the tunnel is of dimension 1x1.
	 * 
	 * return true if point is on starting zone. return false if point is not on
	 * starting zone
	 * 
	 * @param x: x-coordinate of point to check if it is on starting land
	 * @param y: y-coordinate of point to check if it is on starting land
	 * @param zone_ll_x: starting zone lower left x-coordinate
	 * @param zone_ll_y: starting zone lower left y-coordinate
	 * @param zone_ur_x: starting zone upper right x-coordinate
	 * @param zone_ur_y: starting zone upper right y-cooridnate
	 * @return
	 */
	public static boolean isPointOnZone(int x, int y, int zone_ll_x, int zone_ll_y, int zone_ur_x, int zone_ur_y) {
		// lower left x is in the zone
		if (x >= zone_ll_x && x <= zone_ur_x) {
			// lower left y is in the zone
			if (y >= zone_ll_y && y <= zone_ur_y) {
				return true;
			}
		}
		return false;
	}
}
