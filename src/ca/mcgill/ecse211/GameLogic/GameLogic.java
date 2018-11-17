package ca.mcgill.ecse211.GameLogic;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;

/**
 * This class contains helper method to facilitate game logic.
 * Contains methods to determine which tunnel entry to choose and which side of the ring set to navigate to first.
 */
public class GameLogic {
	
	/**
	 * 
	 * This method determines which tunnel entry to take depending on the current land the robot is on.
	 * 
	 * Compute which entry is on or within the border of the current land to decide which entry to take.
	 *
	 * Method is called by the Ev3Boot class.
	 * 
	 * @param tunnel_ll_x: tunnel lower left x coordinate
	 * @param tunnel_ll_y: tunnel lower left y coordinate
	 * @param tunnel_ur_x: tunnel upper right x coordinate
	 * @param tunnel_ur_y: tunnel upper right y coordinate
	 * @param zone_ll_x: current zone lower left x coordinate
	 * @param zone_ll_y: current zone lower left y coordinate
	 * @param zone_ur_x: current zone upper right x coordinate
	 * @param zone_ur_y: current zone upper right y coordinate
	 * @return true if entry to take is lower left, false if entry to take is upper right
	 */
	public static boolean isTunnelEntryLL(int tunnel_ll_x, int tunnel_ll_y, int tunnel_ur_x, int tunnel_ur_y, int zone_ll_x, int zone_ll_y, int zone_ur_x, int zone_ur_y)
	{
		//lower left x is in the zone
		if(tunnel_ll_x >= zone_ll_x && tunnel_ll_x <= zone_ur_x)
		{
			//lower left y is in the zone
			if(tunnel_ll_y >= zone_ll_y && tunnel_ll_y <= zone_ur_x)
			{
				return true;
			}	
		}
		return false;
	}
	
	
	/**
	 * 
	 * This method determines which side of the tree to navigate to first.
	 * 
	 * Using the coordinate of the tree and current location, compute the distance of every line intersection
	 * one tile away from tree position. Return the side of the tree with the smallest distance. 
	 * 
	 * Method is called by the Ev3Boot class.
	 * 
	 * @param ring_x: ring x coordinate
	 * @param ring_y: ring y coordinate
	 * @param robot_x: current x position
	 * @param robot_y: current y position
	 * @return 0 or 1 or 2 or 3
	 * 0 = bottom
	 * 1 = left
	 * 2 = top
	 * 3 = right
	 */
	public static int closestSideOfTree(int ring_x, int ring_y, double robot_x, double robot_y) 
	{
		double smallestDist = 999999;
		int position = -1;
		
		double x0 = ring_x * Ev3Boot.getTileSize();
		double y0 = (ring_y-1) * Ev3Boot.getTileSize();
		if(Math.hypot(x0-robot_x, y0-robot_y) < smallestDist)
		{
			smallestDist = Math.hypot(x0-robot_x, y0-robot_y);
			position = 0;
		}
		
		int x1 = ring_x-1;
		int y1 = ring_y;
		if(Math.hypot(x1-robot_x, y1-robot_y) < smallestDist)
		{
			smallestDist = Math.hypot(x1-robot_x, y1-robot_y);
			position = 1;
		}
		
		int x2 = ring_x;
		int y2 = ring_y+1;
		if(Math.hypot(x2-robot_x, y2-robot_y) < smallestDist)
		{
			smallestDist = Math.hypot(x2-robot_x, y2-robot_y);
			position = 2;
		}
		
		int x3 = ring_x+1;
		int y3 = ring_y;
		if(Math.hypot(x3-robot_x, y3-robot_y) < smallestDist)
		{
			smallestDist = Math.hypot(x3-robot_x, y3-robot_y);
			position = 3;
		}
		
		return position;
	}
}
