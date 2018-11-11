package ca.mcgill.ecse211.GameLogic;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;

public class GameLogic {
	
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
	
	// return 0 or 1 or 2 or 3
	// 0 = bottom
	// 1 = left
	// 2 = top
	// 3 = right
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
