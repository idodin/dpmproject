package ca.mcgill.ecse211.GameLogic;

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

}
