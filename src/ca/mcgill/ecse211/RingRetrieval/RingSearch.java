package ca.mcgill.ecse211.RingRetrieval;

import ca.mcgill.ecse211.Navigation.Navigator;

/**
 * This is the class's constructor.
 *
 */
public class RingSearch {
	
   /** Assuming the robot is at the tree location, this method makes the robot turn around the tree 
     * until it detects a ring.
     *  We call the Check color method from the checkColor class, which uses the light sensor
     *  to look for the four ring colors.
     *  When looking for rings, depending on the value of the detected color, 
     * it either calls the grasp method from RingGrasp, or it keeps turning around the tree 
     * to find a higher value ring.
     */
	public static void turnAroundTree(int position,int  ringSet_x,int ringSet_y)
	{
		switch (position) {
		case 0: 
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, true);
				Navigator.travelTo(ringSet_x-1, ringSet_y, 2, false);
				Navigator.travelTo(ringSet_x, ringSet_y+1, 2, false);
				Navigator.travelTo(ringSet_x+1, ringSet_y, 2, false);
				break;		
		
		case 1: Navigator.travelTo(ringSet_x-1, ringSet_y, 2, true);
				Navigator.travelTo(ringSet_x, ringSet_y+1, 2, false);
				Navigator.travelTo(ringSet_x+1, ringSet_y, 2, false);
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, false);
				break;
				
				
		case 2: Navigator.travelTo(ringSet_x, ringSet_y+1, 2, true);
				Navigator.travelTo(ringSet_x+1, ringSet_y, 2, false);
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, false);
				Navigator.travelTo(ringSet_x-1, ringSet_y, 2, false);
				break;
				
		case 3: Navigator.travelTo(ringSet_x+1, ringSet_y, 2, true);
				Navigator.travelTo(ringSet_x, ringSet_y-1, 2, false);
				Navigator.travelTo(ringSet_x-1, ringSet_y, 2, false);
				Navigator.travelTo(ringSet_x, ringSet_y+1, 2, false);
				break;
			
		}
	}


}
