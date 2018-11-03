package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.FinalProject.FinalProject;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class OdometerCorrections implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double SENSOR_OFFSET = 2.5;
  private static Odometer odometer;
  private static SampleProvider color = FinalProject.getColorBack();
  private static float[] colorBuffer = FinalProject.getColorBufferBack();
  private double[] currentPosition;
  public static final double WHEEL_RAD = FinalProject.getWheelRad();
	private static final double TRACK = FinalProject.getTrack();
  private int yCount;
  private int xCount;
  private int xInc;
  private int yInc;
  private double newX;
  private double newY;
  private int lastColor = 2;
  private float currentColor;
  private static final double TILE_SIZE = 30.48;
  public static boolean correction = false;
  public OdometerCorrections() {
	  
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
	  try {
		odometer = Odometer.getOdometer(FinalProject.getLeftmotor(), FinalProject.getRightmotor(), TRACK, WHEEL_RAD);
	} catch (OdometerExceptions e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}
    long correctionStart, correctionEnd;

    while (true) {
    	  if(!correction) {
    		  continue;
    	  }
      correctionStart = System.currentTimeMillis();
      
      //If detect black line.
      color.fetchSample(colorBuffer, 0);
      currentColor = colorBuffer[0];
      if(currentColor - lastColor > 5) {
    	  
    	  currentPosition = odometer.getXYT();
    	  Sound.beep();
    	  
    	  // How much do you increment by?
    	  yInc = Math.round((float)Math.cos(Math.toRadians(currentPosition[2])));
    	  xInc = Math.round((float)Math.sin(Math.toRadians(currentPosition[2])));
    	  
    	  yCount += yInc;
    	  xCount += xInc;
    	  
    	  //Are we crossing tile boundary from the upper or lower boundary?
    	  if (xInc < 0) {
    		  newX = xCount * TILE_SIZE ;
    	  } else if (xInc > 0) {
    		  newX = (xCount - 1) * TILE_SIZE ;
    	  } else {
    		  newX = currentPosition[0];
    	  }
    	  
    	  if (yInc < 0) {
    		  newY = yCount * TILE_SIZE + SENSOR_OFFSET;
    	  } else if (yInc > 0) {
    		  newY = (yCount - 1) * TILE_SIZE - SENSOR_OFFSET;
    	  } else {
    		  newY = currentPosition[1];
    	  }
    	  
    	  
    	  odometer.setXYT(newX, newY, currentPosition[2]);
    	  
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}