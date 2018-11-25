package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.Ev3Boot.Ev3Boot;
import ca.mcgill.ecse211.Navigation.Navigator;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class OdometerCorrection {
	private static final EV3LargeRegulatedMotor leftMotor = Ev3Boot.getLeftmotor();
	private static final EV3LargeRegulatedMotor rightMotor = Ev3Boot.getRightmotor();

	private static SampleProvider colorSensorLeft = Ev3Boot.getColorLeft();
	private static SampleProvider colorSensorRight = Ev3Boot.getColorRight();
	private static float[] colorLeftBuffer = Ev3Boot.getColorLeftBuffer();
	private static float[] colorRightBuffer = Ev3Boot.getColorRightBuffer();
	
	private static final int CORRECTOR_SPEED = Navigator.getForwardSpeed()/3;
	public boolean isCorrecting = false;
	private static final int BLACK = 300;
	private static final int errorMargin = 150;
	
	private static float colorLeft;
	private static float colorRight;
	private static float oldColorLeft;
	private static float oldColorRight;

	private static final long CORRECTION_PERIOD = 10;
	static long correctionStart;
	static long correctionEnd;

	public static void correct() {

		leftMotor.setSpeed(Navigator.getForwardSpeed());
		rightMotor.setSpeed(Navigator.getForwardSpeed());
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		leftMotor.forward();
		rightMotor.forward();

		colorSensorLeft.fetchSample(colorLeftBuffer, 0);
		colorLeft = colorLeftBuffer[0] * 1000;

		colorSensorRight.fetchSample(colorRightBuffer, 0);
		colorRight = colorRightBuffer[0] * 1000;

		while (true) {
			correctionStart = System.currentTimeMillis();
			oldColorLeft = colorLeft;
			oldColorRight = colorRight;

			colorSensorLeft.fetchSample(colorLeftBuffer, 0);
			colorLeft = colorLeftBuffer[0] * 1000;

			colorSensorRight.fetchSample(colorRightBuffer, 0);
			colorRight = colorRightBuffer[0] * 1000;

			if (colorLeft - oldColorLeft > 19) {
				if (colorRight - oldColorRight > 19 || !pollMultiple(false))
					continue;
				

				leftMotor.stop(true);
				while (colorRight - oldColorRight <= 19) {
					if (rightMotor.getSpeed() != CORRECTOR_SPEED) {
						rightMotor.stop();
						rightMotor.setSpeed(CORRECTOR_SPEED);
						rightMotor.forward();
					}
					oldColorRight = colorRight;
					colorSensorRight.fetchSample(colorRightBuffer, 0);
					colorRight = colorRightBuffer[0] * 1000;
					System.out.println(colorRight-oldColorRight);
				}

				Sound.beep();
				rightMotor.stop();

				
				leftMotor.setSpeed(Navigator.getForwardSpeed());
				rightMotor.setSpeed(Navigator.getForwardSpeed());
				leftMotor.forward();
				rightMotor.forward();

				try {
					Thread.sleep(400);
					colorSensorRight.fetchSample(colorRightBuffer, 0);
					colorRight = colorRightBuffer[0] * 1000;
					colorSensorLeft.fetchSample(colorLeftBuffer, 0);
					colorLeft = colorLeftBuffer[0] * 1000;
					oldColorLeft = colorLeft;
					oldColorRight = colorRight;
					
				} catch (InterruptedException e) { }

			} else if (colorRight - oldColorRight > 19) {
				if (colorLeft - oldColorLeft > 19 || !pollMultiple(true))
					continue;
				System.out.println("Right line detected:\n" + (colorRight - oldColorRight));

				rightMotor.stop(true);
				while (colorLeft - oldColorLeft <= 19) {
					if (leftMotor.getSpeed() != CORRECTOR_SPEED) {
						leftMotor.stop();
						leftMotor.setSpeed(CORRECTOR_SPEED);
						leftMotor.forward();
					}
					oldColorLeft = colorLeft;
					colorSensorLeft.fetchSample(colorLeftBuffer, 0);
					colorLeft = colorLeftBuffer[0] * 1000;
					System.out.println(colorLeft-oldColorLeft);
				}

				Sound.beep();
				leftMotor.stop();

				leftMotor.setSpeed(Navigator.getForwardSpeed());
				rightMotor.setSpeed(Navigator.getForwardSpeed());
				leftMotor.forward();
				rightMotor.forward();

				try {
					Thread.sleep(400);
					colorSensorRight.fetchSample(colorRightBuffer, 0);
					colorRight = colorRightBuffer[0] * 1000;
					colorSensorLeft.fetchSample(colorLeftBuffer, 0);
					colorLeft = colorLeftBuffer[0] * 1000;
					oldColorLeft = colorLeft;
					oldColorRight = colorRight;
				} catch (InterruptedException e) {}
			}

			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {

				}
			}

		}

	}
	
	private static boolean pollMultiple(Boolean isRight) {
		int sampleCount = 10;
		float sum = 0;
		SampleProvider sample = isRight ? colorSensorRight : colorSensorLeft;
		float[] buffer = isRight ?  colorRightBuffer : colorLeftBuffer;
		
		for(int i = 0; i<sampleCount; i++) {
			sample.fetchSample(buffer, 0);
			sum += buffer[0] * 1000;
		}
		
		float avg = sum / sampleCount;
		if (avg > BLACK - errorMargin && avg < BLACK + errorMargin) {
			return true;
		}
		else {
			return false;
		}
		
		
	}

}
