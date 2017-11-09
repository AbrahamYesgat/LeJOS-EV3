/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometerlab;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private EV3ColorSensor lightSensor;

  
  // constructor
  public OdometryCorrection(Odometer odometer, EV3ColorSensor lightsensor) {
		this.odometer = odometer;
		this.lightSensor = lightsensor;
	}
  // run method (required for Thread)
  public void run() {
		long correctionStart, correctionEnd;
		int northCount = 0;
		int eastCount = 0;
        double change= 30.48;
        double prevY = 0;
        double prevX = 0;
		while (true) {
			correctionStart = System.currentTimeMillis();
			if(lightSensor.getColorID()==Color.BLACK) {
				Sound.beepSequenceUp();
				if(odometer.getTheta() < 5 || (odometer.getTheta() > 355 && odometer.getTheta() < 360)) {
					odometer.setY(northCount*change);
					northCount++;
				} else if(odometer.getTheta() > 85 && odometer.getTheta() < 95) {
					odometer.setX(eastCount*change);
					eastCount++;
				} else if(odometer.getTheta() > 175  && odometer.getTheta() < 185 && northCount>=0) {
					odometer.setY((northCount-1)*change);
					northCount--;
				} else if(odometer.getTheta() > 265 && odometer.getTheta() < 275 && eastCount>=0) {
					odometer.setX((eastCount-1)*change);
					eastCount--;
				}
			}

     
			//counts lines passed and resets x and y accordingly
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }
}
