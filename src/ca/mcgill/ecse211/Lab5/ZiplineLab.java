package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import localization.*;

/**
 * @author 
 * Mohamed Reda El-Khili
 * Adam Gobran
 * Marine Huynh
 * Sihui Shen
 * Ali Shobeiri
 * Abraham Yesgat
 * @version 1.1
 */

public class ZiplineLab {

	private SampleProvider colorSensor;
	private float[] colorSample;
	//create the ports
	private static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort=LocalEV3.get().getPort("S4");
	private static boolean isFallingEdge;


	/**
	 * main method that will display on the brick an interface
	 * that the client will be able to pass the coordinate of xo, yo, xc, yc, and the cornerStart
	 * (xo, yo) are the coordinates where the robot will need to reach before mounting the zipline
	 * (xc, yc) are the coordinates where the robot will pass by to mount the zipline
	 * @param args
	 * @return nothing 
	 */
	public static void main(String[] args) {
		int buttonChoice=0;

		//create the instances
		final TextLCD t=LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor,rightMotor);
		OdometryDisplay odometryDisplay =new OdometryDisplay(odometer,t);
		Navigator navigator = new Navigator(leftMotor, rightMotor, odometer);
		LightLocalization lightLocalizer;


		//created for the distance measured
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes colorSensor= new EV3ColorSensor(colorPort);
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		SampleProvider colorValue=colorSensor.getMode("Red");
		float[] usData = new float[usDistance.sampleSize()];
		float[] lightValue=new float[colorValue.sampleSize()];
		UltrasonicLocalizer usLocalizer=new UltrasonicLocalizer(leftMotor, rightMotor, 
				odometer, usDistance, usData);

		// initiate integer to store coordinates
		int xo=0;
		int yo=0;
		int xc=0;
		int yc=0;

		t.clear();
		t.drawString("  Enter         ", 0, 0);
		t.drawString("  X0, Y0           ", 0, 1);	
		t.drawString("  PRESS ANYBUTTON  ", 0, 2);
		t.drawString("  TO START           ", 0, 3);
		Button.waitForAnyPress();

		t.clear();
		t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
		t.drawString(" LEFT X-1", 0, 1);
		t.drawString(" RIGHT X+1",0, 2);
		t.drawString(" UP Y+1", 0, 3);
		t.drawString(" DOWN Y-1 ", 0,4 );
		t.drawString(" CONFIRM PRESS ENTER ", 0, 5);

		buttonChoice= Button.waitForAnyPress();
		t.clear();
		while (buttonChoice!=Button.ID_ENTER) {
			if(buttonChoice==Button.ID_LEFT) {
				xo--;
				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}
			else if(buttonChoice==Button.ID_RIGHT) {
				xo++;
				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}
			else if(buttonChoice==Button.ID_UP) {
				yo++;
				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}
			else if(buttonChoice==Button.ID_DOWN) {
				yo--;
				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}

			buttonChoice=Button.waitForAnyPress();
		}
		t.clear();
		t.drawString(" X0="+xo +"Yo="+yo   , 0, 0);

		t.drawString("   CONFIRMED   ", 0, 3);
		
		Button.waitForAnyPress();
				
		t.clear();
		t.drawString(" Enter         ", 0, 0);
		t.drawString(" XC,YC           ", 0, 1);
		t.drawString(" PRESS ANYBUTTON  ", 0, 3);
		t.drawString(" TO START         ", 0, 4);
		Button.waitForAnyPress();
		
		t.clear();
		t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
		t.drawString(" LEFT X-1", 0, 1);
		t.drawString(" RIGHT X+1",0, 2);
		t.drawString(" UP Y+1", 0, 3);
		t.drawString(" DOWN Y-1 ", 0,4 );
		t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
		buttonChoice=Button.waitForAnyPress();


		while (buttonChoice!=Button.ID_ENTER) {
			if(buttonChoice==Button.ID_LEFT) {
				xc--;
				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}
			else if(buttonChoice==Button.ID_RIGHT) {
				xc++;
				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}
			else if(buttonChoice==Button.ID_UP) {
				yc++;
				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}
			else if(buttonChoice==Button.ID_DOWN) {
				yc--;
				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
				t.drawString(" LEFT X-1", 0, 1);
				t.drawString(" RIGHT X+1",0, 2);
				t.drawString(" UP Y+1", 0, 3);
				t.drawString(" DOWN Y-1 ", 0,4 );
				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
			}

			buttonChoice=Button.waitForAnyPress();
		}
		t.clear();
		t.drawString(" Xc="+xc+"Yc="+yc, 0, 0);
		t.drawString(" CONFIRMED   ", 0, 3);
		odometer.run();
		Button.waitForAnyPress();

		usLocalizer.doLocalization();

		System.exit(0);

	}



}
