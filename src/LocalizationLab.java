package localization;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * LocalizationLab uses a system to localize the robot using the ultrasonic sensor and the light sensor
 * the robot moves and it calculates its position. It can then be places on the (0,0) origin on the grid
 * @author Marine Huynh, Sihui Shen, Adam Gobran, Ali Shobeiri, Abe Yesgat, Reda El Khili
 * @since 10-04-2017
 *
 */
public class LocalizationLab {

	//create the ports
//	private static final EV3LargeRegulatedMotor leftMotor = 
//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
//	private static final EV3LargeRegulatedMotor rightMotor =
//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
//	private static final EV3LargeRegulatedMotor zipMotor =
//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
//	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static boolean isFallingEdge;
	private static double WHEEL_BASE = 9.75;
	private static double WHEEL_RADIUS = 2.1;
	

	/**
	 * TODO
	 * @param args
	 * @return nothing 
	 */
	public static void main(String[] args) {
		Resources resources = new Resources("A", "D", "B", "S4", "S1");
		int buttonChoice=0;

		//create the instances
		final TextLCD t=LocalEV3.get().getTextLCD();
		Odometer odometer=new Odometer();
		OdometryDisplay odometrydisplay=new OdometryDisplay(odometer,t);
		UltrasonicLocalizer usLocalizer;
		LightLocalization lightLocalizer;
		Navigation navigator = new Navigation();



//		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
//		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
//		// this instance
//		float[] usData = new float[usDistance.sampleSize()];
		
		SensorModes colorMode = Resources.getColorSensor();
		SampleProvider colorSensor = colorMode.getMode("Red");
		float[] colorData = new float[colorMode.sampleSize()];
		t.drawString("  TO START           ", 0, 3);
		buttonChoice=Button.waitForAnyPress();
		
		// usData is the buffer in which data are
		// returned  

//		// initiate integer to store coordinates
//		int xo=0;
//		int yo=0;
//		int xc=0;
//		int yc=0;
//
//		t.clear();
//		t.drawString("  Enter         ", 0, 0);
//		t.drawString("  X0, Y0           ", 0, 1);	
//		t.drawString("  PRESS ANYBUTTON  ", 0, 2);
//		t.drawString("  TO START           ", 0, 3);
//		Button.waitForAnyPress();
//
//		t.clear();
//		t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
//		t.drawString(" LEFT X-1", 0, 1);
//		t.drawString(" RIGHT X+1",0, 2);
//		t.drawString(" UP Y+1", 0, 3);
//		t.drawString(" DOWN Y-1 ", 0,4 );
//		t.drawString(" CONFIRM PRESS ENTER ", 0, 5);
//
//		buttonChoice= Button.waitForAnyPress();
//		t.clear();
//		while (buttonChoice!=Button.ID_ENTER) {
//			if(buttonChoice==Button.ID_LEFT) {
//				xo--;
//				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//			else if(buttonChoice==Button.ID_RIGHT) {
//				xo++;
//				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//			else if(buttonChoice==Button.ID_UP) {
//				yo++;
//				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//			else if(buttonChoice==Button.ID_DOWN) {
//				yo--;
//				t.drawString(" Xo="+xo +"Yo="+yo, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//
//			buttonChoice=Button.waitForAnyPress();
//		}
//		t.clear();
//		t.drawString(" X0="+xo +"Yo="+yo   , 0, 0);
//
//		t.drawString("   CONFIRMED   ", 0, 3);
//		
//		Button.waitForAnyPress();
//				
//		t.clear();
//		t.drawString(" Enter         ", 0, 0);
//		t.drawString(" XC,YC           ", 0, 1);
//		t.drawString(" PRESS ANYBUTTON  ", 0, 3);
//		t.drawString(" TO START         ", 0, 4);
//		Button.waitForAnyPress();
//		
//		t.clear();
//		t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
//		t.drawString(" LEFT X-1", 0, 1);
//		t.drawString(" RIGHT X+1",0, 2);
//		t.drawString(" UP Y+1", 0, 3);
//		t.drawString(" DOWN Y-1 ", 0,4 );
//		t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//		buttonChoice=Button.waitForAnyPress();
//
//
//		while (buttonChoice!=Button.ID_ENTER) {
//			if(buttonChoice==Button.ID_LEFT) {
//				xc--;
//				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//			else if(buttonChoice==Button.ID_RIGHT) {
//				xc++;
//				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//			else if(buttonChoice==Button.ID_UP) {
//				yc++;
//				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//			else if(buttonChoice==Button.ID_DOWN) {
//				yc--;
//				t.drawString(" Xc="+xc +"Yc="+yc, 0, 0);
//				t.drawString(" LEFT X-1", 0, 1);
//				t.drawString(" RIGHT X+1",0, 2);
//				t.drawString(" UP Y+1", 0, 3);
//				t.drawString(" DOWN Y-1 ", 0,4 );
//				t.drawString(" CONFIRM PRESS ENTER  ", 0, 5);
//			}
//
//			buttonChoice=Button.waitForAnyPress();
//		}
//		t.clear();
//		t.drawString(" Xc="+xc+"Yc="+yc, 0, 0);
//		t.drawString(" CONFIRMED   ", 0, 3);
//		setFallingEdge(true);

		do {

		} while(buttonChoice!=Button.ID_ENTER);

		if(buttonChoice == Button.ID_ENTER) {
			odometer.start();
			odometrydisplay.start();
			t.clear();
			usLocalizer = new UltrasonicLocalizer(odometer);
			usLocalizer.doLocalization();
//			lightLocalizer = new LightLocalization(odometer, colorSensor, colorData, navigator);
//			Navigation.driveDistance(10, true);
//			lightLocalizer.doLocalization(1, 1);
//			Navigator.turnTo(-10, false);
//			Navigation.travelTo(0, 0);
//			Navigation.turnTo(90, false);
//			lightLocalizer.doLocalization(0, 0);
		}
		
		// navigator.driveZipline();

		while(Button.waitForAnyPress()!=Button.ID_ESCAPE);
		System.exit(0);
	}

	/**
	 * method that checks if it is in falling edge mode
	 * @return boolean, true if is in falling edge
	 * 					false if it is not, which mean it is in rising edge
	 */
	public static boolean getFallingEdge() {
		return isFallingEdge;
	}

	/**
	 * setter for the falling edge
	 * @param isFallingEdge boolean that represents if we are using falling or rising edge localization method
	 * @return nothing
	 */
	public static void setFallingEdge(boolean isFallingEdge) {
		LocalizationLab.isFallingEdge = isFallingEdge;
	}


}
