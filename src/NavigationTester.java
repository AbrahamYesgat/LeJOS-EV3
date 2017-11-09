package localization;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationTester {
	private static final EV3LargeRegulatedMotor leftMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	public static void main(String[] args) {
		final TextLCD t=LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor, Resources.getTrack());
		OdometryDisplay odometryDisplay=new OdometryDisplay(odometer,t);
		Navigation navigator = new Navigation(leftMotor, rightMotor, odometer);
		
		int buttonChoice;
		do {
			t.clear();
			t.drawString("  PRESS ENTER  ", 0, 1);
			t.drawString("  TO START           ", 0, 2);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_ENTER);
		odometer.start();
		odometryDisplay.start();
		//SquareDriver.drive(leftMotor, rightMotor, Resources.getRadius(), Resources.getRadius(), Resources.getTrack());
		Sound.beep();
		navigator.travelTo(1, 1);
		navigator.travelTo(2, 0);
		navigator.travelTo(2, 2);
		
		while(Button.waitForAnyPress()!=Button.ID_ESCAPE);
		System.exit(0);
		
		
	}
}
