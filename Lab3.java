

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {
	
	// Left motor connected to output A
	// Right motor connected to output D
	// Sensor motor to output B
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	// Characteristics of our vehicle
	public static final double TRACK = 11.6;
	public static final double RADIUS = 2.1;
	
	public static void main(String[] args) {
		int buttonChoice;
		
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));	
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	
		float[] usData = new float[1];		
		UltrasonicPoller usPoller; 

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor, TRACK);

		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left    | Right >     ", 0, 0);
			t.drawString("          |             ", 0, 1);
			t.drawString("  Nav     | Nav         ", 0, 2);
			t.drawString("  w/ Block| w/o Block   ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			NavigatorAvoidance navigatorAvoidance = new NavigatorAvoidance(leftMotor, rightMotor, sensorMotor, odometer);
			usPoller = new UltrasonicPoller(usDistance, usData, navigatorAvoidance);
			odometer.start();
			odometryDisplay.start();
			navigatorAvoidance.start();
			usPoller.start();
			
		} else {
			Navigator navigator = new Navigator(leftMotor, rightMotor, odometer);
			odometer.start();
			odometryDisplay.start();
			navigator.start();
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
