package odometer;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class TestOdometer {
	private static Odometer odometer;
	private static InfoDisplay displayer;
	private static EV3LargeRegulatedMotor leftMotor=new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static EV3LargeRegulatedMotor rightMotor=new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static TextLCD t;
	private static SquareDriver driver;
	private static TriangleDriver Tdriver;
	private static RandomMover mover;
	public static void main(String args[]) {
		t=LocalEV3.get().getTextLCD();
		driver=new SquareDriver();
		Tdriver=new TriangleDriver();
		mover=new RandomMover();
		odometer=new Odometer(leftMotor, rightMotor);
		displayer=new InfoDisplay(odometer, t);
		
		odometer.start();
		displayer.start();
		
		//test One
		(new Thread() {
			public void run() {
				//test One
				SquareDriver.drive(leftMotor, rightMotor, 2.13, 2.13, 16.9);
				//test Two
				//Tdriver.drive(leftMotor, rightMotor, 2.13, 2.13, 16.8);
				
				//Test three
				//mover.drive(leftMotor, rightMotor, 2.13, 2.13, 16.8);
			}
			
		}).start();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			System.exit(0);
		
		
	}

}
