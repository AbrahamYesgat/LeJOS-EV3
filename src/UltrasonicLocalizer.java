package localization;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


/** UltrasonicLocalizer uses an ultrasonic sensor to detect walls surrounding the robot which ultimately
 * allows the robot to localize itself to theta = 0
 * @author Marine Hunyh, Sihui Shen
 *
 */
public class UltrasonicLocalizer {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private TextLCD LCD=LocalEV3.get().getTextLCD();
	private Odometer odometer;
	private SampleProvider usDistance;
	private float[] usData;

	private static final int ROTATE_SPEED=70;
	private static final int D=55;
	private static int distance;
	private static int dT;
	private double thetaA, thetaB;


	/**
	 * Constructor for the class UltrasonicLocalizer, that uses the robot's motors to rotate on itself, and the odometer
	 * to know its position related to the grid. It localizes its surroundings by collecting data in an array.
	 * @param leftMotor, Motor for the left wheel
	 * @param rightMotor, Motor for the right wheel
	 * @param odometer, uses odometry to display the position of the robot
	 * @param us, sample provider
	 * @param usData, array that collects the data
	 */
	public UltrasonicLocalizer(Odometer odometer) {
		this.leftMotor=Resources.getLeftMotor();
		this.rightMotor=Resources.getRightMotor();
		this.odometer=odometer;
		
		this.usDistance = Resources.getUltrasonicSensor().getMode("Distance");
		this.usData=new float[usDistance.sampleSize()];

	}

	/**
	 * Main method that runs the program once the LocalizationLab starts the program.
	 * This is activated by a button press in the LocalizationLab, and will run doLocalization.
	 * doLocalization calls two different methods depending on the press in LocalizationLab,
	 * fallingEdge() and risingEdge()
	 * it will execute one of the following
	 */
	public void doLocalization() {
		Navigation.setAcceleration(200);
		//this is a falling edge 
		fallingEdge();
	}
	

	//collecting data for the distances
	public int collectData() {
		usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] *100);//takes value from the buffer in cm
		
		if(distance>D) {
			distance=D;
		}
		return distance;
	}

	
	public void reverseIntoWall() {
		Navigation.setSpeed(ROTATE_SPEED + 100, ROTATE_SPEED + 100);
		
		//rotate clockwise until it sees no wall
		while(collectData()<D) {
			rightMotor.forward();
			leftMotor.backward();
		}
		
		Navigation.driveDistance(20, false);
	}
	
	
	/**
	 * Method that uses the falling edge, where the distance seen by the robot is very large, as it is facing no wall,
	 * and falling as it turns and finds a wall. This is the falling point
	 * It uses the falling points to localize itself in space and time, to be at the 0 degree angle, which is facing
	 * the positive y-axis
	 * 
	 * @return nothing
	 */
	public void fallingEdge() {
//		Navigation.turnTo(-360, false);
		//set rotation speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		//rotate clockwise until it sees no wall
		while(collectData()<D) {
			leftMotor.forward();
			rightMotor.backward();
		}

			//keep rotating until the robot sees the wall,then get the angel
			leftMotor.forward();
			rightMotor.backward();
			boolean isTurning=true;
	
			while(isTurning) {
				if(collectData()<D) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					isTurning=false;
				}
			}
		Sound.beep();
		thetaA=odometer.getThetaDegrees();
		thetaA=normalizeTheta(thetaA);
		
		//switch direction until it sees no wall
		while(collectData()<D) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
		//keep rotation until the robot sees the wall
		leftMotor.backward();
		rightMotor.forward();
		
		isTurning=true;
		while(isTurning) {
			if(collectData()<D) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				isTurning=false;
			}
		}
		Sound.beep();
		
		thetaB=odometer.getThetaDegrees();
		thetaB=normalizeTheta(thetaB);
		Sound.beep();
		
		if(thetaA>thetaB) {
			dT= 45-(int) (thetaA+thetaB)/2;
		}
		else if(thetaA<thetaB) {
			dT= 225-(int) (thetaA+thetaB)/2;
		} 
		double currentTheta= odometer.getThetaDegrees();

		double newtheta=currentTheta+dT;
		odometer.setTheta(newtheta);
		if (newtheta > 180) {
			Navigation.turnTo(360 - newtheta, false);
		} else {
			Navigation.turnTo(-newtheta, false);
		}
		odometer.setPosition(new double [] {0, 0, 0}, 
				new boolean [] {true, true, true});
		
		Navigation.driveDistance(10, false);
	}
	
	/**
	 * Method that constraint the values of the thetas to be in degree between the values of [0, 360)
	 * 
	 * @param theta which is the angle in degree
	 * @return theta, which is the angle in degree between [0,360)
	 */
	public double normalizeTheta(double theta) {
		if (theta >= 360) {
			theta = theta - 360;
		}
		else if( theta < 0) {
			theta = theta +360;
		}
		return theta;
	}

	/**
	 * This method is to know whether or not the robot is traveling
	 * 
	 * @return Boolean isTravelling
	 */
	boolean isNavigating() {
		boolean isTravelling = true;
		return isTravelling;
	}
}
