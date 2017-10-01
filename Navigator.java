

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread {
	 
	// vehicle variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final double RADIUS, TRACK;
	private final int MOTOR_ACCELERATION = 200;
	
	// navigation variables
	private static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100;
	private static boolean isNavigating = true;

	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.RADIUS = Lab3.RADIUS;
		this.TRACK = Lab3.TRACK;
	}
	
	/**
	 * Our main run method
	 */
	public void run() {
		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(MOTOR_ACCELERATION);
		}
		// travel to coordinates
		travelTo(0, 1);
		travelTo(1, 2);
		travelTo(1, 0);
		travelTo(2, 1);
		travelTo(2, 2);
	}
	
	/**
	 * Determine how much the motor must rotate for vehicle to reach a certain distance
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Determine the angle our motors need to rotate in order for vehicle to turn a certain angle 
	 * 
	 * @param radius
	 * @param TRACK
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
	
	/**
	 * A method to drive our vehicle to a certain cartesian coordinate
	 * 
	 * @param x X-Coordinate
	 * @param y Y-Coordinate
	 */
	private void travelTo(double x, double y) {
		
		System.out.println(" ");
		System.out.println("Travelling to x: " + x + ", y: " + y);
		
		isNavigating = true;
		x= x*30.48;
		y= y*30.48;
		
		System.out.println("x " + x);
		System.out.println("y " + y);
		
		System.out.println("Odometer X " + odometer.getX());
		System.out.println("Odometer Y " + odometer.getY());
		
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();
		
		System.out.println("deltaX: " + deltaX);
		System.out.println("deltaY: " + deltaY);

		System.out.println(" ");
		
		// calculate the minimum angle
		double minAngle = Math.toDegrees(Math.atan2(deltaX, deltaY)) - odometer.getThetaDegrees();
		
		System.out.println("minAngle before correction " + minAngle);
		
		if (minAngle < -180) {
			System.out.println("minAngle < -180");
			minAngle = 360 + minAngle;
			System.out.println("minAngle: " + minAngle);
		} else if (minAngle > 180) {
			System.out.println("minAngle > 180");
			minAngle = minAngle - 360;
			System.out.println("minAngle: " + minAngle);
		}
		// turn to the minimum angle
		turnTo(minAngle);
		
		// calculate the distance to next point
		double distance  = Math.hypot(deltaX, deltaY);
		
		// move to the next point
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(RADIUS,distance), true);
		rightMotor.rotate(convertDistance(RADIUS, distance), false);
//pause
		leftMotor.stop(true);
		rightMotor.stop(true);
		//correction?
		//odometer.setX(x + odometer.getX());
		//odometer.setY(y + odometer.getY());
		//odometer.setTheta(Math.atan2( deltaX, deltaY ) - odometer.getTheta());
		isNavigating = false;
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	private void turnTo(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		if(theta < 0) { // if angle is negative, turn to the left
			leftMotor.rotate(-convertAngle(RADIUS, TRACK, -theta), true);
			rightMotor.rotate(convertAngle(RADIUS, TRACK, -theta), false);
		} 
		else { // angle is positive, turn to the right
			leftMotor.rotate(convertAngle(RADIUS, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(RADIUS, TRACK, theta), false);
		}
		
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	/**
	 * A method to determine whether another thread has called travelTo and turnTo methods or not
	 * 
	 * @return
	 */
	private boolean isNavigating() {
 return false;
	}

	
}
