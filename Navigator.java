

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread {
	// Tharsan is a boss ;)  
	
	
	// vehicle variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final double RADIUS, TRACK;
	private final int MOTOR_ACCELERATION = 200;
	
	// navigation variables
	private static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100;

	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.RADIUS = Lab3.RADIUS;
		this.TRACK = Lab3.TRACK;
	}
	

	public void run() {
		leftMotor.stop();
		leftMotor.setAcceleration(MOTOR_ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(MOTOR_ACCELERATION);

		// Call points in order 
		travelTo(1, 0);
		travelTo(2, 1);
		travelTo(2, 2);
		travelTo(0, 2);
		travelTo(1, 1);
	}
	

	// Convert how far they need to travel
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}


	// Determine the angle the motors need to turn
	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
	

	// Drives robot to specified cartesian coordinate
	private void travelTo(double x, double y) {
		x= x*30.48;
		y= y*30.48;
		
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();
		
		
		// calculate the minimum angle
		double minAngle = Math.toDegrees(Math.atan2(deltaX, deltaY)) - odometer.getThetaDegrees();
		
		// Adjust the angle to make sure it takes the min angle
		if (minAngle < -180) {
			minAngle = 360 + minAngle;
		} else if (minAngle > 180) {
			minAngle = minAngle - 360;
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

		leftMotor.stop(true);
		rightMotor.stop(true);
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	

	// Turn to the min angle that you have chosen
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
	
}
