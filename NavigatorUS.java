

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigatorUS extends Thread implements UltrasonicController {

	// vehicle variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	private final double RADIUS, TRACK;
	private final int MOTOR_ACCELERATION = 200;
	
	// navigation variables
	private static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100;
	private static boolean navigating = true;
	private static boolean followingWall = false;
	private static double navigatingX, navigatingY;
	private int index = 0;
	private int[] avgVal = new int[3];
	
	// variables to store sensor data
	private int distance;
	
	
	// wall follower variables
	private static final int motorLow = 50, motorHigh = 200, bandCenter = 10, bandwidth = 3, FILTER_OUT = 20;

	// obstacle avoidance variables
	private double initialTheta;
	private static boolean hasBlockPassed = false;
	private static boolean isPassingBlock = false;

	public NavigatorUS(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,  EV3LargeRegulatedMotor sensorMotor,
			Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.RADIUS = Lab3.RADIUS;
		this.TRACK = Lab3.TRACK;
	}
	
	/**
	 * Reads our sensor distance
	 */
	public int readUSDistance() {
		return this.distance;
	}
	
	/**
	 * Processes our sensor data and acts accordingly based on its readings
	 */
	
	@Override
	public void processUSData(int distance) {
		distance = Math.min(distance, 100);
		this.index++;
	    if(this.index == 3) {
	    	this.index = 0;
	    }
	    // averages last 3 values in array to filter out error values of distance
	    
	    avgVal[index] = distance;
	    
	    int avgDistance = 0;
	    for (int i=0; i<avgVal.length; i++) {
	    	avgDistance+=avgVal[i];
	    }
	    avgDistance = avgDistance / 3;
	    
	    this.distance = avgDistance;
	    
	    
	    if(followingWall) {	
	    	System.out.println("initialTheta: " + initialTheta);
	    	System.out.println("getTheta: " + odometer.getTheta());
	    	System.out.println("Subtract initial from current " + (initialTheta - Math.toDegrees(odometer.getTheta())));
	    	if (initialTheta - Math.toDegrees(odometer.getTheta()) >= 90) {
	    		sensorMotor.rotate(-45);
	    		turnTo(Math.toRadians(90.0));
	    		leftMotor.stop(true);
	    		rightMotor.stop(true);
	    	} else {
			    if (avgDistance <= 15) {
			    	leftMotor.setSpeed(motorHigh); // Start robot moving forward
			        rightMotor.setSpeed(motorHigh);
			        leftMotor.forward();
			        rightMotor.backward();
			    } else if (avgDistance > 25 && distance < 35) {
			        leftMotor.setSpeed(motorHigh); // Start robot moving forward
			        rightMotor.setSpeed(motorHigh);
			        leftMotor.forward();
			        rightMotor.forward();
			  	} else if (avgDistance >= 25) {
			        leftMotor.setSpeed(100); // Left turn
			        rightMotor.setSpeed(motorHigh-20);//motorLow-20
			        leftMotor.forward();
			        rightMotor.forward();
			    } else if (avgDistance <= 35){
			        leftMotor.setSpeed(motorHigh); // Right turn
			        rightMotor.setSpeed(100);
			        leftMotor.forward();
			        rightMotor.forward();
			    }
	    	}
	    } else if (avgDistance <= 15) {
	        leftMotor.stop(true); 
	        rightMotor.stop(true);
	        initialTheta = odometer.getThetaDegrees();
	        turnTo(Math.toRadians(90.0));
	        sensorMotor.rotate(45);
	        followingWall = true;
	        System.out.println(initialTheta);
	    } else {
	    	return;
	    }
	    
//	    else if (avgDistance > 25 && distance < 35) {
//	        leftMotor.setSpeed(motorHigh); // Start robot moving forward
//	        rightMotor.setSpeed(motorHigh);
//	        leftMotor.forward();
//	        rightMotor.forward();
//	  	} else if (avgDistance >= 25) {
//	        leftMotor.setSpeed(100); // Left turn
//	        rightMotor.setSpeed(motorHigh-20);//motorLow-20
//	        leftMotor.forward();
//	        rightMotor.forward();
//	    } else if (avgDistance <= 35){
//	        leftMotor.setSpeed(motorHigh); // Right turn
//	        rightMotor.setSpeed(100);
//	        leftMotor.forward();
//	        rightMotor.forward();
//	    }
	    
	    this.distance = avgDistance;
	  }

	
	/**
	 * Our main run method
	 * 
	 */
	public void run() {
		leftMotor.stop();
		leftMotor.setAcceleration(MOTOR_ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(MOTOR_ACCELERATION);

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
		
//		System.out.println(" ");
//		System.out.println("Travelling to x: " + x + ", y: " + y);
		
		navigating = true;
		x= x*30.48;
		y= y*30.48;
		
		System.out.println("x " + x);
		System.out.println("y " + y);
		
//		System.out.println("Odometer X " + odometer.getX());
//		System.out.println("Odometer Y " + odometer.getY());
		
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();
		
//		System.out.println("deltaX: " + deltaX);
//		System.out.println("deltaY: " + deltaY);
//
//		System.out.println(" ");
		
		// calculate the minimum angle
		double minAngle = Math.toDegrees(Math.atan2(deltaX, deltaY)) - odometer.getThetaDegrees();
		
		System.out.println("minAngle before correction " + minAngle);
		
		if (minAngle < -180) {
			// System.out.println("minAngle < -180");
			minAngle = 360 + minAngle;
			// System.out.println("minAngle: " + minAngle);
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
		navigating = false;
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
			leftMotor.rotate(-convertAngle(RADIUS, TRACK, -(theta*180)/Math.PI), true);
			rightMotor.rotate(convertAngle(RADIUS, TRACK, -(theta*180)/Math.PI), false);
		} 
		else { // angle is positive, turn to the right
			leftMotor.rotate(convertAngle(RADIUS, TRACK, (theta*180)/Math.PI), true);
			rightMotor.rotate(-convertAngle(RADIUS, TRACK, (theta*180)/Math.PI), false);
		}
		
	}

	
	/**
	 * A method to that implements our wall following logic
	 */
//	private void excecuteWallFollow() {
//		// calculate our offset from the bandCenter
//		int error = distance - bandCenter - 5; // -5 for distance from sensor to side of vehicle
//		
//		// Keep moving forward if vehicle is within threshold value
//		if ( Math.abs(error) < this.bandwidth ) {
//			steerStraight();
//		} 
//		// We are too close to the wall, steer vehicle to the right
//		else if ( error < 0 ) {
//			steerRight(); 
//		} 
//		// We are too far away from the wall
//		else { 
//			if ( error > 100 ) {
//				// It is just sensing something very far away, keep going straight
//				steerStraight(); 
//			} else {
//				// We are too far from the wall, steer left
//				steerLeft();
//			}
//		}
//	}
	
	

	
}