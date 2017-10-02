

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
	
	private int distance;
	private double avgDistance = 20;
	
	private int count = 0;
	private double[][] points = {{0, 2}, {1, 1}, {2, 2}, {2, 1}, {1, 0}};
	
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
	
	public void run() {
		leftMotor.stop();
		rightMotor.stop();
			
		for(; count < 5; count++) {
			travelTo(points[count][0], points[count][1]);
		}
	}
	
	private void travelTo(double x, double y) {
		System.out.println("x " + x);
		System.out.println("y " + y);
		
		navigating = true;
		x= x*30.48;
		y= y*30.48;
		
		
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();
		
		double minAngle = Math.toDegrees(Math.atan2(deltaX, deltaY)) - odometer.getThetaDegrees();
		
		if (minAngle < -180) {
			// System.out.println("minAngle < -180");
			minAngle = 360 + minAngle;
			// System.out.println("minAngle: " + minAngle);
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
		rightMotor.rotate(convertDistance(RADIUS, distance), true);

		while((leftMotor.isMoving() && rightMotor.isMoving())) {
			if (followingWall) {
				if (initialTheta - odometer.getThetaDegrees() >= 50) {
		    		System.out.println("Theta statement");

		    		leftMotor.stop(true);
		    		rightMotor.stop(true);
		    		sensorMotor.rotate(-60);
		    		followingWall = false;
		    		count --;
		    		break;
		    	} else {
		    		System.out.println("Bang Bang Bang");
				    if (avgDistance <= 10) {
				    	System.out.println("less than 15");
				    	leftMotor.setSpeed(motorHigh); // Start robot moving forward
				        rightMotor.setSpeed(motorHigh);
				        leftMotor.forward();
				        rightMotor.backward();
				    } else if (avgDistance > 20 && avgDistance < 30) {
				    	System.out.println("between 25 and 35");
				        leftMotor.setSpeed(motorHigh); // Start robot moving forward
				        rightMotor.setSpeed(motorHigh);
				        leftMotor.forward();
				        rightMotor.forward();
				  	} else if (avgDistance >= 20) {
				  		System.out.println("greater than 15");
				        leftMotor.setSpeed(100); // Left turn
				        rightMotor.setSpeed(motorHigh-20);//motorLow-20
				        leftMotor.forward();
				        rightMotor.forward();
				    } else if (avgDistance <= 30){
				    	System.out.println("less than 35");
				        leftMotor.setSpeed(motorHigh); // Right turn
				        rightMotor.setSpeed(100);
				        leftMotor.forward();
				        rightMotor.forward();
				    }
		    	}
			}
			
			if(avgDistance <= 15 && !followingWall) {
				System.out.println("Obstacle detected");
				followingWall = true;
				initialTheta = odometer.getThetaDegrees();
				turnTo(90);
				sensorMotor.rotate(60);
			}
		}
			
		
	    leftMotor.stop(true);
		rightMotor.stop(true);

		navigating = false;
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	
	
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
	    
	    if (followingWall) {
	    	this.avgDistance = avgDistance / Math.sqrt(2);
	    } else {
	    	this.avgDistance = avgDistance;
	    }
	    
	    this.distance = avgDistance;
	  }


	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}


	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
	

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
		
//		leftMotor.setSpeed(0);
//		rightMotor.setSpeed(0);
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	
	

	
}