

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigatorAvoidance extends Thread implements UltrasonicController {

	// vehicle variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	private final double RADIUS, TRACK;
	
	// navigation variables
	private static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100;
	private static boolean followingWall = false;
	private int index = 0;
	private int[] avgVal = new int[3];
	
	private int distance;
	private double avgDistance = 20;
	
	private int count = 0;
	private double[][] points = {{1, 0}, {2, 1}, {2, 2}, {0, 2}, {1, 1}};
	
	// wall follower variables
	private static final int motorHigh = 200;

	public NavigatorAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,  EV3LargeRegulatedMotor sensorMotor,
			Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.RADIUS = Lab3.RADIUS;
		this.TRACK = Lab3.TRACK;
		sensorMotor.setAcceleration(100);
	}
	
	public void run() {
		leftMotor.stop();
		rightMotor.stop();
			
		for(; count < 5; count++) {
			travelTo(points[count][0], points[count][1]);
		}
	}
	
	private void travelTo(double x, double y) {
		x= x*30.48;
		y= y*30.48;
		
		
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();
		
		// Calculate the degree you need to change to
		double minAngle = Math.toDegrees(Math.atan2(deltaX, deltaY)) - odometer.getThetaDegrees();
		
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
		leftMotor.setSpeed(FORWARD_SPEED-150);
		rightMotor.setSpeed(FORWARD_SPEED-150);
		leftMotor.rotate(convertDistance(RADIUS,distance), true);
		rightMotor.rotate(convertDistance(RADIUS, distance), true);

		double initialTheta = 0;
		
		// If the motors are moving then the robot is navigating, if it is following the wall
		// we want to continue bang bang
		while((leftMotor.isMoving() && rightMotor.isMoving()) || followingWall) {
			if (followingWall) {
				// If you have passed the block, based on your final orientation 
				if (initialTheta - odometer.getThetaDegrees() >= 100) {
		    		leftMotor.stop(true);
		    		rightMotor.stop(true);
		    		sensorMotor.rotate(-60);
		    		followingWall = false;
		    		// We reduce the count so they go back to the point you were travelling to
		    		// before you encountered the obstacle
		    		this.count = count - 1;
		    		break;
		    	} else {
				    if (avgDistance <= 10) {
				    	leftMotor.setSpeed(motorHigh); // Start robot moving forward
				        rightMotor.setSpeed(motorHigh);
				        leftMotor.forward();
				        rightMotor.backward();
				    } else if (avgDistance > 17 && avgDistance < 27) {
				        leftMotor.setSpeed(motorHigh); // Start robot moving forward
				        rightMotor.setSpeed(motorHigh);
				        leftMotor.forward();
				        rightMotor.forward();
				  	} else if (avgDistance >= 17) {
				        leftMotor.setSpeed(110); // Left turn
				        rightMotor.setSpeed(motorHigh-10);
				        leftMotor.forward();
				        rightMotor.forward();
				    } else if (avgDistance <= 27){
				        leftMotor.setSpeed(motorHigh); // Right turn
				        rightMotor.setSpeed(100);
				        leftMotor.forward();
				        rightMotor.forward();
				    }
		    	}
			}
			
			
			// Obstacle detected
			// Save incidence angle to determine when you have passed the block
			if(avgDistance <= 12 && !followingWall) {
				followingWall = true;
				initialTheta = odometer.getThetaDegrees();
				turnTo(90);
				sensorMotor.rotate(60);
			}
		}
			
		
	    leftMotor.stop(true);
		rightMotor.stop(true);
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	
	
	@Override
	public void processUSData(int distance) {
		// Filter data and set the values to class variables
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


	// Convert how far they need to travel
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}


	// Determine the angle the motors need to turn
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
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}