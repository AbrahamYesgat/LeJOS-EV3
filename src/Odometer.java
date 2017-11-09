package localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** Odometer uses the number of wheel rotations the robot has completed, along with some simple trigonometry to
 * determine the robot's x, y, and theta coordinates.
 * @author Marine Huynh, Sihui Shen
 *
 */
public class Odometer extends Thread {
	//ALL DISTANCE VALUES ARE IN CM,THETA IN RAD-> CONVERT TO DEGREES
	// robot position
	private double x, y, theta, track;
	private int currentLeftMotorTachoCount, currentRightMotorTachoCount,
				prevLeftMotorTachoCount, prevRightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;
	
	// circumference of our wheel given a radius of 2.1cm
	private static final double WHEEL_CIRCUM = Math.PI*2*Resources.getRadius();

	// lock object for mutual exclusion
	public Object lock;

	// default constructor
	public Odometer() {
		this.leftMotor = Resources.getLeftMotor();
		this.rightMotor = Resources.getRightMotor();
		this.track = Resources.getTrack();
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.currentLeftMotorTachoCount = 0;
		this.currentRightMotorTachoCount = 0;
		this.prevLeftMotorTachoCount = 0;
		this.prevRightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			
			// Get current tachometer values
			currentLeftMotorTachoCount = leftMotor.getTachoCount();
			currentRightMotorTachoCount = rightMotor.getTachoCount();
			
			// Compare it with the previous value to get the change
			int leftDeltaTacho = currentLeftMotorTachoCount - prevLeftMotorTachoCount;
			int rightDeltaTacho = currentRightMotorTachoCount - prevRightMotorTachoCount;
			
			// Use our change in rotation values to calculate displacement of each wheel
			double leftMotorDisplacement = WHEEL_CIRCUM*leftDeltaTacho/360;
			double rightMotorDisplacement = WHEEL_CIRCUM*rightDeltaTacho/360;
			
			// angle at which our vehicle changed
			double thetaChange = ( leftMotorDisplacement - rightMotorDisplacement )/track;
			// change in distance of our vehicle
			double displacement = ( leftMotorDisplacement + rightMotorDisplacement )/2;
			
			prevLeftMotorTachoCount = currentLeftMotorTachoCount;
			prevRightMotorTachoCount = currentRightMotorTachoCount;
			

			synchronized (lock) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here!
				 * Only update the values of x, y, and theta in this block. 
				 * Do not perform complex math
				 * 
				 */
				theta += thetaChange;
				x += displacement*Math.sin(theta);
				y += displacement*Math.cos(theta);
				
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				if(theta>=0) {
	  				position[2] = ( theta * 360 / ( 2 * Math.PI ) ) % 360;
	 			} else {
	 			    position[2] = (( theta * 360 / ( 2 * Math.PI ) ) % 360)+360;
	 			}
		}
	}

	/** Returns class variable x
	 * @return
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/** Returns class variable y
	 * @return
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/** Returns class variable theta in radians
	 * @return
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}
	
	/** Returns class variable theta in degrees ensuring it is in the range [0, 360)
	 * @return
	 */
	public double getThetaDegrees() {
		double result;

		synchronized (lock) {
			if(theta>=0) {
				result = Math.toDegrees(theta) % 360;
 			} else {
 				result = Math.toDegrees(theta)+360;
 			}
		}

		return result;
	}

	
	/** Updates x, y, and theta in one function call
	 * @param position array containing x, y, and theta
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = Math.toRadians(position[2]);
		}
	}

	/** Sets class variable x to desired coordinate in cm relative to x = 0
	 * @param x desired x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}
	
	
	/** Sets class variable y to desired coordinate in cm relative to y = 0
	 * @param y desired y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}
	
	
	/** Sets theta class variable to desired angle in degrees
	 * @param theta desired theta
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the currentLeftMotorTachoCount
	 */
	public int getCurrentLeftMotorTachoCount() {
		return currentLeftMotorTachoCount;
	}

	/**
	 * @param currentLeftMotorTachoCount the currentLeftMotorTachoCount to set
	 */
	public void setCurrentLeftMotorTachoCount(int currentLeftMotorTachoCount) {
		synchronized (lock) {
			this.currentLeftMotorTachoCount = currentLeftMotorTachoCount;	
		}
	}

	/**
	 * @return the currentRightMotorTachoCount
	 */
	public int getCurrentRightMotorTachoCount() {
		return currentRightMotorTachoCount;
	}

	/**
	 * @param currentRightMotorTachoCount the currentRightMotorTachoCount to set
	 */
	public void setCurrentRightMotorTachoCount(int currentRightMotorTachoCount) {
		synchronized (lock) {
			this.currentRightMotorTachoCount = currentRightMotorTachoCount;	
		}
	}
}