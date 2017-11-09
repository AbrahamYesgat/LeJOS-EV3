package localization;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import localization.Odometer;

/** Navigation uses simple trigonometry along with information obtained from the odometer to be able to 
 * allow the robot to travel to certain x and y coordinates, or turn a specified number of degrees
 * @author Marine Huynh, Sihui Shen
 *
 */
public class Navigation {
	
	private static Odometer odometer = Resources.getOdometer();
	private static EV3LargeRegulatedMotor leftMotor = Resources.getLeftMotor();
	private static EV3LargeRegulatedMotor rightMotor = Resources.getRightMotor();
	private static EV3LargeRegulatedMotor zipMotor = Resources.getZipMotor();
	private static final double RADIUS = Resources.getRadius();
	private static final double TRACK = Resources.getTrack();

	public static final int FORWARD_SPEED = 250, ROTATE_SPEED = 100, MOTOR_ACCELERATION = 50;

	

	/**
	 * Primary navigation point of the program, performs all navigation based functions
	 * @param odometer2 
	 * @param rightmotor 
	 * @param leftmotor 
	 */
	

	/**
	 * Adjusts the rotation of the robot and it's heading to travel to designated coordinate point
	 * 
	 * @param x x coordinate in coordinate grid robot needs to travel to
	 * @param y y coordinate in coordinate grid robot needs to travel to
	 */
	public static void travelTo(double x, double y) {
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
		turnTo(minAngle, false);
		
		// calculate the distance to next point
		double distance  = Math.hypot(deltaX, deltaY);
		
		// move to the next point
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(RADIUS,distance), true);
		rightMotor.rotate(convertDistance(RADIUS, distance), false);

		leftMotor.stop(true);
		rightMotor.stop(true);
	}
	

	
	/**
	 * Designed to rotate the robot a preset number of degrees
	 * 
	 * @param theta The angle at which the robot needs to turn too
	 * @param block Whether or not the execution of the turn should block threads
	 */
	public static void turnTo(double theta, boolean block) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		int angle = convertAngle(RADIUS, TRACK, theta);
		
		leftMotor.rotate(angle, true);
		rightMotor.rotate(-angle, block);

	}
	
	
	/**
	 * Designed to rotate robot to face a direction defined by theta
	 * 
	 * @param theta The final orientation desired
	 */
	public static void pointTo(double theta) {
		double angleToTurn = theta - odometer.getThetaDegrees();
		System.out.println("Angle to turn: " + angleToTurn % 360);
		turnTo(angleToTurn % 360, false);
	}
	
	
	/**
	 * Used to stop both left and right motors in sync with each other
	 */
	public static void stopMotors() {
		leftMotor.stop();
		rightMotor.stop();
	}
	
	/**
	 * Allows the robot to drive a fixed number of cm either in reverse or forwards
	 * 
	 * @param distance the distance in cm that the robot will drive
	 * @param forward whether the robot will be going backwards or forwards
	 */
	public static void driveDistance(int distance, boolean forward) {
		if (forward) {
			leftMotor.rotate(convertDistance(RADIUS, distance), true);
			rightMotor.rotate(convertDistance(RADIUS, distance), false);
		} else {
			leftMotor.rotate(-convertDistance(RADIUS, distance), true);
			rightMotor.rotate(-convertDistance(RADIUS, distance), false);
		}

	}
	
	/**
	 * Used to synchronize leftMotor and rightMotor together
	 */
	public static void synchronizeMotors() {
		RegulatedMotor[] motors = new RegulatedMotor[] {leftMotor};
		rightMotor.synchronizeWith(motors);
	}
	
	/**
	 * Must be called before a set of synchronized subroutines
	 */
	public static void startSynchronization() {
		rightMotor.startSynchronization();
	}
	
	/**
	 * Called to end synchronization of motors and to signal an end of a synchronized subroutine
	 */
	public static void endSynchronization() {
		rightMotor.endSynchronization();
	}

	
	/**
	 * Used to set the motor speed and get the motor to travel in a constant direction,
	 * if values are negative the robot will travel backwards
	 * 
	 * @param leftM speed of the left motor
	 * @param rightM speed of the right motor
	 */
	public static void setSpeed(int leftM, int rightM) {
		leftMotor.setSpeed(leftM);
		rightMotor.setSpeed(rightM);
		
		if (leftM > 0) {
			leftMotor.forward();
		} else {
			leftMotor.backward();
		}
		
		if (rightM > 0) {
			rightMotor.forward();
		} else {
			rightMotor.backward();
		}
	}
	
	
	/**
	 * Used to set the motor acceleration
	 * 
	 * @param leftM acceleration of the left motor
	 * @param rightM acceleration of the right motor
	 */
	public static void setAcceleration(int acceleration) {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}
	
	/**
	 * Drives the zipline motor a predefined distance
	 * 
	 * @param distance The distance which the zipline motor needs to drive
	 */
	public static void driveZiplineDistance(double distance) {
		zipMotor.rotate(convertDistance(RADIUS, distance), false);
	}
	
	
	
	/**
	 * Drives left and right motor a certain distance in cm as defined by distance
	 * 
	 * @param distance The distance in cm the left and right motor should move by
	 */
	public static void driveLength(double distance) {
		leftMotor.rotate((int) (360*convertDistance(RADIUS, distance)), true);
		rightMotor.rotate((int) (360*convertDistance(RADIUS, distance)), false);
	}
	
	/**
	 * A set of instructions that all the robot to traverse the zipline from the point Xo, Yo
	 */
	public static void driveZipline() {
		leftMotor.setAcceleration(60);
		rightMotor.setAcceleration(60);
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		zipMotor.setSpeed(350);
		zipMotor.backward();

		driveLength(30.48 * 9);	
	}
	
	
	/**
	 * @param radius Wheel radius of the robot
	 * @param distance Angular distance needed to travel
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180*distance) / (Math.PI * radius));
	}

	/**
	 * @param radius Wheel radius of the robot
	 * @param width Track width of the robot
	 * @param angle Desired rotation angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		int output = convertDistance(radius, Math.PI * width * angle / 360.0);
		return output;
	}
	
	/**
	 * Indicates whether the robot is in the process of navigation
	 * 
	 * @return If the robot's left or right motor is moving it returns true
	 */
	public static boolean isNavigating() {
		return (leftMotor.isMoving() || rightMotor.isMoving());
	}
}
