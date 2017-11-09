package odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RandomMover {
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 150;

	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double width) {
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		//rotate 45 degrees
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, width, 45.0), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, 45.0), false);
			
		//move forward
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, 30.96), true);
		rightMotor.rotate(convertDistance(rightRadius, 30.96), false);
		
		//ROTATE 45 Degrees
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, width, -45), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, -45), false);
			
		//move forward
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, 10.96), true);
		rightMotor.rotate(convertDistance(rightRadius, 10.96), false);
		
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, width, 90.0), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, 90.0), false);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, 10.96), true);
		rightMotor.rotate(convertDistance(rightRadius, 10.96), false);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, width, -90.0), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, -90.0), false);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, 30.96), true);
		rightMotor.rotate(convertDistance(rightRadius, 30.96), false);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(leftRadius, width, -60.0), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, -60.0), false);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, 40.96), true);
		rightMotor.rotate(convertDistance(rightRadius, 40.96), false);
			
		}
	

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
