package org.usfirst.frc.team86.robot;



import org.usfirst.frc.team86.robot.MecanumDrive.DriveType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
	
	private MecanumDrive drive;
	private NavX gyro;
	
	private boolean grabAngleOnce = false;
	private double holdAngle = 0;
	
	private double kP = 0.025;
	private double kI;
	private double kD = 0.03;
	private double kF;
	private double maxI;

	private double scaledX;
	private double scaledY;
	private double scaledRotate;
	
	private SpeedController leftFrontMotor;
	private SpeedController leftBackMotor;
	private SpeedController rightFrontMotor;
	private SpeedController rightBackMotor;
	
	public Drive(SpeedController leftFrontMotor,
			SpeedController leftRearMotor, 
			SpeedController rightFrontMotor, 
			SpeedController rightRearMotor, 
			NavX gyro) {
		this.leftFrontMotor = leftFrontMotor;
		this.leftBackMotor = leftRearMotor;
		this.rightFrontMotor = rightFrontMotor;
		this.rightBackMotor = rightRearMotor;
		
		this.drive = new MecanumDrive(leftFrontMotor, leftRearMotor,
				rightFrontMotor, rightRearMotor, gyro);
		this.gyro = gyro;
		
	}
	
	
	public void init() {
		drive.setPIDValues(kP, kI, kD, kF, maxI);
	}
	
	public void update() {
		SmartDashboard.putNumber("left front power", leftFrontMotor.get());
		SmartDashboard.putNumber("left back power", leftBackMotor.get());
		SmartDashboard.putNumber("right front power", rightFrontMotor.get());
		SmartDashboard.putNumber("right back power", rightBackMotor.get());
		SmartDashboard.putNumber("Gyro", gyro.getAngle()); 
		double x = .7*(JoystickIO.leftJoystick.getX());
		if(x < .15 && x > -.15){
			 scaledX = 0;	
		}else{
			 scaledX = x;
		}
		double y = .7*(JoystickIO.leftJoystick.getY());
		if(y < .15 && y > -.15){
			 scaledY = 0;	
		}else{
			 scaledY = y;
		}
		
		double r = .7*(JoystickIO.rightJoystick.getX());
		if(r < .15 && r > -.15){
			 scaledRotate = 0;	
		}else{
				scaledRotate = r;
		}		
		SmartDashboard.putBoolean("bthnholdhelf", JoystickIO.btnHoldLeft.isDown() || JoystickIO.rightJoystick.getRawButton(4));
		if (JoystickIO.btnHoldLeft.isDown()) {
			drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, 0, 330);
			grabAngleOnce = true;
			
		} else if (JoystickIO.btnHoldCenter.isDown()) {
			drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, 0, 270);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldRight.isDown()) {
			drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, 0, 210);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldLeftHopper.isDown()) {
			drive.drive(DriveType.ROTATE_PID, -1, scaledY, 0, 0);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldRightHopper.isDown()) {
			drive.drive(DriveType.ROTATE_PID, 1, scaledY, 0, 180);
			grabAngleOnce = true;
		} else {
			SmartDashboard.putNumber("Angle Being Held", holdAngle);
			if (scaledRotate == 0) {
				SmartDashboard.putBoolean("Is Holding Angle", true);
				if (grabAngleOnce) {
					grabAngleOnce = false;
					holdAngle = gyro.getNormalizedAngle();
				}
				drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, scaledRotate, holdAngle);
			} else {
				grabAngleOnce = true;
				drive.drive(DriveType.STICK_FIELD, scaledX, scaledY, scaledRotate, 0);
			}
		}
	}

	public void drive(DriveType driveState, double x, double y, double rotation, double angle){
		drive.drive(driveState, x, y, rotation, angle);
	}
	

}
