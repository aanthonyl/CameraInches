package org.usfirst.frc.team86.robot;

import org.usfirst.frc.team86.robot.JoystickIO;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MecanumDrive {

	// Objects
	private RobotDrive drive;
	private NavX gyro;
	
	// PID Constants -- TODO read from file
	private double kP;
	private double kI;
	private double kD;
	private double kF;
	private double maxI;
	
	// Other variables
	private double integral;
	private double prevError;
	
	public MecanumDrive(SpeedController leftFrontMotor,
			SpeedController leftRearMotor, 
			SpeedController rightFrontMotor, 
			SpeedController rightRearMotor, 
			NavX gyro) {
		this.drive = new RobotDrive(leftFrontMotor, leftRearMotor,
				rightFrontMotor, rightRearMotor);
		this.gyro = gyro;
	}
	
	public void init() {
		// TODO read kP, kI, kD, kF from file
		this.prevError = 0.0;
		this.integral = 0.0;
	}
	
	public void drive(DriveType driveState, double x, double y, double rotation, double angle) {
		double gyroAngle = gyro.getAngle();
		switch(driveState) {
		case STICK_FIELD:
			drive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
			break;	
		case ROTATE_PID:
			gyroAngle = gyro.getNormalizedAngle();
			angle = ((angle % 360) + 360) % 360;
			double error = angle - gyroAngle;
			if (Math.abs(error) > 180) { // if going around the other way is closer
				if (error > 0) { // if positive
					error = error - 360;
				} else { // if negative
				    error =  error + 360;
				}
			}
			
			if (kI != 0) {
	            double potentialIGain = (integral + error) * kI;
	            if (potentialIGain < maxI) {
	              if (potentialIGain > -maxI) {
	                integral += error;
	              } else {
	                integral = -maxI; // -1 / kI
	              }
	            } else {
	              integral = maxI; // 1 / kI
	            }
	        } else {
	        	integral = 0;
	        }
			
			if (Math.abs(error) < 3.0) {
				error = 0;
			}
			
	        double result = (kP * error) + (kI * integral) + (kD * (error - prevError));
	        if (result > 0) {
	        	result += kF;
	        } else {
	        	result -= kF;
	        }
	       	prevError = error;
	       	
	        if (result > 1) {
	          result = 1;
	        } else if (result < -1) {
	          result = -1;
	        }
	        	        
			drive.mecanumDrive_Cartesian(x, y, result, gyroAngle);
			break;
		}
	}
	
	public void setPIDValues(double kP, double kI, double kD, double kF, double maxI) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.maxI = maxI;
	}
	
	public enum DriveType {
		STICK_FIELD,
		ROTATE_PID,
	}

}
