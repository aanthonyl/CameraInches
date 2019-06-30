/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team86.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
	public static Drive drive;
	Cameraa came;
	int init = 0;
	@Override
	public void robotInit() {
		drive = new Drive(IO.leftFrontMotor,IO.leftRearMotor,IO.rightFrontMotor,IO.rightRearMotor,IO.navX);
		came = new Cameraa();
		
		IO.cam.setResolution(320, 240);
		IO.cam.setBrightness(10);	
		IO.cam.setExposureManual(0);
		came.init();
	}

	
	@Override
	public void autonomousInit() {
		
	}

	
	@Override
	public void autonomousPeriodic() {
		
		}
	
	@Override
	public void teleopInit(){
		IO.navX.reset();
		drive.init();
		SmartDashboard.putNumber("Init", init++);
		
	}
	@Override
	public void teleopPeriodic() {
//		IO.leftFrontMotor.set( -.3 * IO.leftJoystick.getY());
//		IO.rightFrontMotor.set( -.3 * IO.rightJoystick.getY());
//		IO.leftRearMotor.set( -.3 * IO.leftJoystick.getY());
//		IO.rightRearMotor.set( -.3 * IO.rightJoystick.getY());
		drive.update();
		JoystickIO.update();
		came.update();
	}


	@Override
	public void testPeriodic() {
	}
}
