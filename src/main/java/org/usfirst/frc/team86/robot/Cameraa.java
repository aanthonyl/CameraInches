package org.usfirst.frc.team86.robot;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team86.robot.MecanumDrive.DriveType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Cameraa {


	private VisionThread visionThread;

	private Object imgLock = new Object();

	private double centerX = 0.0;
	private double integralErr = 0;
	private double prevError = 0;
	private Mat outside;
	int count = 0;
	int count2 = 0;
	int count3 = 0;
	int count4 = 0;
	double totalcen = 0;
	int recs;
	CvSource outputStream = CameraServer.getInstance().putVideo("pipeline", 320, 240);
	public void init(){
		SmartDashboard.putBoolean("yeah", false);
//		visionThread = new VisionThread(IO.camera, new TyGod(), pipeline -> {
//			count2++;
//			SmartDashboard.putNumber("count2", count2);
//			SmartDashboard.putNumber("Contour Counts", pipeline.filterContoursOutput().size());
//			SmartDashboard.putNumber("Find Contour Counts", pipeline.findContoursOutput().size());
//	        if (!pipeline.filterContoursOutput().isEmpty()) {
//	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
//	            synchronized (imgLock) {
//	            	count++;
//	                centerX = r.x + (r.width / 2);
//	                SmartDashboard.putNumber("count", count);
//	            }
//	        }
//	    });
		
		visionThread = new VisionThread(IO.cam, new TyGod(), pipeline -> {
			SmartDashboard.putBoolean("yeah", true);
			SmartDashboard.putNumber("Contour Counts", pipeline.filterContoursOutput().size());
			SmartDashboard.putNumber("Find Contour Counts", pipeline.findContoursOutput().size());
			outside = pipeline.hslThresholdOutput();
	    	outputStream.putFrame(outside);
	    	count2++;
	        SmartDashboard.putNumber("count2", count2);
	        
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	        	
	        	
//	        	for(MatofPoints contours :: ){
//	        		
//	        	}
	        //	Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	          //  Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
	            synchronized (imgLock) {
	            	recs = pipeline.filterContoursOutput().size();
	            	if(recs == 2){
	            		totalcen = 0;
//	            		for(; recs >=1  ;recs--){
//	                    	Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(recs-1));
//	                    	centerX = 2*r.x + r.width - (320/2);
//	                    	totalcen += centerX;
//	                    	}
//	                   	totalcen /= 2; 
	                    	Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	                    	Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
	                    	SmartDashboard.putNumber("r1x", r1.x);
	                    	SmartDashboard.putNumber("r1wid", r1.width);
	                    	SmartDashboard.putNumber("r2x", r2.x);
	                        SmartDashboard.putNumber("r2wid", r2.width);
	                        totalcen = (r1.x + r1.width + r2.x)/2;
	                        
	            	}else{
	            		totalcen = -1;
	            	}
	            	
	            	SmartDashboard.putBoolean("outside", outside == null);
	                
	                count++;
	                SmartDashboard.putNumber("count", count);
	                
	               // centerY = 2*r.y + r.height - (IMG_HEIGHT/2);
	               // targetArea = r.area();
	            }
	        }
	    });
		
		
	    visionThread.start();
	    SmartDashboard.putNumber("Kp", .0013);
		SmartDashboard.putNumber("Ki", 0.0001);
		SmartDashboard.putNumber("Kd", 0);
		SmartDashboard.putNumber("Kf", .5);
	}

	public void update(){
		 SmartDashboard.putNumber("count4", count4);
		double totalcen;
		 synchronized (imgLock) {
		        totalcen = this.totalcen;
		        SmartDashboard.putNumber("Center cX", totalcen);
		        count3++;
	            SmartDashboard.putNumber("count3", count3);
		        
		    }						//IMG_WIDTH
		    double turn = totalcen - (320 / 2);
		    count4++;
		    SmartDashboard.putNumber("count4", count4);
	        SmartDashboard.putNumber("centertotal", totalcen);
		  //  SmartDashboard.putNumber("Center X", totalcen);
		    SmartDashboard.putNumber("result",   getPIDresult(160,totalcen));
		    SmartDashboard.putBoolean("camon",   JoystickIO.cam.isDown());
		  if(JoystickIO.cam.isDown()){
			  if(totalcen == -1){
				  
			  }else{
				  Robot.drive.drive(DriveType.ROTATE_PID, -getPIDresult(160,totalcen), JoystickIO.leftJoystick.getY(), 0, 0);
			  }
			  
			}
			  //  IO.left.set(ControlMode.PercentOutput, 0);
			   // IO.right.set(ControlMode.PercentOutput, 0);
							

		  }

	private double getPIDresult(double desired,double curr){
		double error = curr - desired;
		
		//SmartDashboard
		double kP = SmartDashboard.getNumber("Kp", .001);
		//double kP = 0.001;
		//double kI= 0;
		double kI = SmartDashboard.getNumber("Ki", 0);
		double kD = SmartDashboard.getNumber("Kd", 500);
		//double kD = 500;
		double kF = SmartDashboard.getNumber("Kf", .1);
		double deadband = 15;
		double maxI = 0;
		
		double result = 0;
		
		
		if (kI != 0) {
	        double potentialIGain = (integralErr + error) * kI;
	        if (potentialIGain < maxI) {
	          if (potentialIGain > -maxI) {
	            integralErr += error;
	          } else {
	            integralErr = -maxI; // -1 / kI
	          }
	        } else {
	          integralErr = maxI; // 1 / kI
	        }
	    } else {
	    	integralErr = 0;
	    }
		
		if (Math.abs(error) <= deadband) {
			error = 0;
		}else{
		
	    result = (kP * error) + (kI * integralErr) + (kD * (error - prevError));
	    if (result > 0) {
	    	result += kF;
	    } else {
	    	result -= kF;
	    }
		}
	   	prevError = error;
	   	
	    if (result > 1) {
	      result = 1;
	    } else if (result < -1) {
	      result = -1;
	    }
	    
//	    if(Math.abs(result) < Constants.MIN_ROTATE_SPEED && result > 0) {
//	    	if(result < 0) {
//	    		result = -Constants.MIN_ROTATE_SPEED;
//	    	} else {
//	    		result = Constants.MIN_ROTATE_SPEED;
//	    	}
//	    }
	    SmartDashboard.putNumber("error", error);
	    return result;
	}

	}
