/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1086.robot;

import java.io.File;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends TimedRobot {
	Joystick leftStick, rightStick, auxStick;
	
	Waypoint[] points;
	EncoderFollower left; 
	EncoderFollower right;
	TalonSRX talonFrontLeft, talonFrontRight, talonBackLeft, talonBackRight;
	AHRS vmx;
	
	PIDController turnController;
	double turnOutput = 0;
	
	double angleDifference = 0;
	
	@Override
	public void robotInit() {
		talonFrontLeft = new TalonSRX(1);
		talonFrontRight = new TalonSRX(2);
		talonBackLeft = new TalonSRX(3);
		talonBackRight = new TalonSRX(4);
		
		talonFrontRight.setInverted(true);
		talonBackRight.setInverted(true);
		
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		auxStick = new Joystick(5);
		
		try {
			vmx = new AHRS(Port.kUSB1);
			//vmx = new AHRS(SPI.Port.kMXP);
		} catch(Exception e) {
			System.out.println("VMX is null");
		}
		
		//The first int is pidIdx - 0 for primary closed loop, 1 for cascaded closed loop
		//The second int is timeoutMs - >0 will timeout error if it times out, 0 means dont check at all
		talonFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		talonFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	
		turnController = new PIDController(.07, 0.001, 0.06, new PIDSource() {
			@Override public void setPIDSourceType(PIDSourceType pidSource) {}

			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}

			@Override
			public double pidGet() {
				return angleDifference;
			}
			
		}, v -> turnOutput = v);
	}
	
	public void drive(double drive, double turn, double strafe) {
		talonFrontLeft.set(ControlMode.PercentOutput, drive + turn + strafe);
		talonFrontRight.set(ControlMode.PercentOutput, drive - turn - strafe);
		talonBackLeft.set(ControlMode.PercentOutput, drive + turn - strafe);
		talonBackRight.set(ControlMode.PercentOutput, drive - turn + strafe);
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		/*points = new Waypoint[] {
				new Waypoint(0, 0, 0),
				new Waypoint(.6, 0, Pathfinder.d2r(-30)),
				new Waypoint(.6, -1, Pathfinder.d2r(-150)),
				new Waypoint(-.4, -1, Pathfinder.d2r(-150)),
				new Waypoint(-.4, -2, Pathfinder.d2r(-30)),
				new Waypoint(.6, -2, Pathfinder.d2r(-30)),
				new Waypoint(.6, -3, Pathfinder.d2r(-150)),
				new Waypoint(-.68, -3, Pathfinder.d2r(-180))
		}; */
		points = new Waypoint[] {
				new Waypoint(0, 0, Pathfinder.d2r(0)),
				new Waypoint(.8, -0.5, Pathfinder.d2r(-90)),
				new Waypoint(0, -1, Pathfinder.d2r(-180)),
				new Waypoint(-.6, -1.5, Pathfinder.d2r(-90)),
				new Waypoint(0, -2, Pathfinder.d2r(0)),
				new Waypoint(.8, -2.5, Pathfinder.d2r(-90)),
                new Waypoint(0, -3.2, Pathfinder.d2r(180)),
                new Waypoint(-0.6, -3.2, Pathfinder.d2r(180))
		};
		
		talonFrontLeft.setSelectedSensorPosition(0, 0, 0);
		talonFrontRight.setSelectedSensorPosition(0, 0, 0);
		vmx.reset();
	}
	
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("Left encoder", getLeftDistance());
		SmartDashboard.putNumber("Right encoder", getRightDistance());
		SmartDashboard.putNumber("Distance Travelled", getEncDistance());
		SmartDashboard.putNumber("VMX Angle", vmx.getYaw());
		
		if(leftStick.getRawButton(1))
			drive(-leftStick.getY() * Math.abs(leftStick.getY()), rightStick.getX() * Math.abs(rightStick.getX()), leftStick.getX() * Math.abs(leftStick.getX()));
		else drive(0, 0, 0);

		if(auxStick.getRawButton(2)) {
			drive(0, .5, 0);
		}
		
		if(auxStick.getRawButtonPressed(3)) {
			double dt = 0.02;
			double maxVelocity = 1.086;
            double maxAcceleration = .6;
            double maxJerk = 60.0;
            Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                    Trajectory.Config.SAMPLES_FAST, dt, maxVelocity, maxAcceleration, maxJerk);
            Trajectory trajectory = Pathfinder.generate(points, config);
            double wheelbaseWidth = 0.584; //Distance between the two sides of DT, in meters.
			TankModifier modifier = new TankModifier(trajectory).modify(wheelbaseWidth);
			
			left = new EncoderFollower(modifier.getLeftTrajectory(), "/home/lvuser/left.csv");
			right = new EncoderFollower(modifier.getRightTrajectory(), "/home/lvuser/right.csv");
			
			//configureEncoder(encoderPosition, encoderTickPerRevolution, wheelDiameter)
			left.configureEncoder(talonFrontLeft.getSelectedSensorPosition(0), 4096, .1524 / 1.07);
			right.configureEncoder(talonFrontRight.getSelectedSensorPosition(0), 4096, .1524 / 1.07);

			double p = 1.7;
			double i = 0;
			double d = 0.45;
			double velocityInverse = 1 / maxVelocity;
			double accelGain = 0;
			left.configurePIDVA(p, i, d, velocityInverse, accelGain);
			right.configurePIDVA(p, i, d, velocityInverse, accelGain);
			
			turnController.setSetpoint(0);
			turnController.setInputRange(-180, 180);
			turnController.setOutputRange(-2, 2);
			turnController.setContinuous(true);
			turnController.enable();

			vmx.reset();
			vmx.getAngle();
		}
		else if(auxStick.getRawButton(3)) { 
			if(!left.isFinished() && !right.isFinished()) {
				double leftSpeed = left.calculate(talonFrontLeft.getSelectedSensorPosition(0), talonFrontLeft.getSelectedSensorVelocity(0) / 4096.0 * .1524 * Math.PI);
				double rightSpeed = right.calculate(talonFrontRight.getSelectedSensorPosition(0), talonFrontRight.getSelectedSensorVelocity(0) / 4096.0 * .1524 * Math.PI);
				double gyroHeading = -vmx.getAngle();
				double desiredHeading = Pathfinder.r2d(left.getHeading());
				angleDifference = normalizeAngle(desiredHeading - gyroHeading);
				SmartDashboard.putNumber("Angle Difference", angleDifference);
				SmartDashboard.putNumber("Target Angle", desiredHeading);
				SmartDashboard.putNumber("Current Angle", gyroHeading);
				//double turn = .7 * (-1.0 / 80.0) * angleDifference;
				double turn = turnController.get();
				SmartDashboard.putNumber("MP Left Speed", leftSpeed);
				SmartDashboard.putNumber("MP Right Speed", rightSpeed);
				SmartDashboard.putNumber("MP Turn", turn);
				
				
				//System.out.println("Left speed: " + leftSpeed);
				//System.out.println("Right speed: " + rightSpeed);
				//System.out.println("Turn: " + turn);
				//System.out.println("MP Target Turn " +  Pathfinder.r2d(left.getHeading()));
				
				talonFrontLeft.set(ControlMode.PercentOutput, leftSpeed + turn);
				talonBackLeft.set(ControlMode.PercentOutput, leftSpeed + turn);
				talonFrontRight.set(ControlMode.PercentOutput, rightSpeed - turn);
				talonBackRight.set(ControlMode.PercentOutput, rightSpeed - turn); 
			}
			else {
				talonFrontLeft.set(ControlMode.PercentOutput, 0);
				talonBackLeft.set(ControlMode.PercentOutput, 0);
				talonFrontRight.set(ControlMode.PercentOutput, 0);
				talonBackRight.set(ControlMode.PercentOutput, 0); 
				turnController.reset();
				turnController.disable();
			}
		}
		
		if(auxStick.getRawButtonPressed(4)) {
			
		}
	}
	
	public double getLeftDistance() {
		double d = talonFrontLeft.getSelectedSensorPosition(0) / 4096.0 * 6 * Math.PI;
		//return d + bestFitLine(d);
		return d;
	}
	
	public double getRightDistance() {
		double d = talonFrontRight.getSelectedSensorPosition(0) / 4096.0 * 6 * Math.PI;
		//return d + bestFitLine(d);
		return d;
	}
	
	public double bestFitLine(double dist) {
		return dist * 0.0618 + .148;
	}
	
	public double getEncDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	@Override
	public void testPeriodic() {
	}

	public double normalizeAngle(double angle){
		double ang = (angle % 360 + 360) % 360;
		return ang > 180 ? ang - 360 : ang;
	}
}
