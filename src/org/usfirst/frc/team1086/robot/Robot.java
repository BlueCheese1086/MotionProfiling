/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1086.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
public class Robot extends TimedRobot {
	Joystick leftStick, rightStick, auxStick;
	
	Waypoint[] points;
	EncoderFollower left, right;
	TalonSRX talonFrontLeft, talonFrontRight, talonBackLeft, talonBackRight;
	AHRS vmx;
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
		points = new Waypoint[] {
				new Waypoint(0, 0, 0),
				new Waypoint(1, 0, 0)
				
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
			drive(-leftStick.getY(), rightStick.getX(), leftStick.getX());
		else drive(0, 0, 0);
	
		if(auxStick.getRawButtonPressed(3)) {
			double dt = 0.05;
			double maxVelocity = 1.7;
			double maxAcceleration = 2.0;
			double maxJerk = 60.0;
			Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_FAST, dt, maxVelocity, maxAcceleration, maxJerk);
			Trajectory trajectory = Pathfinder.generate(points, config);
			
			double wheelbaseWidth = 0.43; //Distance between the two sides of DT, in meters.
			TankModifier modifier = new TankModifier(trajectory).modify(wheelbaseWidth);
			
			left = new EncoderFollower(modifier.getLeftTrajectory());
			right = new EncoderFollower(modifier.getRightTrajectory());
			
			//configureEncoder(encoderPosition, encoderTickPerRevolution, wheelDiameter)
			left.configureEncoder((int)(talonFrontLeft.getSelectedSensorPosition(0) * 1.07), 4096, .1524);
			right.configureEncoder((int)(talonFrontRight.getSelectedSensorPosition(0) * 1.07), 4096, .1524);
		
			double p = 0.1;
			double i = 0;
			double d = 0;
			double velocityInverse = 1/maxVelocity;
			double accelGain = 0;
			left.configurePIDVA(p, i, d, velocityInverse, accelGain);
			right.configurePIDVA(p, i, d, velocityInverse, accelGain);
		}
		if(auxStick.getRawButton(3)) { 
			double leftSpeed = left.calculate(talonFrontLeft.getSelectedSensorPosition(0));
			double rightSpeed = right.calculate(talonFrontRight.getSelectedSensorPosition(0));
			double gyroHeading = vmx.getAngle();
			double desiredHeading = Pathfinder.r2d(left.getHeading());
			SmartDashboard.putNumber("MP Path x", left.getSegment().x);
			SmartDashboard.putNumber("MP Path position", left.getSegment().position);
			double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
			double turn = 0.04 * (-1.0 / 80.0) * angleDifference;
			SmartDashboard.putNumber("MP Left Speed", leftSpeed);
			SmartDashboard.putNumber("MP Right Speed", rightSpeed);
			SmartDashboard.putNumber("MP Turn", turn);
			
			talonFrontLeft.set(ControlMode.PercentOutput, leftSpeed + turn);
			talonBackLeft.set(ControlMode.PercentOutput, leftSpeed + turn);
			talonFrontRight.set(ControlMode.PercentOutput, rightSpeed - turn);
			talonBackRight.set(ControlMode.PercentOutput, rightSpeed - turn); 
		}
	}
	
	public double getLeftDistance() {
		double d = talonFrontLeft.getSelectedSensorPosition(0) / 4096.0 * 6 * Math.PI;
		return d + bestFitLine(d);
	}
	
	public double getRightDistance() {
		double d = talonFrontRight.getSelectedSensorPosition(0) / 4096.0 * 6 * Math.PI;
		return d + bestFitLine(d);
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
}
