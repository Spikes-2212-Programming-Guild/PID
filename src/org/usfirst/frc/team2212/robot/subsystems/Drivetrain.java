package org.usfirst.frc.team2212.robot.subsystems;

import org.usfirst.frc.team2212.robot.RobotMap;

import com.ni.vision.NIVision.GeometricSetupDataItem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * 
 *
 */
public class Drivetrain extends PIDSubsystem {
	private static double i;
	private static double d;
	private static double p;
	private static SpeedController leftMotor = RobotMap.leftDriveMotor;
	private static SpeedController rightMotor = RobotMap.rightDriveMotor;
	private Encoder rightPort = RobotMap.rightDrivePort;
	private Encoder leftPort = RobotMap.leftDrivePort;


	// Initialize your subsystem here
	public Drivetrain() {
    super( p,  i, d);
    super.setSetpoint(1);
    super.enable();
    rightPort.setDistancePerPulse(0.01);
    leftPort.setDistancePerPulse(0.01);
		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable() - Enables the PID controller.
	}
	public static void drive(double left, double right){
		leftMotor.set(left);
		rightMotor.set(right);
		
	}
		
	

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	protected double returnPIDInput() {
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		return rightPort.getDistance()+leftPort.getDistance()/2;
	}

	protected void usePIDOutput(double output) {
		// Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
		
	}
}
