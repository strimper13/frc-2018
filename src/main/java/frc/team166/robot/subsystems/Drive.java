/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.commands.SubsystemCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
	WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(RobotMap.CAN.FrontLeft);
	WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(RobotMap.CAN.BackLeft);
	SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

	WPI_TalonSRX m_frontRight = new WPI_TalonSRX(RobotMap.CAN.FrontRight);
	WPI_TalonSRX m_rearRight = new WPI_TalonSRX(RobotMap.CAN.BackRight);
	SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		setDefaultCommand(new SubsystemCommand(this) {

			@Override
			protected boolean isFinished() {
				return false;
			}

			@Override
			protected void execute() {
				m_drive.arcadeDrive(-Robot.m_oi.JoystickDrive.getY(), -Robot.m_oi.JoystickDrive2.getX());
			}
		});
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
