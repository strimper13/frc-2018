/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team166.robot.commands.SubsystemCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Cart extends Subsystem {
	Victor motor = new Victor(1);
	DigitalInput nearSensor = new DigitalInput(8);
	DigitalInput farSensor = new DigitalInput(9);

	public Command moveAway() {
		return new SubsystemCommand("Move away", this) {
			@Override
			protected void initialize() {
				setSpeed(-0.3);
			}

			@Override
			protected boolean isFinished() {
				return isFarPressed();
			}

			@Override
			protected void end() {
				setSpeed(0);
			}
		};
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public Command moveBack() {
		return new SubsystemCommand("Move back", this) {
			@Override
			protected void initialize() {
				setSpeed(0.3);
			}

			@Override
			protected boolean isFinished() {
				return isNearPressed();
			}

			@Override
			protected void end() {
				setSpeed(0);
			}
		};
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	private boolean isNearPressed() {
		return !nearSensor.get();
	}

	private boolean isFarPressed() {
		return !farSensor.get();
	}

	private void setSpeed(double speed) {
		//set motor speed
		motor.setSpeed(speed);
	}
}