/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.robot.subsystems.Drive;
import frc.team166.robot.subsystems.Manipulator;
import frc.team166.robot.subsystems.LED;
import frc.team166.robot.subsystems.Lift;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final LED led = new LED();
    public static final Drive drive = new Drive();
    public static final Manipulator manipulator = new Manipulator();
    public static final Lift lift = new Lift();
    public static OI m_oi;
    public static final Compressor compressy = new Compressor(1);

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    Command driveCommand;
    SendableChooser<Command> driveChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        m_oi = new OI();
        // m_chooser.addDefault("Default Auto", drive.DriveTime(3, 0.6));
        m_chooser.addObject("Mid Auto", MidAuto());
        m_chooser.addDefault("Cross Line And Drop Cube", CrossLineAndDropCube());

        driveChooser.addDefault("Joystick Drive", drive.JoystickArcadeTwoStick());
        driveChooser.addDefault("Xbox Drive", drive.XboxArcade());

        SmartDashboard.putData("Auto mode", m_chooser);
        SmartDashboard.putData("Turn 90", drive.TurnByDegrees(90));
        SmartDashboard.putData("Turn -90", drive.TurnByDegrees(-90));
        CameraServer.getInstance().startAutomaticCapture();
        SmartDashboard.putData("Drive Type", driveChooser);
    }

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit() {
        drive.reset();
        lift.reset();
        manipulator.reset();

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_chooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
         * switch(autoSelected) { case "My Auto": autonomousCommand = new
         * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
         * ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {

            m_autonomousCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //
        // create timer that goes up evry seond.
        driveCommand = driveChooser.getSelected();
        if (driveCommand != null) {
            driveCommand.start();
        }

        led.teleopInit();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public Command CrossLineAndDropCube() {
        return new CommandChain("Cross Line And Drop Cube")
                .then(drive.DriveTime(3.2, 0.6), lift.DeployManipulatorForSwitch()).then(manipulator.CubeEject());
    }

    public Command MidAuto() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        double firstDegrees = 0.0;
        double secondDegrees = 0.0;
        if (gameData.length() > 0) {
            if (gameData.charAt(0) == 'R') {
                // "R" is for RIGHT NOT RED
                firstDegrees = 70;
                secondDegrees = 250;
                // turning right
                System.out.println("Right");

            } else {
                firstDegrees = 250;
                secondDegrees = 70;
                // turning left
                System.out.println("Left");
            }
        }
        Command cmdMidAuto = new CommandChain("Mid Auto").then(
                new CommandChain("Drive To Switch").then(drive.DriveTime(.4, .6))
                        .then(drive.TurnByDegrees(firstDegrees)).then(drive.DriveTime(1.75, .6))
                        .then(drive.TurnByDegrees(secondDegrees)).then(drive.DriveTime(3.9, .6)),
                lift.DeployManipulatorForSwitch()).then(manipulator.ManipulatorDischarge());
        return cmdMidAuto;

    }

    public Command RightSwitch(double degrees) {
        return new CommandChain("Right Switch").then(new CommandChain("SWITCHYYYYYYY").then(drive.DriveTime(3.2, 0.6))
                .then(drive.TurnByDegrees(degrees)).then(drive.DriveTime(1, .6)), lift.DeployManipulatorForSwitch())
                .then(manipulator.CubeEject());
    }
}
