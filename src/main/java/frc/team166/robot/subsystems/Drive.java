/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team166.robot.Robot;
import frc.team166.robot.RobotMap;
import frc.team166.robot.RobotMap.PreferenceStrings;
import frc.team166.chopshoplib.commands.CommandChain;
import frc.team166.chopshoplib.commands.SubsystemCommand;
import frc.team166.chopshoplib.sensors.Lidar;
import edu.wpi.first.wpilibj.I2C.Port;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {

    // declare lidar
    Lidar frontLidar = new Lidar(Port.kOnboard, 0x10);
    // defines the gyro
    AnalogGyro gyro = new AnalogGyro(RobotMap.AnalogInputs.tempestgyro);
    // defines the encoders (left and right)
    Encoder leftEncoder = new Encoder(RobotMap.DigitalInputs.LEFT_ENCODER_A, RobotMap.DigitalInputs.LEFT_ENCODER_B);
    Encoder rightEncoder = new Encoder(RobotMap.DigitalInputs.RIGHT_ENCODER_A, RobotMap.DigitalInputs.RIGHT_ENCODER_B);

    // defines the left motors as motors and combines the left motors into one motor
    WPI_TalonSRX m_rearleft = new WPI_TalonSRX(RobotMap.CAN.BACK_LEFT);
    WPI_TalonSRX m_frontleft = new WPI_TalonSRX(RobotMap.CAN.FRONT_LEFT);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontleft, m_rearleft);
    // defines the right motors as motors and combines the left motors into one
    // motor
    WPI_TalonSRX m_rearright = new WPI_TalonSRX(RobotMap.CAN.BACK_RIGHT);
    WPI_TalonSRX m_frontright = new WPI_TalonSRX(RobotMap.CAN.FRONT_RIGHT);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontright, m_rearright);

    /**
     * defines the left and right motors defined above into a differential drive
     * that can be used for arcade and tank drive, amung other things
     */
    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

    // 3 Waypoints
    // Waypoint[] points = new Waypoint[] {

    // new Waypoint(-4, -1, Pathfinder.d2r(-45)),
    // Waypoint @ x=-4, y=-1, exit angle=-45 degrees

    // new Waypoint(-2, -2, 0),
    // Waypoint @ x=-2, y=-2, exit angle=0 radians

    // new Waypoint(0, 0, 0)
    // Waypoint @ x=0, y=0, exit angle=0 radians

    // };

    // Create the Trajectory Configuration
    //
    // Arguments:
    // Fit Method: HERMITE_CUBIC or HERMITE_QUINTIC
    // Sample Count: SAMPLES_HIGH (100 000)
    // SAMPLES_LOW (10 000)
    // SAMPLES_FAST (1 000)
    // Time Step: 0.05 Seconds
    // Max Velocity: 1.7 m/s
    // Max Acceleration: 2.0 m/s/s
    // Max Jerk: 60.0 m/s/s/s
    final double maxVelocity = 1.7;
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
            0.05, maxVelocity, 2.0, 60.0);
    Waypoint[] points = new Waypoint[] { new Waypoint(-4, -1, Pathfinder.d2r(-45)), new Waypoint(-2, -2, 0),
            new Waypoint(0, 0, 0)

    };

    // Generate the trajectory
    Trajectory trajectory = Pathfinder.generate(points, config);

    // Wheelbase Width = 0.5m
    // Tempest Frame is 29 inches across = .737Meters
    // Tempest Axel spacing is 25 inches = .635Meters
    TankModifier modifier = new TankModifier(trajectory).modify(0.635);

    // Do something with the new Trajectories...

    EncoderFollower leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower rightFollower = new EncoderFollower(modifier.getRightTrajectory());

    // defines values that will be used in the PIDController (In order of where they
    // will fall in the Controller)
    final static double kP = 0.016;
    final static double kI = 0.00007;
    final static double kD = 0;
    final static double kF = 0;

    // defines a new double that is going to be used in the line that defines the
    // drive type
    double angleCorrection;

    // PIDController loop used to find the power of the motors needed to keep the
    // angle of the gyro at 0
    PIDController drivePidController = new PIDController(kP, kI, kD, kF, gyro, (double value) -> {
        // this assigns the output to the angle (double) defined later in the code)
        angleCorrection = value;
    });

    final static double AUTOMATIC_ROBOT_FORWARD_SPEED = .2;
    final static double ABSOLUTE_TOLERANCE_ANGLE = 3;

    // this makes children that control the gyro, drive motors, and
    // PIDController loop.
    public Drive() {

        // SmartDashboard.putData("XBox", XboxArcade());
        // SmartDashboard.putData("Turn -45", TurnByDegrees(-45));
        // SmartDashboard.putData("Turn 45", TurnByDegrees(45));
        // SmartDashboard.putData("Drive 2s", DriveTime(2, .6));
        // SmartDashboard.putData("Drive Box", DriveBox());

        addChild(gyro);
        addChild(m_drive);
        addChild(drivePidController);
        addChild("Front LiDAR", frontLidar);
        gyro.setSensitivity(0.0125 / 5.45 * (.825)); // We don't know what the
        // sensitivity of this gyro actually
        // is...
        // Update: We think this is just the default
        drivePidController.setOutputRange(-0.6, 0.6);
        drivePidController.setPercentTolerance(0.90);

        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.AUTOMATIC_ROBOT_FORWARD_SPEED,
                AUTOMATIC_ROBOT_FORWARD_SPEED);
        PreferenceStrings.setDefaultDouble(RobotMap.PreferenceStrings.ABSOLUTE_TOLERANCE_ANGLE,
                ABSOLUTE_TOLERANCE_ANGLE);

        drivePidController.disable();
        drivePidController.setInputRange(0, 360);
        drivePidController.setContinuous();
        drivePidController.setAbsoluteTolerance(Preferences.getInstance()
                .getDouble(RobotMap.PreferenceStrings.ABSOLUTE_TOLERANCE_ANGLE, ABSOLUTE_TOLERANCE_ANGLE));

        // Encoder Position is the current, cumulative position of your encoder. If
        // you're using an SRX, this will be the
        // 'getEncPosition' function.
        // 1000 is the amount of encoder ticks per full revolution
        // Wheel Diameter is the diameter of your wheels (or pulley for a track system)
        // in meters
        leftFollower.configureEncoder(leftEncoder.get(), 1000, .1016);
        rightFollower.configureEncoder(rightEncoder.get(), 1000, .1016);
        // The first argument is the proportional gain. Usually this will be quite high
        // The second argument is the integral gain. This is unused for motion profiling
        // The third argument is the derivative gain. Tweak this if you are unhappy with
        // the tracking of the trajectory
        // The fourth argument is the velocity ratio. This is 1 over the maximum
        // velocity you provided in the
        // trajectory configuration (it translates m/s to a -1 to 1 scale that your
        // motors can read)
        // The fifth argument is your acceleration gain. Tweak this if you want to get
        // to a higher or lower speed quicker
        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / maxVelocity, 0);
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / maxVelocity, 0);
    }

    // the default command for this code is supposed to rotate the robot so that
    // it's gyro value is 0
    public void initDefaultCommand() {
        setDefaultCommand(JoystickArcadeTwoStick());

    }

    public void reset() {
        m_drive.stopMotor();
    }

    public Command followPath() {
        return new SubsystemCommand("followPath", this) {

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                double leftOutput = leftFollower.calculate(leftEncoder.get());
                double rightOutput = rightFollower.calculate(rightEncoder.get());

                double gyro_heading = gyro.getAngle(); // Assuming the gyro is giving a value in degrees
                double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

                double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
                double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

                m_left.set(leftOutput + turn);
                m_right.set(rightOutput - turn);

            }
        };
    }

    public Command XboxArcade() {
        return new SubsystemCommand("XBoxArcade", this) {
            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(
                        (Robot.m_oi.xBoxDrive.getTriggerAxis(Hand.kRight)
                                - Robot.m_oi.xBoxDrive.getTriggerAxis(Hand.kLeft)),
                        Robot.m_oi.xBoxDrive.getX(Hand.kLeft));

            }

        };
    }

    public Command JoystickArcadeTwoStick() {
        return new SubsystemCommand("joystick Arcade with two sticks", this) {
            @Override
            protected void initialize() {
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(-Robot.m_oi.leftDriveStick.getY(), Robot.m_oi.rightDriveStick.getX());

            }

            @Override
            protected boolean isFinished() {
                return false;
            }

        };
    };

    public Command DriveStraight() {
        return new SubsystemCommand("Drive Straight", this) {
            @Override
            protected void initialize() {
                drivePidController.reset();
                drivePidController.setSetpoint(gyro.getAngle());
                drivePidController.enable();
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kRight)
                        - Robot.m_oi.xBoxTempest.getTriggerAxis(Hand.kLeft), angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return false;

            }

            @Override
            protected void end() {
                drivePidController.disable();
            }

            @Override
            protected void interrupted() {
                drivePidController.disable();
            }
        };
    }

    public Command DrivetoProximity(double inches) {
        return new SubsystemCommand("Drive Distance", this) {

            // double realDistanceInches = frontLidar.getDistance(true);

            @Override
            protected void initialize() {
                drivePidController.setSetpoint(gyro.getAngle());
                drivePidController.reset();
                drivePidController.enable();
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(Preferences.getInstance()
                        .getDouble(RobotMap.PreferenceStrings.ABSOLUTE_TOLERANCE_ANGLE, ABSOLUTE_TOLERANCE_ANGLE),
                        angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                if (frontLidar.getDistance(true) <= inches) {
                    return true;
                } else {
                    return false;
                }

            }

            @Override
            protected void end() {
                drivePidController.disable();
            }

            @Override
            protected void interrupted() {
                drivePidController.disable();
            }
        };
    }

    public Command TurnByDegrees(double degrees, double forwardSpeed) {
        return new SubsystemCommand("Turn " + degrees, this) {
            @Override
            protected void initialize() {
                gyro.reset();
                drivePidController.reset();
                drivePidController.setAbsoluteTolerance(Preferences.getInstance()
                        .getDouble(RobotMap.PreferenceStrings.ABSOLUTE_TOLERANCE_ANGLE, ABSOLUTE_TOLERANCE_ANGLE));
                drivePidController.setSetpoint(degrees);
                drivePidController.enable();
            }

            @Override
            protected void execute() {
                SmartDashboard.putNumber("Drive Angle", angleCorrection);
                m_drive.arcadeDrive(forwardSpeed, angleCorrection);

            }

            @Override
            protected boolean isFinished() {
                return drivePidController.onTarget();
            }

            @Override
            protected void end() {
                drivePidController.disable();
            }

            @Override
            protected void interrupted() {
                drivePidController.disable();
            }
        };
    }

    public Command TurnByDegrees(double degrees) {
        return TurnByDegrees(degrees, 0);
    }

    public Command DriveTime(double seconds, double speed) {
        return new SubsystemCommand("Drive Time", this) {
            @Override
            protected void initialize() {
                drivePidController.reset();
                // drivePidController.setSetpoint(gyro.getAngle());
                // drivePidController.enable();
                setTimeout(seconds);
            }

            @Override
            protected void execute() {
                m_drive.arcadeDrive(speed, angleCorrection);
            }

            @Override
            protected boolean isFinished() {
                return isTimedOut();
            }

            @Override
            protected void end() {
                drivePidController.disable();
                m_drive.stopMotor();
            }

            @Override
            protected void interrupted() {
                drivePidController.disable();
            }
        };
    }

    public Command DriveBox() {
        return new CommandChain("Box Drive").then(DriveTime(1, .8)).then(TurnByDegrees(90)).then(DriveTime(.5, .8))
                .then(TurnByDegrees(90)).then(DriveTime(1, .8)).then(TurnByDegrees(90)).then(DriveTime(.5, .8))
                .then(TurnByDegrees(90));

    }

}
