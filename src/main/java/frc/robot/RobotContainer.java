// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.subsystems.Vision;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Intake m_intake = new Intake();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    Vision vision = new Vision();
    private SendableChooser<Command> autoChooser;

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // XboxController m_driverController = new
    // XboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
        //

        vision.setSimPoseSupplier(m_robotDrive::getSimPose);
        vision.setPoseEstimator(m_robotDrive.getPoseEstimator());
        vision.setChassisSpeedsSupplier(m_robotDrive::getSpeeds); // was getChassisSpeeds
        vision.setHeadingSupplier(m_robotDrive::getRotation);
        vision.setVisionMeasurementConsumer(m_robotDrive::addVisionMeasurement);
        vision.setPreciseVisionMeasurementConsumer(m_robotDrive::addPreciseVisionMeasurement);
        // Configure the button bindings
        configureButtonBindings();
        // Auto chooser

        
        // Registered named commands for Path Planner.
        NamedCommands.registerCommand("Shooter", m_shooter.runShooterCommand());
        // NamedCommands.registerCommand("Climb", m_shooter.runClimbCommand());

                try {
                        autoChooser = AutoBuilder.buildAutoChooser();
                } catch (RuntimeException e) {
                        // If AutoBuilder wasn't configured (e.g. PathPlanner GUI/settings unavailable),
                        // fall back to a simple chooser to avoid crashing the robot program.
                        e.printStackTrace();
                        autoChooser = new SendableChooser<>();
                        // Provide a safe default empty command
                        autoChooser.setDefaultOption("None", new InstantCommand(() -> {
                        }));
                }
                SmartDashboard.putData("Auto Chooser", autoChooser);


        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY() / 2, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX() / 2, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX() / 2, OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // new JoystickButton(m_driverController, Button.kR1.value)
        //         .whileTrue(new RunCommand(
        //                 () -> m_robotDrive.setX(),
        //                 m_robotDrive));

        // new JoystickButton(m_driverController,
        // CommandXboxController.Button.kStart.value)
        // .onTrue(new InstantCommand(
        // () -> m_robotDrive.zeroHeading(),
        // m_robotDrive));

        m_driverController.start().onTrue(new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));

        // This is the intake button (L1) so you can test if it works or change it.
        // Also this should work on the driver controller and we can maybe change it
        // later to the operator controller if we have one.
        // new JoystickButton(m_driverController, Button.kL1.value)
        // new JoystickButton(m_driverController,
        // CommandXboxController.Button.kLeftBumper.value)
        // .whileTrue(new RunCommand(
        // () -> m_intake.runIntakeCommand(),
        // m_intake));

        m_driverController.leftBumper().toggleOnTrue(new RunCommand(
                () -> m_intake.runIntakeCommand(),
                m_intake));

        // new JoystickButton(m_driverController,
        // CommandXboxController.Button.kRightBumper.value) //change the left trigger
        // .whileTrue(new RunCommand(
        // () -> m_intake.runOuttakeCommand(),
        // m_intake));

        m_driverController.rightBumper().toggleOnTrue(new RunCommand(
                () -> m_intake.runOuttakeCommand(),
                m_intake));

        // new JoystickButton(m_driverController,
        // XboxController.Button.kLeftTrigger.value)
        // .whileTrue(m_intake.runLowerCommand());
        // new JoystickButton(m_driverController,
        // XboxController.Button.kRightTrigger.value)
        // .whileTrue(m_intake.runRaiseCommand());
        m_driverController
                .leftTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(m_intake.runLowerCommand());
        m_driverController
                .rightTrigger(OIConstants.kTriggerButtonThreshold)
                .whileTrue(m_intake.runRaiseCommand());

        // new JoystickButton(m_driverController, CommandXboxController.Button.kY.value)
        //         .toggleOnTrue(m_shooter.runShooterCommand());
        
        m_driverController.y().toggleOnTrue(m_shooter.runShooterCommand());
        // new JoystickButton(m_driverController,
        // XboxController.Button.kY.value).toggleOnTrue(m_shooter.runShooterCommand().alongWith(m_intake.runIntakeCommand()));
        // new JoystickButton(m_driverController, CommandXboxController.Button.kB.value)
        //         .whileTrue(m_climber.runClimbCommand()); // change button later (if needed)

        m_driverController.pov(0).whileTrue(m_climber.runClimbCommand());

        // new JoystickButton(m_driverController, CommandXboxController.Button.kA.value)
        //         .whileTrue(m_climber.runDescendCommand()); // change button later (if needed)
        
        m_driverController.pov(180).whileTrue(m_climber.runDescendCommand());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // m_robotDrive::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // m_robotDrive::setModuleStates,
        // m_robotDrive);

        // // Reset odometry to the starting pose of the trajecto

        // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false));
        return autoChooser.getSelected(); // use this line for Path Planner Selector

    }
}
