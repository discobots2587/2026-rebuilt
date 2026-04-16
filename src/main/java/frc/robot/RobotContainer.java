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
import frc.robot.Constants.AutoConstants;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.ShooterWithParametersCommand;
import frc.robot.commands.BlueLeftNeutralAuto;
import frc.robot.commands.BlueRightNeutralAuto;
import frc.robot.commands.BlueLefNeutralClimbAuto;
import frc.robot.commands.BLOAAuto;
import frc.robot.commands.BROAAuto;
import frc.robot.commands.BlueRoboAutoLAuto;
import frc.robot.commands.BlueRoboAutoRAuto;
import frc.robot.commands.TeleopShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
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
    private final SpindexerSubsystem m_spindexer = new SpindexerSubsystem(m_intake);
    private final HoodSubsystem m_hood = new HoodSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();

    Vision vision = new Vision();
    private SendableChooser<Command> autoChooser;

    //test align
    private final AlignToHub m_AlignToHub = new AlignToHub(m_robotDrive, m_shooter);

    // The controllers
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        // Vision Configurations
        vision.setSimPoseSupplier(m_robotDrive::getSimPose);
        vision.setPoseEstimator(m_robotDrive.getPoseEstimator());
        vision.setChassisSpeedsSupplier(m_robotDrive::getSpeeds); // was getChassisSpeeds
        vision.setHeadingSupplier(m_robotDrive::getRotation);
        vision.setVisionMeasurementConsumer(m_robotDrive::addVisionMeasurement);
        vision.setPreciseVisionMeasurementConsumer(m_robotDrive::addPreciseVisionMeasurement);

        // Enable Flywheel PID Control with default target of 0
        // NOTE: kP will be live-tuned via SmartDashboard during testing
        // Start with kP = 0.15, then adjust based on response
        m_shooter.enableFlywheelVelocityPID(0.0);

        // Configure the button bindings
        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        // Auto chooser
        // Registered named commands for Path Planner.
        //NamedCommands.registerCommand("Shooter", m_shooter.runShooterCommand()); //non-timer
        //NamedCommands.registerCommand("ShooterHub", m_shooter.autoShootCommand());
        NamedCommands.registerCommand("StopShooter", m_shooter.stopShooter());
        NamedCommands.registerCommand("Spindexer", m_spindexer.runSpindexerCommand(false).withTimeout(2.7)); //non-timer 
        NamedCommands.registerCommand("Intake", m_intake.runIntakeCommand().withTimeout(1.8)); //non-timer
        NamedCommands.registerCommand("Intake Arm Raise", m_intake.runRaiseCommand().withTimeout(1.5)); //non-timer 
        NamedCommands.registerCommand("Intake Arm Lower", m_intake.runLowerCommand().withTimeout(1.0)); //non-timer 
        //NamedCommands.registerCommand("Spindexer", m_spindexer.autoSpinCommand());

        // Register preset shooter configurations for different positions
         NamedCommands.registerCommand("ShooterHub", 
                 new ShooterWithParametersCommand(m_shooter, m_hood, 1.0, 0.0).withTimeout(11.0)); 
        
        NamedCommands.registerCommand("ShooterLR", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.675, 0.0)); // 80% speed, 35° hood
        
        NamedCommands.registerCommand("ShooterTrench", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 1.66, 0.0).withTimeout(11.0)); // 60% speed, 25° hood
        
        NamedCommands.registerCommand("ShooterClimb", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.6, 15.0)); // Close range
        
        NamedCommands.registerCommand("ShooterDepot", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.675, 15.0)); // Long range

        NamedCommands.registerCommand("ShooterSide", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.7, 15.0)); // Long range

        // Needs to be switched to autoRaiseCommand and autoLowerCommand so auton will use motor rotation limit rather than a timer
        NamedCommands.registerCommand("Climber", m_climber.autoRaiseCommand().withTimeout(3.0));
        // NamedCommands.registerCommand("Climber", m_climber.runRaiseCommandOld().withTimeout(3.0));
        NamedCommands.registerCommand("Descend", m_climber.autoLowerCommand().withTimeout(3.0));
        // NamedCommands.registerCommand("Descend", m_climber.runLowCommand().withTimeout(8.0));


                try {
                        autoChooser = AutoBuilder.buildAutoChooser();       
                        // Add alliance-aware autos that mirror based on alliance
                        autoChooser.addOption("Blue Left Neutral (Auto Mirror)", new BlueLeftNeutralAuto());
                        autoChooser.addOption("Blue Right Neutral (Auto Mirror)", new BlueRightNeutralAuto());
                        autoChooser.addOption("Blue Left Neutral Climb (Auto Mirror)", new BlueLefNeutralClimbAuto());
                        autoChooser.addOption("BLOA (Auto Mirror)", new BLOAAuto());
                        autoChooser.addOption("BROA (Auto Mirror)", new BROAAuto());
                        autoChooser.addOption("Blue Robo Auto L (Auto Mirror)", new BlueRoboAutoLAuto());
                        autoChooser.addOption("Blue Robo Auto R (Auto Mirror)", new BlueRoboAutoRAuto());
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
                                // The * .# limits the driver controller input by ##%
                                -MathUtil.applyDeadband(m_driverController.getLeftY() * .8, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX() * .8, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX() * .8, OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));

        // Operator Left Joystick controlled flywheel speed
        m_shooter.setDefaultCommand(
        new RunCommand(
                () -> {
                // The left stick Y measure sets the flywheel speed
                double y = m_operatorController.getLeftY();
                if (Math.abs(y) < 0.05) {
                        y = 0;
                }
                m_shooter.setFlywheelVelocity(y * 12);
                },
                m_shooter )
        );

        // ============ TELEOP SHOOTER SETUP ============
        // TODO: Uncomment once distance-to-parameter mapping is tuned
        /*
        // Set up dynamic shooter control for teleop
        m_shooter.setDefaultCommand(
            new TeleopShooterCommand(m_robotDrive, m_shooter, m_hood)
        );
        */
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
        //Driver Controller Commands
        //Makes wheels enter an X shape for defense
        m_driverController.x().whileTrue(new RunCommand(
                () -> m_robotDrive.setX(), 
                m_robotDrive));
        //Reset Heading
        m_driverController.start().onTrue(new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
        
        //Intake Commands
        m_driverController.leftBumper().toggleOnTrue(m_intake.runIntakeCommand());
        m_driverController.rightBumper().toggleOnTrue(m_intake.runOuttakeCommand());

        // Working arm code commented for other command testing
          m_driverController
                  .leftTrigger(OIConstants.kTriggerButtonThreshold)
                 .whileTrue(m_intake.runLowerCommand());
          m_driverController
                  .rightTrigger(OIConstants.kTriggerButtonThreshold)
                 .whileTrue(m_intake.runRaiseCommand());
        //test code below
        //   m_driverController
        //         .leftTrigger(OIConstants.kTriggerButtonThreshold)
        //          .whileTrue(m_intake.runLowArmCommand());
        //  m_driverController
        //          .rightTrigger(OIConstants.kTriggerButtonThreshold)
        //          .whileTrue(m_intake.runRaiseArmCommand());

        //Climber Commands
        // m_driverController.pov(0).whileTrue(m_climber.runRaiseCommand()); //og command 
        m_driverController.pov(0).toggleOnTrue(m_climber.autoRaiseCommand()); //New command 
        // m_driverController.pov(180).whileTrue(m_climber.runDescendCommand()); //og command 
        m_driverController.pov(180).toggleOnTrue(m_climber.autoLowerCommand()); //New command 

        //Auto Align Command
        m_driverController.pov(90).whileTrue(m_AlignToHub);



        // Operator Controller Commands
        // Shooter Commands
        m_operatorController.y().toggleOnTrue(m_shooter.runShooterCommand()); 
        m_operatorController.b().toggleOnTrue(m_spindexer.runSpindexerCommand(false)); //runs the spinsdexer and the indexer(feeder) 

        // More Manual Shooter Commands
        m_operatorController.rightBumper().toggleOnTrue(m_shooter.increaseFlywheelVoltageCommand());
        m_operatorController.leftBumper().toggleOnTrue(m_shooter.decreaseFlywheelVoltageCommand()); 
        m_operatorController.a().toggleOnTrue(m_shooter.runTeleOpShooterCommand());
        
        // Hood Controls
        m_operatorController.pov(0).whileTrue(m_hood.autoUpCommand());
        m_operatorController.pov(180).whileTrue(m_hood.autoDownCommand()); 

        // Unknown Command
        // m_operatorController.a().toggleOnTrue(m_intake.runArmCyclePositionCommand(0, 0, 0)); //who added this??
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

    /**
     * Gets the Vision subsystem
     * @return the Vision subsystem instance
     */
    public Vision getVision() {
        return vision;
    }
}
