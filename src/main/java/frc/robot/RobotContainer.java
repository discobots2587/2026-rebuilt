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
import frc.robot.commands.AlignToHub;
import frc.robot.commands.ShooterWithParametersCommand;
import frc.robot.commands.BlueLeftNeutralAuto;
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
    private final SpindexerSubsystem m_spindexer = new SpindexerSubsystem();
    private final HoodSubsystem m_hood = new HoodSubsystem();


    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    Vision vision = new Vision();
    private SendableChooser<Command> autoChooser;

    //test align
    private final AlignToHub m_AlignToHub = new AlignToHub(m_robotDrive, m_shooter);

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
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
        //NamedCommands.registerCommand("Shooter", m_shooter.runShooterCommand()); //non-timer

        //NamedCommands.registerCommand("ShooterHub", m_shooter.autoShootCommand());



        NamedCommands.registerCommand("StopShooter", m_shooter.stopShooter());

        NamedCommands.registerCommand("Spindexer", m_spindexer.runSpindexerCommand(false).withTimeout(5.0)); //non-timer 

        NamedCommands.registerCommand("Intake", m_intake.runIntakeCommand().withTimeout(5.0)); //non-timer

        NamedCommands.registerCommand("Intake Arm Raise", m_intake.runRaiseCommand().withTimeout(1.0)); //non-timer 
        NamedCommands.registerCommand("Intake Arm Lower", m_intake.runLowerCommand().withTimeout(5.0)); //non-timer 

        //NamedCommands.registerCommand("Spindexer", m_spindexer.autoSpinCommand());


        
    
        // Register preset shooter configurations for different positions
         NamedCommands.registerCommand("ShooterHub", 
                 new ShooterWithParametersCommand(m_shooter, m_hood, 1.0, 0.0).withTimeout(6.0)); // Full speed, 45° hood
        
        NamedCommands.registerCommand("ShooterLR", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.675, 0.0)); // 80% speed, 35° hood
        
        NamedCommands.registerCommand("ShooterTrench", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.675, 35.0)); // 60% speed, 25° hood
        
        NamedCommands.registerCommand("ShooterClimb", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.6, 15.0)); // Close range
        
        NamedCommands.registerCommand("ShooterDepot", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.675, 15.0)); // Long range

        NamedCommands.registerCommand("ShooterSide", 
                new ShooterWithParametersCommand(m_shooter, m_hood, 0.7, 15.0)); // Long range

        NamedCommands.registerCommand("Climber", m_climber.runRaiseCommand().withTimeout(3.0));
        NamedCommands.registerCommand("Descend", m_climber.runLowCommand().withTimeout(8.0));


                try {

                        autoChooser = AutoBuilder.buildAutoChooser();
                        
                        // Add alliance-aware autos that mirror based on alliance
                        autoChooser.addOption("Blue Left Neutral (Auto Mirror)", new BlueLeftNeutralAuto());
                        
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
                                -MathUtil.applyDeadband(m_driverController.getLeftY() * .6, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX() * .6, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX() * .6, OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));


        m_shooter.setDefaultCommand(
        new RunCommand(
                () -> {
         // The left stick Y measure sets the flywheel speed
                double y = m_driverController.getLeftY();
                if (Math.abs(y) < 0.05) {
                        y = 0;
                }
                m_shooter.setFlywheelVelocity(y * 12 * -1);
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

        // new JoystickButton(m_driverController, Button.kR1.value)
        //         .whileTrue(new RunCommand(
        //                 () -> m_robotDrive.setX(),
        //                 m_robotDrive));

        //Driver Controller Commands
        //Reset Heading
        m_driverController.start().onTrue(new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
        
        //Intake Commands
        m_driverController.leftBumper().toggleOnTrue(m_intake.runIntakeCommand());

        m_driverController.rightBumper().toggleOnTrue(m_intake.runOuttakeCommand());



        //working arm code commented for other command testing
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



        
        
        //Shooter Commands
        // m_driverController.y().toggleOnTrue(m_shooter.runShooterCommand()); 
        // m_driverController.b().toggleOnTrue(m_spindexer.runSpindexerCommand(false)); //runs the spinsdexer and the indexer(feeder) 
        // m_driverController.x().whileTrue(m_hood.runHoodCommand());
        // m_driverController.a().whileTrue(m_hood.runbackHoodCommand());

        //Climber Commands
        m_driverController.pov(0).whileTrue(m_climber.runRaiseCommand()); //og command 
        // m_driverController.pov(0).toggleOnTrue(m_climber.autoClimberCommand()); //testing for one click climber 
        
        m_driverController.pov(180).whileTrue(m_climber.runDescendCommand());

        //Auto Align Command
        m_driverController.pov(90).whileTrue(m_AlignToHub);

        //Operator Controller Commands
        //Shooter Commands
        m_operatorController.y().toggleOnTrue(m_shooter.runShooterCommand()); 
        m_operatorController.b().toggleOnTrue(m_spindexer.runSpindexerCommand(false)); //runs the spinsdexer and the indexer(feeder) 
        m_operatorController.pov(0).whileTrue(m_hood.runHoodCommand());
        m_operatorController.pov(180).whileTrue(m_hood.runbackHoodCommand()); 
        m_operatorController.a().toggleOnTrue(m_intake.runArmCyclePositionCommand(0, 0, 0)); //who added this??
        
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
