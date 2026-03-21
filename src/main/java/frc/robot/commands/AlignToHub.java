package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToHub extends Command {
    private final PIDController rotController;
    
    private final DriveSubsystem m_drivetrain;
    private final ShooterSubsystem m_shooter;
    
    // Hub positions (field coordinates)
    private static final Translation2d BLUE_HUB = new Translation2d(4.63, 4.03);
    private static final Translation2d RED_HUB = new Translation2d(11.92, 4.03);
    
    private Translation2d targetHub;
    private final Timer safetyTimer = new Timer();

    public AlignToHub(DriveSubsystem drivetrain, ShooterSubsystem shooter) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;

        rotController = new PIDController(0.05, 0.0, 0.01); // Tune rotation gains
        rotController.setTolerance(2.0); // Degrees
        rotController.enableContinuousInput(-180, 180);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        safetyTimer.restart();
        rotController.reset();
        
        // Determine target hub based on alliance
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                targetHub = BLUE_HUB;
                SmartDashboard.putString("AlignToHub/TargetAlliance", "BLUE");
            } else {
                targetHub = RED_HUB;
                SmartDashboard.putString("AlignToHub/TargetAlliance", "RED");
            }
        } else {
            // No alliance yet, default to blue (shouldn't happen in match)
            targetHub = BLUE_HUB;
            SmartDashboard.putString("AlignToHub/TargetAlliance", "UNKNOWN");
        }
        
        SmartDashboard.putNumber("AlignToHub/TargetX", targetHub.getX());
        SmartDashboard.putNumber("AlignToHub/TargetY", targetHub.getY());
        SmartDashboard.putBoolean("AlignToHub/Running", true);
    }

    @Override
    public void execute() {
        // Get current robot pose from odometry/vision
        Pose2d robotPose = m_drivetrain.getPose();
        
        SmartDashboard.putNumber("AlignToHub/RobotX", robotPose.getX());
        SmartDashboard.putNumber("AlignToHub/RobotY", robotPose.getY());
        SmartDashboard.putNumber("AlignToHub/RobotHeading", robotPose.getRotation().getDegrees());
        
        // Calculate vector from robot to hub
        Translation2d robotToHub = targetHub.minus(robotPose.getTranslation());
        
        // Calculate desired heading (angle toward hub)
        double desiredHeading = Math.atan2(robotToHub.getY(), robotToHub.getX());
        desiredHeading = Math.toDegrees(desiredHeading);
        
        // Get current heading
        double currentHeading = robotPose.getRotation().getDegrees();
        
        // Calculate heading error
        double headingError = desiredHeading - currentHeading;
        
        // Normalize error to [-180, 180]
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        SmartDashboard.putNumber("AlignToHub/DesiredHeading", desiredHeading);
        SmartDashboard.putNumber("AlignToHub/HeadingError", headingError);
        
        // Calculate rotation speed using PID
        double rotSpeed = rotController.calculate(currentHeading, desiredHeading);
        rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);
        
        SmartDashboard.putNumber("AlignToHub/RotSpeed", rotSpeed);
        
        // OVERRIDE: Drive with rotation only (no x/y movement from driver)
        // This completely ignores driver input and only uses our calculated rotation
        m_drivetrain.drive(0, 0, rotSpeed, false);
        
        safetyTimer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0, 0, false);
        SmartDashboard.putBoolean("AlignToHub/Running", false);
        SmartDashboard.putBoolean("AlignToHub/Finished", !interrupted);
    }

    @Override
    public boolean isFinished() {
        // Finish when heading is aligned to hub
        if (safetyTimer.hasElapsed(5.0)) {
            return true; // Safety timeout
        }
        
        return rotController.atSetpoint();
    }
}