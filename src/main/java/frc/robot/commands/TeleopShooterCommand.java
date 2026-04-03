package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Teleop command that dynamically adjusts flywheel speed and hood angle based on robot distance from hub
 * TODO: Uncomment and activate once distance-to-parameter mapping data is available
 */
public class TeleopShooterCommand extends Command {
  private DriveSubsystem m_drivetrain;
  private ShooterSubsystem m_shooter;
  private HoodSubsystem m_hood;

  // Hub positions (same as AlignToHub)
  private static final Translation2d BLUE_HUB = new Translation2d(4.63, 4.03);
  private static final Translation2d RED_HUB = new Translation2d(11.92, 4.03);
  private Translation2d activeHub;
  
  // Temporary constructor to prevent errors while code is commented
  public TeleopShooterCommand() {
    // TODO: Remove this constructor when uncommenting the full implementation
  }

  /*
   * TODO: Uncomment and test once you have distance-to-speed and distance-to-hood data
   * 
   * public TeleopShooterCommand(DriveSubsystem drivetrain, ShooterSubsystem shooter, HoodSubsystem hood) {
   *   m_drivetrain = drivetrain;
   *   m_shooter = shooter;
   *   m_hood = hood;
   *   addRequirements(m_shooter, m_hood);
   * }
   * 
   * @Override
   * public void initialize() {
   *   // Determine active hub based on alliance
   *   var alliance = DriverStation.getAlliance();
   *   if (alliance.isPresent()) {
   *     activeHub = alliance.get() == Alliance.Blue ? BLUE_HUB : RED_HUB;
   *   } else {
   *     activeHub = BLUE_HUB; // Default to blue
   *   }
   * }
   * 
   * @Override
   * public void execute() {
   *   // Get current robot pose
   *   Pose2d robotPose = m_drivetrain.getPose();
   *   
   *   // Calculate distance to hub
   *   double dx = activeHub.getX() - robotPose.getX();
   *   double dy = activeHub.getY() - robotPose.getY();
   *   double distanceToHub = Math.sqrt(dx * dx + dy * dy);
   *   
   *   // Get optimal flywheel voltage and hood angle based on distance
   *   double flywheelVoltage = m_shooter.getFlywheelVoltageFromDistance(distanceToHub);
   *   double hoodAngle = m_shooter.getHoodAngleFromDistance(distanceToHub);
   *   
   *   // Apply to motors
   *   m_shooter.setFlywheelVelocity(flywheelVoltage);
   *   m_hood.setHoodAngleClosed(hoodAngle);
   *   
   *   // Log to SmartDashboard for debugging
   *   SmartDashboard.putNumber("TeleopShooter/DistanceToHub", distanceToHub);
   *   SmartDashboard.putNumber("TeleopShooter/TargetFlywheelVoltage", flywheelVoltage);
   *   SmartDashboard.putNumber("TeleopShooter/TargetHoodAngle", hoodAngle);
   * }
   * 
   * @Override
   * public void end(boolean interrupted) {
   *   m_shooter.setFlywheelVelocity(0.0);
   *   m_hood.setHoodMotorPower(0.0);
   * }
   * 
   * @Override
   * public boolean isFinished() {
   *   return false; // Runs continuously in teleop
   * }
   */
}
