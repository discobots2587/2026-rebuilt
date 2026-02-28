// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ShooterSubsystemConstants.HubTarget;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToHub extends Command {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    
    private final DriveSubsystem m_drivetrain;
    private final ShooterSubsystem m_shooter;
    private final Vision m_vision;

    // Safety timer to stop if we lose the tag for too long
    private final Timer safetyTimer = new Timer();

  public AlignToHub(DriveSubsystem drivetrain, ShooterSubsystem shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_vision = vision;

    xController = new PIDController(1.0, 0.0, 0.0); // Tune these gains
    yController = new PIDController(1.0, 0.0, 0.0); // Tune these gains
    rotController = new PIDController(0.05, 0.0, 0.0); // Tune these gains


    // Set Tolerances (How close is "good enough"?)
    xController.setTolerance(HubTarget.kTargetXTol); 
    yController.setTolerance(HubTarget.kTargetYTol);
    rotController.setTolerance(2.0); // Degrees

    // Set the goal (Setpoints)
    // We want to be at kTargetX distance and 0.0 Y offset (centered)
    xController.setSetpoint(HubTarget.kTargetX);
    yController.setSetpoint(HubTarget.kTargetY); 
    rotController.setSetpoint(0.0); // We want 0 degrees rotation error
    
    // Allow rotation to wrap around 180 (so it takes the shortest path)
    rotController.enableContinuousInput(-180, 180);

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        safetyTimer.restart();
        
        // Reset PID loops so they don't remember old errors
        xController.reset();
        yController.reset();
        rotController.reset();

        SmartDashboard.putBoolean("AlignToHub/Running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // 1. Get Target Info
        Transform3d targetPos = m_vision.getTargetPos();
        double currentRange = targetPos.getX();
       // currentRange = diff_x;   remove hack
        SmartDashboard.putNumber("AlignToBranch/X",currentRange);
        double currentYOffset = targetPos.getY();
        // currentYOffset = diff_y;  remove hack
        SmartDashboard.putNumber("AlignToBranch/Y",currentYOffset);
        
        // IMPORTANT: Ensure your subsystem returns rotation in the Transform3d
        double currentYawError = Math.toDegrees(targetPos.getRotation().getZ());
        SmartDashboard.putNumber("AlignToBranch/rot",currentYawError); 
        currentYawError=0;//ignore rotaition
        // 2. Safety Check: If data is "empty" (0.0), treat as invalid.
        boolean validTarget = currentRange > 0.01 && currentRange < 4.0; 

        if (validTarget) {
            safetyTimer.reset(); // We saw a tag, reset safety timer

            // 3. Calculate Speeds
            // INVERTED Logic: If range is 2.0 (too far), error is negative, so we must invert to drive forward.
            double xSpeed = -xController.calculate(currentRange);
            double ySpeed = -yController.calculate(currentYOffset);
            double rotSpeed = rotController.calculate(currentYawError);

            // 4. Clamp Speeds (Safety)
            xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
            ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
            rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);

            // 5. Drive (Robot Relative)
            m_drivetrain.drive(xSpeed, ySpeed, rotSpeed, false); 

            // Debug
            SmartDashboard.putNumber("AlignToBranch/X_Speed", xSpeed);
            SmartDashboard.putNumber("AlignToBranch/Y_Speed", ySpeed);
            SmartDashboard.putNumber("AlignToBranch/Range", currentRange);
            SmartDashboard.putNumber("AlignToBranch/YOffset", currentYOffset);
        } else {
            // Target lost? Stop momentarily (or hunt if you prefer)
            m_drivetrain.drive(0, 0, 0, false); 
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0, 0, false);
        
        SmartDashboard.putBoolean("AlignToHub/Running", false);
        SmartDashboard.putBoolean("AlignToHub/Finished", !interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atX = xController.atSetpoint();
    boolean atY = yController.atSetpoint();
    // boolean atRot = rotController.atSetpoint(); // Optional: enable if rotation is critical

    // Timeout safety: If we haven't seen a tag for 0.5 seconds, just quit.
    if (safetyTimer.hasElapsed(0.5)) {
        return true;
    }

    return atX && atY; 
  }
}
