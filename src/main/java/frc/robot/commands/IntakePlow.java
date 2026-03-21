


package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakePlow extends Command{



private final Intake m_Intake;
    // Safety timer to stop if we lose the tag for too long

    private final Timer safetyTimer = new Timer();



    public IntakePlow(Intake intake){

        m_Intake = intake; 

    }
//     public AlignToHub(DriveSubsystem drivetrain, ShooterSubsystem shooter, Vision vision) {
//     // Use addRequirements() here to declare subsystem dependencies.
//         m_drivetrain = drivetrain;
//         m_shooter = shooter;
//         m_vision = vision;

//         xController = new PIDController(1.0, 0.0, 0.0); // Tune these gains
//         yController = new PIDController(1.0, 0.0, 0.0); // Tune these gains
//         rotController = new PIDController(0.05, 0.0, 0.0); // Tune these gains


//         // Set Tolerances (How close is "good enough"?)
//         xController.setTolerance(HubTarget.kTargetXTol); 
//         yController.setTolerance(HubTarget.kTargetYTol);
//         rotController.setTolerance(2.0); // Degrees

//         // Set the goal (Setpoints)
//         // We want to be at kTargetX distance and 0.0 Y offset (centered)
//         xController.setSetpoint(HubTarget.kTargetX);
//         yController.setSetpoint(HubTarget.kTargetY); 
//         rotController.setSetpoint(90.0); // We want 0 degrees rotation error
        
//         // Allow rotation to wrap around 180 (so it takes the shortest path)
//         rotController.enableContinuousInput(-180, 180);

//         addRequirements(m_drivetrain);
//     }

  




 // Called when the command is initially scheduled.
 @Override
  public void initialize() {
//         safetyTimer.restart();
        
//         // Reset PID loops so they don't remember old errors
//         xController.reset();
//         yController.reset();
//         rotController.reset();

  }

//   // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//         //push intake arm
        m_Intake.runRaiseCommand();
//       
   }

//   // Called once the command ends or is interrupted.
//   @Override
   public void end(boolean interrupted) {
     
         }
        
//         SmartDashboard.putBoolean("AlignToHub/Running", false);
//         SmartDashboard.putBoolean("AlignToHub/Finished", !interrupted);
   

}
