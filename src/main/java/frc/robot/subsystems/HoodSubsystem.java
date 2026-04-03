package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodSubsystemConstants;
import frc.robot.Constants.HoodSubsystemConstants.HoodMotorSetPoints;
import frc.robot.Configs;

public class HoodSubsystem extends SubsystemBase{
    public final SparkMax hoodMotor;
    public HoodSubsystem() {
        hoodMotor = new SparkMax(HoodSubsystemConstants.kHoodCanID, MotorType.kBrushless);
        hoodMotor.configure(Configs.ShooterSubsystem.hoodConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);

    }
    public void setHoodMotorPower(double power){
    hoodMotor.set(power);
    }
    
    /**
     * Moves hood to a specific angle using closed-loop control
     * @param targetAngle Target hood angle (in degrees or rotations depending on your mechanism)
     */
    public void moveHoodToAngle(double targetAngle) {
        // If you have a closed-loop controller, use it here
        // Otherwise, you can use open-loop with a timeout
        // This depends on your hood motor configuration
        
        // Placeholder - adjust based on your actual hood implementation
        // hoodController.setSetpoint(targetAngle, ControlType.kPosition);
    }
    
  public Command runHoodCommand(){
    return this.startEnd(
      ()-> {
        this.setHoodMotorPower(HoodMotorSetPoints.kMove);
      },
      ()-> {
        this.setHoodMotorPower(0.0);
      }
    ).withName("Hood");
  }
  public Command runbackHoodCommand(){
    return this.startEnd(
      ()-> {
        this.setHoodMotorPower(HoodMotorSetPoints.kBackMove);
      },
      ()-> {
        this.setHoodMotorPower(0.0);
      }
    ).withName("Hood");
  }
       @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood | Applied Output", hoodMotor.getAppliedOutput());
  }

  // ============ POSITION-BASED HOOD CONTROL ============
  /*
   * TODO: Uncomment and configure once you add a closed-loop controller
   * This will allow moving to specific angles instead of just open-loop power
   */
  
  /*
  private SparkClosedLoopController hoodClosedLoopController;
  private RelativeEncoder hoodEncoder;
  private double targetHoodAngle = 0.0;
  
  // Conversion factor from motor rotations to degrees
  // TODO: Calculate based on your hood gearing
  // private static final double ROTATIONS_TO_DEGREES = 360.0; // Adjust based on gearing
  
  public void initializeClosedLoopControl() {
    hoodClosedLoopController = hoodMotor.getClosedLoopController();
    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPosition(0);
  }
  
  public void setHoodAngleClosed(double targetAngleDegrees) {
    targetHoodAngle = targetAngleDegrees;
    // Convert degrees to motor rotations
    // double targetRotations = targetAngleDegrees / ROTATIONS_TO_DEGREES;
    // hoodClosedLoopController.setSetpoint(targetRotations, ControlType.kPosition);
  }
  
  public double getHoodAngle() {
    // return hoodEncoder.getPosition() * ROTATIONS_TO_DEGREES;
    return 0.0; // Placeholder
  }
  */

}
