package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.HoodSubsystemConstants;
import frc.robot.Constants.HoodSubsystemConstants.HoodMotorSetPoints;
import frc.robot.Configs;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase{
    //private final SparkMax hoodMotor;
    //private RelativeEncoder hoodEncoder;
    private SparkMax hoodMotor = new SparkMax(HoodSubsystemConstants.kHoodCanID, MotorType.kBrushless);
    private SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
    private RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    public HoodSubsystem() {
        //hoodMotor = new SparkMax(HoodSubsystemConstants.kHoodCanID, MotorType.kBrushless);
        
      hoodEncoder.setPosition(0);
      hoodMotor.configure(
        Configs.ShooterSubsystem.hoodConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }
    
    /**
     * Moves hood to a specific angle using closed-loop control
     * @param targetAngle Target hood angle (in degrees or rotations depending on your mechanism)
     */

  public boolean isUpLimit() { // At limit of bottom
    double rotationLimit = 0.25; // The min value of rotations the motor should go down to. (prevent going down too far)
    if (getHoodRotations() <= 0) {
      // Checks if the limit switch or rotations limit is hit
      return true;
    } else {
      return false;
    }
    // return climberMotor.getReverseLimitSwitch().isPressed(); // Original Code
  }

  public boolean isDownLimit() {
    if (getHoodRotations() >= .5) {
      return true;
    } else {
      return false;
    }
    // return climberMotor.getForwardLimitSwitch().isPressed(); // Original Code
  }
   public double getHoodRotations() {
    return hoodEncoder.getPosition();
  }

public void setHoodPower(double power) {
    if ((isDownLimit() && power > 0)) {
      hoodMotor.set(0);
      return;
    } else if (power < 0 && isUpLimit()) {
      // turn off in auton
      hoodMotor.set(0);
      return;
    }
    hoodMotor.set(power);
  }

public Command autoUpCommand() {
    return this.run(
        () -> this.setHoodPower(HoodSubsystemConstants.HoodMotorSetPoints.kBackMove))
        .until(this::isUpLimit) 
        .finallyDo(() -> this.setHoodPower(0))
        .withName("Up");
  }
  
  public Command autoDownCommand() {
    return this.run(
        () -> this.setHoodPower(HoodSubsystemConstants.HoodMotorSetPoints.kMove))
        .until(this::isDownLimit)
        .finallyDo(() -> this.setHoodPower(0))
        .withName("Down");
  }


       @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood | Applied Output", hoodMotor.getAppliedOutput());
    SmartDashboard.putNumber("Hood /Rotations", getHoodRotations());

  }

  // ============ POSITION-BASED HOOD CONTROL ============
  /*
   * TODO: Uncomment and configure once you add a closed-loop controller
   * This will allow moving to specific angles instead of just open-loop power
   */
  
  
  //private SparkClosedLoopController hoodClosedLoopController;
  //private double targetHoodAngle = 0.0;
  
  // Conversion factor from motor rotations to degrees
  // TODO: Calculate based on your hood gearing
  // private static final double ROTATIONS_TO_DEGREES = 360.0; // Adjust based on gearing
  
  public void setHoodPosition(double position)
  {
    hoodController.setSetpoint(position, ControlType.kPosition);
  }
  
  public Command hoodDown()
  {
    return this.runOnce(() -> setHoodPosition(Constants.HoodSubsystemConstants.HoodMotorSetPoints.kDown));
  }

  public Command hoodUp()
  {
    return this.runOnce(() -> setHoodPosition(Constants.HoodSubsystemConstants.HoodMotorSetPoints.kUp));
  }
  

}
