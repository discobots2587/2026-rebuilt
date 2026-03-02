// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.SpindexerSetpoints;
import frc.robot.Constants.IntakeConstants.IntakeSetPoints;
import frc.robot.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  // Initialize flywheel SPARKs. We will use MAXMotion velocity control for the flywheel, so we also need to
  // initialize the closed loop controllers and encoders.
  private SparkMax flywheelMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
  private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

  private SparkMax flywheelFollowerMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);

  // Initialize feeder SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax feederMotor =
      new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);
    
  private SparkMax spindexerMotor =
      new SparkMax(ShooterSubsystemConstants.kSpindexerCanID, MotorType.kBrushless);

  // Member variables for subsystem state management
  private double flywheelTargetVelocity = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    flywheelMotor.configure(
        Configs.ShooterSubsystem.flywheelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    flywheelFollowerMotor.configure(
        Configs.ShooterSubsystem.flywheelFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    feederMotor.configure(
        Configs.ShooterSubsystem.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    spindexerMotor.configure(
      Configs.ShooterSubsystem.spindexerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Zero flywheel encoder on initialization
    flywheelEncoder.setPosition(0);

    System.out.println("---> ShooterSubsystem initialized");
  }

  private boolean isFlywheelAt(double velocity) {
    return MathUtil.isNear(flywheelEncoder.getVelocity(), 
            velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  /** 
   * Trigger: Is the flywheel spinning at the required velocity?
   */
  public final Trigger isFlywheelSpinning = new Trigger(
      () -> isFlywheelAt(-5000) || flywheelEncoder.getVelocity() > -5000
  );

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(5000) || flywheelEncoder.getVelocity() < 5000
  );

  /** 
   * Trigger: Is the flywheel stopped?
   */
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  /**
   * Drive the flywheels to their set velocity. This will use MAXMotion
   * velocity control which will allow for a smooth acceleration and deceleration to the mechanism's
   * setpoint.
   */
  private void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    flywheelTargetVelocity = velocity;
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
    feederMotor.set(power);
  }

  private void setSpindexerPower(double power) {
    spindexerMotor.set(power);
  }
  
  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
        },
        () -> {
          this.setFlywheelVelocity(0.0);
        }).withName("Spinning Up Flywheel");
  }

    public Command runFlywheelCommandHalfSpeed() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm/2);
        },
        () -> {
          this.setFlywheelVelocity(0.0);
        }).withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFeederCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
        }, () -> {
          this.setFlywheelVelocity(0.0);
          this.setFeederPower(0.0);
        }).withName("Feeding");
  }
   public Command runSpindexerCommand() {
    return this.startEnd(
        () -> {
          this.setSpindexerPower(SpindexerSetpoints.kSpindex);
        }, () -> {
          this.setSpindexerPower(0.0);
        }).withName("Spnindexing");
  }
  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches
   * the desired speed it starts the Feeder.
   */
  public Command runShooterCommand() {
    //add period to wait for shooter to spin up
    return this.startEnd(
      () -> this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm),
      () -> flywheelMotor.stopMotor()
    ).until(isFlywheelSpinning).andThen(
      this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
          // this.setSpindexerPower(SpindexerSetpoints.kSpindex);
        }, () -> {
          flywheelMotor.stopMotor();
          feederMotor.stopMotor();
          //spindexerMotor.stopMotor();
        })
    ).withName("Shooting");
  }
public Command runIntakeCommand(){
    return this.startEnd(
      () -> {
      this.setSpindexerPower(SpindexerSetpoints.kSpindex);
      }, () ->{
      this.setSpindexerPower(0);
      }).withName("Intaking");
  }


  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current", flywheelFollowerMotor.getOutputCurrent());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", flywheelEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }

}
