// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeArmSetPoints;
import frc.robot.Constants.IntakeConstants.IntakeSetPoints;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMax intakeArmMotor;
  private final SparkMax intakeArmFollowerMotor;
  // Encoder and closed-loop controller for the intake arm
  private RelativeEncoder intakeArmEncoder;
  private SparkClosedLoopController intakeArmController;
  // Safety limits (rotations) and default timeout (seconds)
  private double m_minIntakeArmRotations = -2.0; // tune to your mechanism
  private double m_maxIntakeArmRotations = 20.0; // tune to your mechanism
  private double m_moveTimeoutSeconds = 3.0;

  public Intake() {
    // initializing motors
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanId, SparkMax.MotorType.kBrushless);
    intakeArmMotor = new SparkMax(IntakeConstants.kIntakeArmMotorCanId, SparkMax.MotorType.kBrushless);
    intakeArmFollowerMotor = new SparkMax(IntakeConstants.kIntakeArmMotorFollowerCanId, SparkMax.MotorType.kBrushless);

    intakeMotor.configure(Configs.IntakeSubsystem.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeArmMotor.configure(Configs.IntakeSubsystem.intakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeArmFollowerMotor.configure(Configs.IntakeSubsystem.intakeArmFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  // Initialize encoder and closed-loop controller for intake arm
  intakeArmEncoder = intakeArmMotor.getEncoder();
  intakeArmController = intakeArmMotor.getClosedLoopController();
  // Optionally zero the encoder at startup
  intakeArmEncoder.setPosition(0);

  // Publish safety limits to SmartDashboard for tuning
  SmartDashboard.putNumber("IntakeArm/MinRotations", m_minIntakeArmRotations);
  SmartDashboard.putNumber("IntakeArm/MaxRotations", m_maxIntakeArmRotations);
  SmartDashboard.putNumber("IntakeArm/MoveTimeout", m_moveTimeoutSeconds);

  }

  private void setDownArmPower(double power) {
    if (getClimberRotations() <=4) {
      intakeArmMotor.set(0);
      return;
    }
    intakeArmMotor.set(power);
  }

  private void setRaiseArmPower(double power) {
    if (getClimberRotations() >= 10) {
      intakeArmMotor.set(0);
      return;
    }
    intakeArmMotor.set(power);
  }


  public double getClimberRotations() {
    return intakeArmEncoder.getPosition();
  }
  private void setIntakePower(double power){
    intakeMotor.set(power);
  }
  //set speed for the arm motor 
  private void setIntakeArmPower(double power){
    intakeArmMotor.set(power);
  }

  private void setIntakeArmFollowerPower(double power){
    intakeArmFollowerMotor.set(power);
  }

  //command to run the motors (if command interrupted the motors will stop)

  public Command runIntakeCommand(){
    return this.startEnd(
      () -> {
      this.setIntakePower(IntakeSetPoints.kIntake);
      }, () ->{
        this.setIntakePower(0);
      }).withName("Intaking");
  }
  //command to run the motors in (if command interrupted the motors will stop)
  public Command runOuttakeCommand(){
    return this.startEnd(
      () -> {
      this.setIntakePower(IntakeSetPoints.kExtake);
      }, () ->{
        this.setIntakePower(0);
      }).withName("Outtaking");
  }
  public Command runRaiseCommand(){
      return this.startEnd(
        
        () -> {
        this.setIntakeArmPower(IntakeArmSetPoints.kRaise);
        this.setIntakePower(IntakeSetPoints.kIntake);
        // this.setIntakeArmFollowerPower(IntakeArmSetPoints.kRaise);
        }, () ->{
          this.setIntakeArmPower(0);
          this.setIntakeArmFollowerPower(0);
          this.setIntakePower(0);
        }).withName("Raising Arm");
    }
  public Command runLowerCommand(){
      return this.startEnd(
        () -> {
        this.setIntakeArmPower(IntakeArmSetPoints.kLower);
        this.setIntakePower(IntakeSetPoints.kIntake);
        // this.setIntakeArmFollowerPower(IntakeArmSetPoints.kLower);
        }, () ->{
          this.setIntakeArmPower(0);
          this.setIntakeArmFollowerPower(0);
          this.setIntakePower(0);
        }).withName("Lowering Arm");
    }

  public Command runPlow(){
    // Simple sequence: raise the arm for a set duration, then lower it.
    // This implementation uses timed open-loop commands. If you prefer
    // position control, use moveIntakeArmToRotations and related methods.
    return this.startEnd(
      () -> {
        this.setIntakeArmPower(IntakeArmSetPoints.kRaise);
      },
      () -> {
        this.setIntakeArmPower(0);
      }
    ).withTimeout(0.5)
    .andThen(new WaitCommand(0.1))
    .andThen(this.startEnd(
      () -> { this.setIntakeArmPower(IntakeArmSetPoints.kLower); },
      () -> { this.setIntakeArmPower(0); }
    ).withTimeout(0.5)).withName("PlowSequence");
  }

  /** Returns the intake arm encoder position in rotations. */
  public double getIntakeArmRotations() {
    if (intakeArmEncoder != null) {
      return intakeArmEncoder.getPosition();
    }
    return 0.0;
  }

  /** Resets the intake arm encoder to zero. */
  public void resetIntakeArmEncoder() {
    if (intakeArmEncoder != null) {
      intakeArmEncoder.setPosition(0);
    }
  }

  /** Move the intake arm to a target in motor rotations using closed-loop position control. */
  public void moveIntakeArmToRotations(double rotations) {
    // Clamp target to safety limits
    double target = Math.max(m_minIntakeArmRotations, Math.min(m_maxIntakeArmRotations, rotations));
    if (intakeArmController != null) {
      // Use position control (native SparkMax position setpoint is in rotations)
      intakeArmController.setSetpoint(target, ControlType.kPosition);
    } else {
      // Fallback: open-loop approximate control (not precise)
      double power = target > getIntakeArmRotations() ? IntakeArmSetPoints.kRaise : IntakeArmSetPoints.kLower;
      intakeArmMotor.set(power);
    }
  }

  /**
   * Position-based command that moves the intake arm to raiseRotations then back to lowerRotations.
   * It waits until the encoder reports within tolerance of the target before proceeding.
   */
  public Command runArmCyclePositionCommand(double raiseRotations, double lowerRotations, double tolerance) {
    // Clamp inputs to safety limits
    double raiseTarget = Math.max(m_minIntakeArmRotations, Math.min(m_maxIntakeArmRotations, raiseRotations));
    double lowerTarget = Math.max(m_minIntakeArmRotations, Math.min(m_maxIntakeArmRotations, lowerRotations));

    // Build robust move-to-target command: power the arm (set closed-loop setpoint)
    // until the encoder is within tolerance or the timeout expires. Use a
    // ParallelRaceGroup so either the arrival condition or the timeout ends
    // the powering command. The startEnd ensures motors are stopped on end
    // or interruption.
  StartEndCommand raisePowerCmd = new StartEndCommand(
    () -> moveIntakeArmToRotations(raiseTarget),
    () -> { setIntakeArmPower(0); setIntakeArmFollowerPower(0); },
    this);
  WaitUntilCommand raiseWait = new WaitUntilCommand(() -> Math.abs(getIntakeArmRotations() - raiseTarget) <= tolerance);

  StartEndCommand lowerPowerCmd = new StartEndCommand(
    () -> moveIntakeArmToRotations(lowerTarget),
    () -> { setIntakeArmPower(0); setIntakeArmFollowerPower(0); },
    this); 
  WaitUntilCommand lowerWait = new WaitUntilCommand(() -> Math.abs(getIntakeArmRotations() - lowerTarget) <= tolerance);

  // ParallelRaceGroup will finish when either the powering command ends (due to timeout) or the WaitUntil succeeds.
  ParallelRaceGroup raisePhase = new ParallelRaceGroup(raisePowerCmd, raiseWait).withTimeout(m_moveTimeoutSeconds);
  ParallelRaceGroup lowerPhase = new ParallelRaceGroup(lowerPowerCmd, lowerWait).withTimeout(m_moveTimeoutSeconds);

    SequentialCommandGroup seq = new SequentialCommandGroup(raisePhase, lowerPhase);

    // Ensure motors are stopped if the overall sequence ends or is interrupted.
    return seq.withName("ArmCyclePosition").andThen(new InstantCommand(() -> {
      setIntakeArmPower(0);
      setIntakeArmFollowerPower(0);
    }, this));
  }

  //add command to raise and lower 


private boolean isSpinnerAt(double velocity) {
    return MathUtil.isNear(intakeMotor.getOutputCurrent(), 
            velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  /** 
   * Trigger: Is the flywheel spinning at the required velocity?
   */
   
  public final Trigger isFlywheelSpinning = new Trigger(
      () -> isSpinnerAt(IntakeArmSetPoints.kStallCurrentThreshehold) // || flywheelEncoder.getVelocity() > FlywheelSetpoints.kShootRpm 
  );

  public Command runLowArmCommand() {
  return this.run(
            () -> this.setDownArmPower(IntakeArmSetPoints.kLower)
        )
        .until(() -> getClimberRotations() <= 4)          // stop if limit switch triggers
        .finallyDo(() -> this.setDownArmPower(0))
        .withName("AutoLower");
  }

  public Command runRaiseArmCommand() {
  return this.run(
            () -> this.setRaiseArmPower(IntakeArmSetPoints.kRaise)
        )
        .until(() -> getClimberRotations() >= 10)          // stop if limit switch triggers
        .finallyDo(() -> this.setRaiseArmPower(0))
        .withName("AutoRaise");
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/ Applied Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("IntakeArm/Applied Output", intakeArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("IntakeArm/Rotations", getIntakeArmRotations());

    // Read possible updated limits from dashboard
    m_minIntakeArmRotations = SmartDashboard.getNumber("IntakeArm/MinRotations", m_minIntakeArmRotations);
    m_maxIntakeArmRotations = SmartDashboard.getNumber("IntakeArm/MaxRotations", m_maxIntakeArmRotations);
    m_moveTimeoutSeconds = SmartDashboard.getNumber("IntakeArm/MoveTimeout", m_moveTimeoutSeconds);

  }
}
