// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private boolean wasResetByLimit = false;
  private boolean wasResetByButton = false;

  public ClimberSubsystem() {
    climberMotor = new SparkMax(ClimberSubsystemConstants.kClimberMotorCanId, SparkMax.MotorType.kBrushless);
    climberMotor.configure(Configs.ClimberSubsystem.climberMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0);
  }

  // Returns true when the SPARK MAX reverse limit switch is triggered and uses
  // rotation as a safety mechanism
  public boolean isAtLimit() { // At limit of bottom
    double rotationLimit = 0.25; // The min value of rotations the motor should go down to. (prevent going down too far)
    if (DriverStation.isAutonomous() || (DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() >= 0)) {
      // If it is auton or last 30 seconds then we want to actually climb. This means
      // should be raising the robot up to the max.
      // Possible logic error here with how match time (-1) is dealt with outside of
      // FMS connections
      rotationLimit = 4;
    }
    if (climberMotor.getReverseLimitSwitch().isPressed() || getClimberRotations() <= rotationLimit) {
      // Checks if the limit switch or rotations limit is hit
      return true;
    } else {
      return false;
    }
    // return climberMotor.getReverseLimitSwitch().isPressed(); // Original Code
  }

  public boolean isAttopLimit() {
    if (climberMotor.getForwardLimitSwitch().isPressed() || getClimberRotations() >= 83) {
      return true;
    } else {
      return false;
    }
    // return climberMotor.getForwardLimitSwitch().isPressed(); // Original Code
  }

  public boolean isAtLimitForLimitSwitch() {
    // This method was duplicated from the original as its used for zeroing the bottom with the limit switch.
    // This may get refactor if the limit switch is removed.
    return climberMotor.getReverseLimitSwitch().isPressed();
  }

  // Returns the climber encoder position in rotations. 
  public double getClimberRotations() {
    return climberEncoder.getPosition();
  }

  private void zeroClimberOnLimitSwitch() {
    if (!wasResetByLimit && isAtLimitForLimitSwitch()) {
      climberEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!isAtLimitForLimitSwitch()) {
      wasResetByLimit = false;
    }
  }

  // The User button is on the roboRIO, this is probably unnecessary
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      climberEncoder.setPosition(0);
      wasResetByButton = true;
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  private void setClimberPower(double power) {
    if ((isAttopLimit() && power > 0)) {
      climberMotor.set(0);
      return;
    } else if (power < 0 && isAtLimit()) {
      // turn off in auton
      climberMotor.set(0);
      return;
    }
    climberMotor.set(power);
  }

  // public Command runRaiseCommandOld() {
  //   return this.startEnd(
  //       () -> {
  //         this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb);
  //       }, () -> {
  //         this.setClimberPower(0);
  //       }).withName("Descending");
  // }

  // public Command runLowCommand() {
  //   return this.run(
  //       () -> this.setdownClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend))
  //       .finallyDo(() -> this.setdownClimberPower(0))
  //       .withName("AutoRaise");
  // }

  public Command autoRaiseCommand() {
    return this.run(
        () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb))
        .until(this::isAttopLimit) 
        .withTimeout(3.0) 
        .finallyDo(() -> this.setClimberPower(0))
        .withName("AutoRaise");
  }
  
  public Command autoLowerCommand() {
    return this.run(
        () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend))
        .until(this::isAtLimit)
        .withTimeout(3.0)
        .finallyDo(() -> this.setClimberPower(0))
        .withName("AutoLower");
  }

  @Override
  public void periodic() {
    zeroClimberOnLimitSwitch();
    zeroOnUserButton();

    SmartDashboard.putNumber("Climber/Motor Output", climberMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Climber/At Limit", isAtLimit());
    SmartDashboard.putBoolean("Climber/Reverse Limit", climberMotor.getReverseLimitSwitch().isPressed());
    SmartDashboard.putBoolean("Climber/Forward Limit", climberMotor.getForwardLimitSwitch().isPressed());
    SmartDashboard.putNumber("Climber/Rotations", getClimberRotations());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());

  }
}