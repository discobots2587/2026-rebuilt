// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    climberMotor.configure(Configs.ClimberSubsystem.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0);
  }

  /** Returns true when the SPARK MAX reverse limit switch is triggered. */
  public boolean isAtLimit() {
    return climberMotor.getReverseLimitSwitch().isPressed();
  }

  /** Returns the climber encoder position in rotations. */
  public double getClimberRotations() {
    return climberEncoder.getPosition();
  }
  

  private void zeroClimberOnLimitSwitch() {
    if (!wasResetByLimit && isAtLimit()) {
      climberEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!isAtLimit()) {
      wasResetByLimit = false;
    }
  }

  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      climberEncoder.setPosition(0);
      wasResetByButton = true;
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  private void setClimberPower(double power) {
    if (isAtLimit() && power > 0 ) {
      climberMotor.set(0);
      return;
    }
    climberMotor.set(power);
  }

  private void setdownClimberPower(double power) {
    if ((isAtLimit() && power > 0) || getClimberRotations() <=4) {
      climberMotor.set(0);
      return;
    }
    climberMotor.set(power);
  }


  public Command runClimbCommand() {
  return this.startEnd(
        () -> {
  this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb);
        }, () -> {
  this.setClimberPower(0);
      }).withName("Climbing");
  }

public Command runDescendCommand() {
  return this.startEnd(
        () -> {
  this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend);
        },() -> {
  this.setClimberPower(0);
        }).withName("Descending");
  }
public Command runLowCommand() {
  return this.run(
            () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend)
        )
        .until(this::isAtLimit)          // stop if limit switch triggers
        .finallyDo(() -> this.setClimberPower(0))
        .withName("AutoRaise");
  }

  public Command runRaiseCommand() {
  return this.run(
            () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb)
        )
        .until(this::isAtLimit)          // stop if limit switch triggers
        .finallyDo(() -> this.setClimberPower(0))
        .withName("AutoRaise");
  } 
   public Command autoRaiseCommand() {
        return this.run(
            () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb)
        )
        .until(this::isAtLimit)          // stop if limit switch triggers
        .withTimeout(2.0)                // hard fallback: stop after 2 seconds
        .finallyDo(() -> this.setClimberPower(0))
        .withName("AutoRaise");

    }

  public Command autoLowerCommand() {
    return this.run(
        () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend)
    )
    .withTimeout(2.0)
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
  }
}

/** comment pt1 for new code
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ClimberSubsystemConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
public class ClimberSubsystem extends SubsystemBase {
  //Creates a new ClimberSubsystem. 
private final SparkMax climberMotor;
private final DigitalInput limitSwitch;

public ClimberSubsystem() {
  // Initialize the climber motor
  climberMotor = new SparkMax(ClimberSubsystemConstants.kClimberMotorCanId, SparkMax.MotorType.kBrushless);
  //config climber motor
  climberMotor.configure(Configs.ClimberSubsystem.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   limitSwitch = new DigitalInput(ClimberSubsystemConstants.kClimberLimitSwitchPort);

    } 
  

  public boolean isAtLimit() {
        return !limitSwitch.get(); // invert for normally-open switch
    }

private void setClimberPower(double power) {
   if (isAtLimit() && power > 0) {
            climberMotor.set(0);
            return;
        }
  climberMotor.set(power);
  } 
public Command runClimbCommand() {
  return this.startEnd(
        () -> {
  this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb);
        }, () -> {
  this.setClimberPower(0);
      }).withName("Climbing");
  }

public Command runDescendCommand() {
  return this.startEnd(
        () -> {
  this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend);
        },() -> {
  this.setClimberPower(0);
        }).withName("Descending");
  }
  */
  
/**
     * Runs the climber until the limit switch triggers OR 2 seconds elapse.
     * Uses run().until() instead of a blocking while loop so the scheduler
     * continues running normally.
     */ 
  //commented pt2 for new code
    
     /* 
    public Command autoRaiseCommand() {
        return this.run(
            () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb)
        )
        .until(this::isAtLimit)          // stop if limit switch triggers
        .withTimeout(2.0)                // hard fallback: stop after 2 seconds
        .finallyDo(() -> this.setClimberPower(0))
        .withName("AutoRaise");
    }

    public Command autoLowerCommand() {
    return this.run(
        () -> this.setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend)
    )
    .withTimeout(2.0)
    .finallyDo(() -> this.setClimberPower(0))
    .withName("AutoLower");
}



  @Override
public void periodic() {
// This method will be called once per scheduler run
SmartDashboard.putNumber("Climber Motor Output", climberMotor.getAppliedOutput());
SmartDashboard.putBoolean("Climber At Limit", isAtLimit());


  }
}
*/