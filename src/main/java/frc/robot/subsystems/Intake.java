// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMax intakeArmMotor;

  public Intake() {
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanId, SparkMax.MotorType.kBrushless);
    intakeArmMotor = new SparkMax(IntakeConstants.kIntakeArmMotorCanId, SparkMax.MotorType.kBrushless);

    intakeMotor.configure(Configs.IntakeSubsystem.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeArmMotor.configure(Configs.IntakeSubsystem.intakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
