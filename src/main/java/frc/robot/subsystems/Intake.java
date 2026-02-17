// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeArmSetPoints;
import frc.robot.Constants.IntakeConstants.IntakeSetPoints;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMax intakeArmMotor;

  public Intake() {
    // initializing motors
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanId, SparkMax.MotorType.kBrushless);
    intakeArmMotor = new SparkMax(IntakeConstants.kIntakeArmMotorCanId, SparkMax.MotorType.kBrushless);

    intakeMotor.configure(Configs.IntakeSubsystem.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeArmMotor.configure(Configs.IntakeSubsystem.intakeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  //Everything below this should by my (Brandon's) changes, feel free to do whatever you want with it since im not sure if it even works.
  //set speed for the intake motor 
  private void setIntakePower(double power){
    intakeMotor.set(power);
  }
  //set speed for the arm motor 
  private void setIntakeArmPower(double power){
    intakeArmMotor.set(power);
  }

  //command to run the motors (if command interrupted the motors will stop)

  public Command runIntakeCommand(){
    return this.startEnd(
      () -> {
      this.setIntakePower(IntakeSetPoints.kIntake);
      this.setIntakeArmPower(IntakeArmSetPoints.kIntake);
      }, () ->{
        this.setIntakePower(0);
        this.setIntakeArmPower(0);
      }).withName("Intaking");
  }
  //command to run the motors in (if command interrupted the motors will stop)
  public Command runOuttakeCommand(){
    return this.startEnd(
      () -> {
      this.setIntakePower(IntakeSetPoints.kExtake);
      this.setIntakeArmPower(IntakeArmSetPoints.kExtake);
      }, () ->{
        this.setIntakePower(0);
        this.setIntakeArmPower(0);
      }).withName("Outtaking");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //diaplaying the subsystem values, not sure why im using "getAppliedOutput" but it was in the example code so here we are
    SmartDashboard.putNumber("Intake/ Applied Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("IntakeArm/Applied Output", intakeArmMotor.getAppliedOutput());
  }
}
