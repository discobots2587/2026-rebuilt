// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  /** Creates a new ClimberSubsystem. */
  private final SparkMax climberMotor;
   
  public ClimberSubsystem() {
    // Initialize the climber motor
    climberMotor = new SparkMax(ClimberSubsystemConstants.kClimberMotorCanId, SparkMax.MotorType.kBrushless);

    //config climber motor
    climberMotor.configure(Configs.ClimberSubsystem.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setClimberPower(double power) {
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
   public Command autoClimberCommand(){
        // this cmmand needs a limit either software or hardware as during testing it came off the sprocket.
        return this.runOnce(() -> {
            final Timer m_time = new Timer();
            m_time.restart();
            ;
            while (!m_time.hasElapsed(2.0)){ //Was 2.0 in Space City
                setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb);;
                /**
                if (Timer.getMatchTime() > 3){
                    autoStopClimber();
                }
                    */
            }; 
            setClimberPower(0.0);
        
        });
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Motor Output", climberMotor.getAppliedOutput());
    
  }
}
