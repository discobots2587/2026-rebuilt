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
    private void setHoodMotorPower(double power){
    hoodMotor.set(power);
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

}
