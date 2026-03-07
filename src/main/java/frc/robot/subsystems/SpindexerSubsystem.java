package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

public class SpindexerSubsystem extends SubsystemBase{
    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;
    public SpindexerSubsystem() {
        spindexerMotor = new SparkMax(ShooterSubsystemConstants.kSpindexerCanID, SparkMax.MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId , SparkMax.MotorType.kBrushless);
        feederMotor.configure(
            Configs.ShooterSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        spindexerMotor.configure(
        Configs.ShooterSubsystem.spindexerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    }

    private void setSpindexerPower(double power){
        spindexerMotor.set(power);
    }
    private void setFeederPower(double power){
        feederMotor.set(power);
    }
    public Command runSpindexerCommand(){
        return this.startEnd(
            () -> {
                this.setSpindexerPower(ShooterSubsystemConstants.SpindexerSetpoints.kSpindex);
                this.setFeederPower(ShooterSubsystemConstants.FeederSetpoints.kFeed);
            }, () -> {
                this.setSpindexerPower(0);
                this.setFeederPower(0);
            }).withName("Spindexing");
    }
     @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());
  }



    
    
}
