package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

public class SpindexerSubsystem extends SubsystemBase{
    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;
    private final SparkClosedLoopController spindexerController;
    private final RelativeEncoder spindexerEncoder;
    public SpindexerSubsystem() {
        spindexerMotor = new SparkMax(ShooterSubsystemConstants.kSpindexerCanID, SparkMax.MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId , SparkMax.MotorType.kBrushless);
        spindexerController = spindexerMotor.getClosedLoopController();
        spindexerEncoder = spindexerMotor.getEncoder();
        feederMotor.configure(
            Configs.ShooterSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        spindexerMotor.configure(
        Configs.ShooterSubsystem.spindexerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        spindexerEncoder.setPosition(0);

//   private double flywheelTargetVelocity = 0.0;
//   private boolean runSpindexer =  false;

    }
    private double spindexerTargetVelocity = 0.0;

    private void setSpindexerVelocity(double velocity) {
    // flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    spindexerController.setSetpoint(velocity, ControlType.kVoltage); //was duty cycle for percentage
    spindexerTargetVelocity = velocity;
  }

    private void setSpindexerPower(double power){
        spindexerMotor.set(power);
    }
    private void setFeederPower(double power){
        feederMotor.set(power);
    }
    public Command runSpindexerCommand(boolean spindexer_only){
        return this.startEnd(
            () -> {
                double power = ShooterSubsystemConstants.SpindexerSetpoints.kSpindex;
                if(spindexer_only){
                    power *= -0.3;
                }else{
                    this.setFeederPower(ShooterSubsystemConstants.FeederSetpoints.kFeed);
                }
                this.setSpindexerVelocity(ShooterSubsystemConstants.SpindexerSetpoints.kSpindexVolt);
            }, () -> {
                this.setSpindexerPower(0);
                this.setFeederPower(0);
            }).withName("Spindexing");
    }

     @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());

    SmartDashboard.putNumber("Shooter | Spindexer | Voltage", spindexerMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter | Spindexer | Output", spindexerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Spindexer | Current", spindexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Spindexer | Velocity", spindexerEncoder.getVelocity());


   }



    
    
}
