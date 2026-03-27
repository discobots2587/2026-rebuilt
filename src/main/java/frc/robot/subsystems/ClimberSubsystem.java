package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(
            ClimberSubsystemConstants.kClimberMotorCanId,
            SparkMax.MotorType.kBrushless
        );
        climberMotor.configure(
            Configs.ClimberSubsystem.climberMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    private void setClimberPower(double power) {
        climberMotor.set(power);
    }

    // Manual climb — held button
    public Command runClimbCommand() {
        return this.startEnd(
            () -> setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb),
            () -> setClimberPower(0)
        ).withName("Climbing");
    }

    // Manual descend — held button
    public Command runDescendCommand() {
        return this.startEnd(
            () -> setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kDescend),
            () -> setClimberPower(0)
        ).withName("Descending");
    }

    // Auto climb — runs for a fixed time then stops
    // Limit switches will cut power automatically if hit before timeout
    public Command autoClimberCommand() {
        return this.startEnd(
            () -> setClimberPower(ClimberSubsystemConstants.ClimberSetPoints.kClimb),
            () -> setClimberPower(0)
        )
        .withTimeout(2.0)  // was 2.0 at Space City
        .withName("AutoClimbing");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Motor Output", climberMotor.getAppliedOutput());
        SmartDashboard.putNumber("Climber/Encoder Position", climberMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Climber/Forward Limit", climberMotor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("Climber/Reverse Limit", climberMotor.getReverseLimitSwitch().isPressed());
    }
}