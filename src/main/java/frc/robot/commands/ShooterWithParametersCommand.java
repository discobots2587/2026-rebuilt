package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;

public class ShooterWithParametersCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final HoodSubsystem m_hood;
    private final double flywheelSpeedPercent;
    private final double hoodAngle;

    public ShooterWithParametersCommand(ShooterSubsystem shooter, HoodSubsystem hood, 
                                       double flywheelSpeedPercent, double hoodAngle) {
        m_shooter = shooter;
        m_hood = hood;
        this.flywheelSpeedPercent = flywheelSpeedPercent;
        this.hoodAngle = hoodAngle;
        
        addRequirements(m_shooter, m_hood);
    }

    @Override
    public void initialize() {
        m_shooter.setFlywheelVelocity(FlywheelSetpoints.kShootVoltage * flywheelSpeedPercent);
        m_hood.moveHoodToAngle(hoodAngle);
    }

    @Override
    public void execute() {
        // Command runs while parameters are applied
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setFlywheelVelocity(0.0);
        m_hood.setHoodMotorPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}