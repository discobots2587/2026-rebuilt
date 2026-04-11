package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Auto command that adapts based on alliance color:
 * - Blue Alliance → Runs as "BROA"
 * - Red Alliance → Runs as "BROA" with auto mirroring enabled
 */
public class BROAAuto extends Command {
    private Command autoCommand;

    public BROAAuto() {
        // No requirements needed - PathPlannerAuto handles subsystem requirements
    }

    @Override
    public void initialize() {
        try {
            // Load the auto file - PathPlannerAuto handles mirroring automatically
            autoCommand = new PathPlannerAuto("BROA");
            autoCommand.initialize();
        } catch (Exception e) {
            System.out.println("Error loading BROA auto: " + e.getMessage());
            autoCommand = Commands.none();
        }
    }

    @Override
    public void execute() {
        if (autoCommand != null) {
            autoCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (autoCommand != null) {
            autoCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if (autoCommand != null) {
            return autoCommand.isFinished();
        }
        return true;
    }
}
