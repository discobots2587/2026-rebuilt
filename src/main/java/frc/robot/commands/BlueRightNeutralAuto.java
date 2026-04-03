package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Auto command that adapts based on alliance color:
 * - Blue Alliance → Runs as "blueRightNeutral"
 * - Red Alliance → Runs as "blueRightNeutral" with auto mirroring enabled
 */
public class BlueRightNeutralAuto extends Command {
    private Command autoCommand;

    public BlueRightNeutralAuto() {
        // No requirements needed - PathPlannerAuto handles subsystem requirements
    }

    @Override
    public void initialize() {
        try {
            // Load the auto file - PathPlannerAuto handles mirroring automatically
            autoCommand = new PathPlannerAuto("blueRightNeutral");
            autoCommand.initialize();
        } catch (Exception e) {
            System.out.println("Error loading blueRightNeutral auto: " + e.getMessage());
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
