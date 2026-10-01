package frc.robot.commands.PeterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

public class BackupPeter extends Command {
    // 1. Declare your subsystem dependency here

    private final PeterSubsystem peterSubsystem;

    public BackupPeter(PeterSubsystem subsystem) {
        this.peterSubsystem = subsystem;
    
        // 2. Add the subsystem requirements so no other command tries to use it at the same time
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        peterSubsystem.resetPreshooter();
    }

    // Called every time the scheduler runs while the command is scheduled (approx every 20ms).
    @Override
    public void execute() {
        peterSubsystem.reversePreshooterRotations(1.25);
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return peterSubsystem.isBackedUp(1.25);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        peterSubsystem.stopPreshooter();
    }
}