package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class BackupPeter extends Command {
    private PeterSubsystem peterSubsystem; 

    public BackupPeter(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
        addRequirements(peterSubsystem);
    }
    
    @Override
    public void initialize() {
        peterSubsystem.resetPreshooter();
    }

    @Override
    public void execute() {
        peterSubsystem.reversePreshooterRotations(1.25);
    }

    @Override
    public void end(boolean interrupted) {
        peterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return peterSubsystem.isBackedUp(1.25);
    }
}
