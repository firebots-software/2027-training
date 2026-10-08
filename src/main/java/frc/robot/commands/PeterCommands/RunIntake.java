package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class RunIntake extends Command {
    private PeterSubsystem peterSubsystem; 

    public RunIntake(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
        addRequirements(peterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        peterSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        peterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return !peterSubsystem.noNoteDetected();
    }
}