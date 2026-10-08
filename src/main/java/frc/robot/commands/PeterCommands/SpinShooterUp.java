package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class SpinShooterUp extends Command {
    private PeterSubsystem peterSubsystem; 

    public SpinShooterUp(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
        addRequirements(peterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        peterSubsystem.spinShooterUp();
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