package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class SpinShooterUpTo extends Command {
    private PeterSubsystem peterSubsystem; 

    public SpinShooterUpTo(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
    }
    
}