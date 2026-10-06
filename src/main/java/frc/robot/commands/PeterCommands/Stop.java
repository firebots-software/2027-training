package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class Stop extends Command {
    private PeterSubsystem peterSubsystem; 

    public Stop(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
    }
    
}