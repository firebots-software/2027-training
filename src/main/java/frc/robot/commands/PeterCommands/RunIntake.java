package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class RunIntake extends Command {
    private PeterSubsystem peterSubsystem; 

    public RunIntake(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
    }
    
}