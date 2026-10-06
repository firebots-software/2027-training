package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class ShootPeter extends Command {
    private PeterSubsystem peterSubsystem; 

    public ShootPeter(PeterSubsystem peterSubsystem) {
        this.peterSubsystem = peterSubsystem; 
    }
    
}