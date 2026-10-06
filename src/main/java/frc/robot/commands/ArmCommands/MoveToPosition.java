package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToPosition extends Command {
    private ArmSubsystem armSubsystem; 

    public MoveToPosition(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem; 
    }
    
}
