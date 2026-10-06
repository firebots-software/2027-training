package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToIntake extends Command {
    private ArmSubsystem armSubsystem; 

    public MoveToIntake(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem; 
    }
    
}
