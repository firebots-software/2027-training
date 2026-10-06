package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToPosition extends Command {
    private ArmSubsystem armSubsystem; 

    public MoveToPosition(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem; 
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        armSubsystem.moveToIntake();
    } 

    @Override 
    public void end(boolean interrupted) {
        armSubsystem.goToNeutral();
    }

}