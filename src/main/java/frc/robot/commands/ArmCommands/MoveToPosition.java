package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToPosition extends Command {
    private ArmSubsystem armSubsystem; 
    private double position;

    public MoveToPosition(ArmSubsystem armSubsystem, double position) {
        this.armSubsystem = armSubsystem; 
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        armSubsystem.moveToPosition(position);
    } 

    @Override 
    public void end(boolean interrupted) {
        armSubsystem.goToNeutral();
    }

    @Override
    public boolean isFinished() {
        return (armSubsystem.getAbsolutePosition() == position);
    }

}