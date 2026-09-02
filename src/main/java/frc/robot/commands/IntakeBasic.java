package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBasic extends Command {
    private DoubleSupplier armSpeed;
    private DoubleSupplier angle;
    private DoubleSupplier rollerSpeed;
    private IntakeSubsystem intakeSubsystem;

    public IntakeBasic(IntakeSubsystem intakeSubsystem, DoubleSupplier armSpeed, DoubleSupplier angle, DoubleSupplier rollerSpeed){
        this.intakeSubsystem = intakeSubsystem;
        this.armSpeed = armSpeed;
        this.angle = angle;
        this.rollerSpeed = rollerSpeed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.setArmSpeed(armSpeed.getAsDouble());
        intakeSubsystem.setRollerSpeed(rollerSpeed.getAsDouble());
        intakeSubsystem.setAngle(angle.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
         intakeSubsystem.stopRollers();
    }

    @Override
    public boolean isFinished(){
        return intakeSubsystem.areRollersAtSpeed();
    }
}