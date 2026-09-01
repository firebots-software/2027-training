package frc.robot.Commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;



public class IntakeRollersAndArm extends Command {
    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier speed;
    private DoubleSupplier angle;

    public IntakeRollersAndArm(IntakeSubsystem intakeSubsystem, DoubleSupplier speed, DoubleSupplier angle) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        this.angle = angle;
        addRequirements(intakeSubsystem);
    }


    @Override
    public void initialize() {}

    public void execute() {
        intakeSubsystem.setSpeed(speed.getAsDouble());
        intakeSubsystem.setPosition(angle.getAsDouble());
    }

    @Override
    public void end() {
        intakeSubsystem.stopRollers();
        intakeSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
