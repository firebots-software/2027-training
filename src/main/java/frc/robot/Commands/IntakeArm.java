package frc.robot.Commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;



public class IntakeArm extends Command {
    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier angle;

    public IntakekArm(IntakeSubsystem intakeSubsystem, DoubleSupplier angle) {
        this.intakeSubsystem = intakeSubsystem;
        this.angle = angle;
        addRequirements(intakeSubsystem);
    }


    @Override
    public void initialize() {}

    public void execute() {
        intakeSubsystem.setPosition(angle.getAsDouble());
    }

    @Override
    public void end() {
        intakeSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isArmAtPos();
    }
}
