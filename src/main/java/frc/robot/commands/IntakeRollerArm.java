package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRollerArm extends Command {
    private IntakeSubsystem intake;
    private DoubleSupplier armSpeed;
    private DoubleSupplier rollerSpeed;
    private DoubleSupplier position;
    private DoubleSupplier angle;

    public IntakeRollerArm(IntakeSubsystem intake, DoubleSupplier armSpeed, DoubleSupplier rollerSpeed,
            DoubleSupplier position, DoubleSupplier angle) {
        intake = new IntakeSubsystem();
        this.armSpeed = armSpeed;
        this.rollerSpeed = rollerSpeed;
        this.position = position;
        this.angle = angle;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake.setPosition(position.getAsDouble());
        intake.setArmSpeed(armSpeed.getAsDouble());
        intake.setRollerSpeed(rollerSpeed.getAsDouble());

    }
    @Override
    public void end() {
        intake.stopMotor();

    }
    @Override
    public boolean isFinished() {
        return intake.isAtTargetPositon();
    }
}