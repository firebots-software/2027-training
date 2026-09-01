import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeBasic extends Command {
    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier armAngle;
    private DoubleSupplier rollerSpeed;


    public IntakeBasic (IntakeSubsystem intakeSubsystem, DoubleSupplier armAngle, DoubleSupplier rollerSpeed) {
        intakeSubsystem = new IntakeSubsystem ();
        this.armAngle = armAngle;
        this.rollerSpeed = rollerSpeed;
    }

    public @Override
    public void execute() {
        intakeSubsystem.setSpeed(rollerSpeed.getAsDouble());
        intakeSubsystem.setPosition(armAngle.getAsDouble());
    }

    public @Override
    public void end() {
        intakeSubsystem.stopMotor();
    }

    public @Override
    public boolean isFinished() {
        return intakeSubsystem.getArmAngle() == armAngle;
    }
    
}