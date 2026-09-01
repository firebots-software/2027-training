package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;


public class IntakeRollersAndArm {
    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier speed;
    private DoubleSupplier angle;

    public IntakeRollersAndArm(IntakeSubsystem intakeSubsystem, DoubleSupplier speed, DoubleSupplier angle){
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        this.angle = angle;
        addRequirements(intakeSubsystem);

    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setSpeed(speed.getAsDouble());
  }

  @Override
  public void end() {
    intakeSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }


    
}
