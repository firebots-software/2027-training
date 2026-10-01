package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends Command {
  private ArmSubsystem armSubsystem;

  public ResetArm(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.reset();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}