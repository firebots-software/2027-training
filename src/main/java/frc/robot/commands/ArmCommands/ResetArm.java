package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends Command {
  private final ArmSubsystem armSubsystem;

  public ResetArm(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.reset();
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.isInitialized();
  }
}
