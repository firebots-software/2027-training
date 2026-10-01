

package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class SpinUpShooter extends Command {
  private PeterSubsystem peterSubsystem;

  public SpinUpShooter(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    peterSubsystem.runShooter();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return peterSubsystem.isShooterReady();
  }
}