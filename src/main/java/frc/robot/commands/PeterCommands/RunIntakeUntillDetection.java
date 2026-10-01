

package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class RunIntakeUntillDetection extends Command {
  private PeterSubsystem peterSubsystem;

  public RunIntakeUntillDetection(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    peterSubsystem.spinUpIntake();
    peterSubsystem.spinUpPreShooterVelocity();
  }

  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopIntake();
    peterSubsystem.stopPreShooterMotor();
  }

  @Override
  public boolean isFinished() {
    return peterSubsystem.notePresent();
  }
}