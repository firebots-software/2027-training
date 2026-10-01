package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class BackupPeter extends Command {
  private final PeterSubsystem peter;
  private boolean positionReset;

  public BackupPeter(PeterSubsystem peter) {
    this.peter = peter;
    addRequirements(peter);
  }

  @Override
  public void initialize() {
    peter.stopPreShooterMotor();
    positionReset = peter.resetPreshooterPos();
  }

  @Override
  public void execute() {
    if (!positionReset) {
      positionReset = peter.resetPreshooterPos();
      return;
    }
    peter.reversePreshooter(1.25);
  }

  @Override
  public boolean isFinished() {
    return positionReset && peter.isBackedUp(1.25);
  }

  @Override
  public void end(boolean interrupted) {
    peter.stopPreshooter();
  }
}
