package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class BackupPeter extends Command {

  private final PeterSubsystem peter;

  public BackupPeter(PeterSubsystem peter) {
    this.peter = peter;
  }

  @Override
  public void initialize() {
    peter.resetPreshooter();
  }

  @Override
  public void execute() {
    peter.reversePreshooter(1.25);
  }

  @Override
  public void end(boolean interrupted) {
    peter.stopPreshooter();
  }

  @Override
  public boolean isFinished() {
    return peter.isBackedUp(1.25);
  }
} 
