package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;


public class ShootNoWarm extends Command {
    private PeterSubsystem peterSubsystem;


  public ShootNoWarm(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinLeftShooter();
    peterSubsystem.spinRightShooter();
    peterSubsystem.spinUpPreshooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopShooter();
    peterSubsystem.stopPreshooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}