package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPos extends Command {
  private ArmSubsystem armSubsystem;
  private double pos;

  public ArmToPos(ArmSubsystem armSubsystem, double deg) {
    this.armSubsystem = armSubsystem;
    this.pos = deg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setTargetDeg(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.atTarget(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}