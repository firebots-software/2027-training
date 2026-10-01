package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class SpinUpShooter extends Command {

    private PeterSubsystem peterSubsystem;

    public SpinUpShooter(PeterSubsystem peterSubsystem, boolean isAmp) {
        this.peterSubsystem = peterSubsystem;
        
        addRequirements(peterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      peterSubsystem.spinLeftShooter();
      peterSubsystem.spinRightShooter();
    }

     // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return peterSubsystem.isShooterReady();
  }

}
