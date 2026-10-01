

package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

public class SpinUpShooter extends Command {
  private final PeterSubsystem peterSubsystem;
  private final double shooter1RPS;
  private final double shooter2RPS;

  public SpinUpShooter(PeterSubsystem peterSubsystem) {
    this(peterSubsystem, Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS,
        Constants.Pooer.SHOOTER.SHOOTER_2.SPEED_RPS);
  }

  public SpinUpShooter(PeterSubsystem peterSubsystem, double shooter1RPS, double shooter2RPS) {
    this.peterSubsystem = peterSubsystem;
    this.shooter1RPS = shooter1RPS;
    this.shooter2RPS = shooter2RPS;
    addRequirements(peterSubsystem);
  }

  @Override
  public void initialize() {
    peterSubsystem.stopIntake();
    peterSubsystem.stopPreShooterMotor();
  }

  @Override
  public void execute() {
    peterSubsystem.runShooter(shooter1RPS, shooter2RPS);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      peterSubsystem.stopShooter();
    }
    // Keep wheel speed after normal completion for the next feed step.
  }

  @Override
  public boolean isFinished() {
    return peterSubsystem.isShooterReady();
  }
}
