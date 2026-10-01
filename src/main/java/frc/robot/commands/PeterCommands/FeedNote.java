package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class FeedNote extends Command {
  private static final double TIMEOUT_SECONDS = 2;
  private final PeterSubsystem peter;
  private final double shooter1RPS;
  private final double shooter2RPS;
  private final Timer timer = new Timer();
  private boolean seenNote;

  public FeedNote(PeterSubsystem peter, double shooter1RPS, double shooter2RPS) {
    this.peter = peter;
    this.shooter1RPS = shooter1RPS;
    this.shooter2RPS = shooter2RPS;
    addRequirements(peter);
  }

  @Override
  public void initialize() {
    seenNote = peter.notePresent();
    timer.restart();
    peter.stopIntake();
    peter.stopPreShooterMotor();
  }

  @Override
  public void execute() {
    peter.runShooter(shooter1RPS, shooter2RPS);
    peter.spinUpPreShooterVelocity();
    if (peter.notePresent()) {
      seenNote = true;
    }
  }

  @Override
  public boolean isFinished() {
    return (seenNote && !peter.notePresent()) || timer.hasElapsed(TIMEOUT_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    peter.stopIntake();
    peter.stopPreShooterMotor();
    peter.stopShooter();
  }
}
