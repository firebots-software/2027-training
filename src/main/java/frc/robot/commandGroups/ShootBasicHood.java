package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBasicHood extends ParallelCommandGroup {
  public ShootBasicHood(
      double shooterSpeed,
      double hoodAngle,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootWithHood(shooterSpeed, hoodAngle),
        Commands.waitUntil(shooterSubsystem::isShooterAtSpeed)
            .andThen(hopperSubsystem.runHopperUntilInterruptedCommand()));
  }
}
