package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBasicHood extends ParallelCommandGroup {
  public ShootBasicHood(
      double speed, double angle, ShooterSubsystem shooter, HopperSubsystem hopper) {
    addCommands(
        shooter.shootWithHood(speed, angle),
        Commands.waitUntil(shooter::isShooterAtSpeed)
            .andThen(hopper.runHopperUntilInterruptedCommand()));
  }
}