package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootBasicHood extends ParallelCommandGroup {
  public ShootBasicHood(
      DoubleSupplier hoodAngle, DoubleSupplier shooterVelocity, ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootWithHood(hoodAngle.getAsDouble(), shooterVelocity.getAsDouble()),
        Commands.waitUntil(shooterSubsystem::isShooterAtSpeed).andThen(hopperSubsystem.runHopperUntilInterruptedCommand()));
  }
}
