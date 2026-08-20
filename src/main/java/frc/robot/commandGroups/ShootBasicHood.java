package frc.robot.commandGroups.ShootCommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootBasicHood extends ParallelCommandGroup {
  public ShootBasicHood(
      DoubleSupplier speed,
      double hoodAngle,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {

    addCommands(
        shooterSubsystem.shootWithHood(speed.getAsDouble(), hoodAngle),
        Commands.waitUntil(shooterSubsystem::isShooterAtSpeed)
            .andThen(hopperSubsystem.runHopperUntilInterruptedCommand()));
  }
}