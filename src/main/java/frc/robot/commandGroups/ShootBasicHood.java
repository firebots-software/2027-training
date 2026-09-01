package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShootBasicHood extends ParallelCommandGroup {
  public ShootBasicHood(
      DoubleSupplier speed,
      DoubleSupplier angle,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootWithHood(speed.getAsDouble(), angle.getAsDouble()),
        Commands
        .waitUntil(() -> {return shooterSubsystem.isShooterAtSpeed(speed.getAsDouble()); })
            .andThen(hopperSubsystem.runHopperUntilInterruptedCommand()));
  }
}