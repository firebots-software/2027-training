package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootBasicHood extends ParallelCommandGroup {
  public ShootBasicHood(
      DoubleSupplier hoodAngle, DoubleSupplier shooterVelocity, ShooterSubsystem shooterSubsystem) {
    addCommands(
        shooterSubsystem.shootWithHood(hoodAngle.getAsDouble(), shooterVelocity.getAsDouble()));
  }
}
