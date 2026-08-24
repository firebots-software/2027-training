package frc.robot.commandGroups.ShootCommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBasicHood extends ParallelCommandGroup {
    public ShootBasicHood(double speed, double hoodAngle, ShooterSubsystem shooterSubsystem) {
        addCommands(
            shooterSubsystem.shootWithHood(speed, hoodAngle),
            Commands.waitUntil(shooterSubsystem::isShooterAtSpeed)
        );
    }
}
