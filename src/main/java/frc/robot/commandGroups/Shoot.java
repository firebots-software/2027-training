package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.PeterCommands.FeedNote;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Shoot extends SequentialCommandGroup {
  private static final double ARM_TOLERANCE_DEGREES = 1;

  public Shoot(PeterSubsystem peter, ArmSubsystem arm) {
    this(peter, arm, Constants.Arm.BUNDT_ANGLE,
        Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS,
        Constants.Pooer.SHOOTER.SHOOTER_2.SPEED_RPS);
  }

  // Wheel speeds are mechanism rotations per second, not RPM.
  public Shoot(PeterSubsystem peter, ArmSubsystem arm, double targetAngleDegrees,
      double shooter1RPS, double shooter2RPS) {
    addCommands(
        new SequentialCommandGroup(
            new ResetArm(arm),
            new ParallelCommandGroup(
                new ArmToAngleCmd(targetAngleDegrees, arm).withTolerance(ARM_TOLERANCE_DEGREES),
                new SpinUpShooter(peter, shooter1RPS, shooter2RPS)),
            new FeedNote(peter, shooter1RPS, shooter2RPS)
                .onlyIf(() -> arm.atTarget(ARM_TOLERANCE_DEGREES) && peter.isShooterReady()),
            peter.runOnce(peter::stopShooter),
            ArmToAngleCmd.toNeutral(arm).withTolerance(ARM_TOLERANCE_DEGREES))
            .finallyDo(interrupted -> {
              peter.stopIntake();
              peter.stopPreShooterMotor();
              peter.stopShooter();
              arm.setTargetDegrees(Constants.Arm.DEFAULT_ARM_ANGLE);
            }));
  }
}
