package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToAngleCmd extends Command {
  private final ArmSubsystem arm;
  private final double angle;
  private double tolerance = -1;

  public ArmToAngleCmd(double angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setTargetDegrees(angle);
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(tolerance);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.setTargetDegrees(Constants.Arm.DEFAULT_ARM_ANGLE);
    }
  }

  public ArmToAngleCmd withTolerance(double degrees) {
    tolerance = degrees;
    return this;
  }

  public static ArmToAngleCmd toIntake(ArmSubsystem arm) {
    return new ArmToAngleCmd(Constants.Arm.INTAKE_ANGLE, arm);
  }

  public static ArmToAngleCmd toNeutral(ArmSubsystem arm) {
    return new ArmToAngleCmd(Constants.Arm.DEFAULT_ARM_ANGLE, arm);
  }
}
