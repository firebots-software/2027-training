package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;
import frc.robot.Constants;


public class MoveArmToAngle extends Command {

  private final ArmSubsystem arm;
  private final Supplier<Double> angle;
  private double endToleranceDegrees = -1;
  private EndBehavior returnToRest = EndBehavior.RETURN_IF_INTERRUPTED;

  public enum EndBehavior {
    STAY,
    RETURN_ALWAYS,
    RETURN_IF_INTERRUPTED
  }

  public MoveArmToAngle(Supplier<Double> angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    arm.setTargetDegrees(angle.get());
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(endToleranceDegrees);
  }

  @Override
  public void end(boolean interrupted) {}

  public static MoveArmToAngle toBundt (ArmSubsystem arm) {
    return new MoveArmToAngle(() -> Constants.Arm.BUNDT_ANGLE, arm);
  } 

  public MoveArmToAngle withTolerance(double degrees) {
    this.endToleranceDegrees = degrees;
    return this;
  }
} 