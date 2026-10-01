package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmToPos;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.Constants;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Shoot extends SequentialCommandGroup {
  public Shoot(
      PeterSubsystem peterSubsystem,
      ArmSubsystem armSubsystem,
      JoystickSubsystem joystickSubsystem) {
    addCommands(
        new ResetArm(armSubsystem),
        new ParallelCommandGroup(
            new SpinUpShooter(peterSubsystem, false),
            new ArmToPos(armSubsystem, Constants.Arm.BUNDT_ANGLE)),
        new ArmToPos(armSubsystem, Constants.Arm.BUNDT_ANGLE));
  }
}