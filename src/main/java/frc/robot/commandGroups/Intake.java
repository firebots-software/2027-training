package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.PeterCommands.BackupPeter;
import frc.robot.commands.PeterCommands.RunIntakeUntillDetection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Intake extends SequentialCommandGroup {
  public Intake(PeterSubsystem peter, ArmSubsystem arm) {
    addCommands(
        new ResetArm(arm),
        new RunIntakeUntillDetection(peter)
            .deadlineFor(ArmToAngleCmd.toIntake(arm)),
        new ParallelCommandGroup(
            ArmToAngleCmd.toNeutral(arm).withTolerance(1),
            new BackupPeter(peter)));
  }
}
