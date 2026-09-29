package frc.robot.commandGroups;

// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.PeterCommands.BackupPeter;
import frc.robot.commands.PeterCommands.RunIntakeUntilDetection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Intake extends SequentialCommandGroup {
    public Intake(PeterSubsystem peter, ArmSubsystem arm) {
        addCommands(new ResetArm(arm),
                new RunIntakeUntilDetection(peter).deadlineFor(new ArmToAngleCmd(arm, Constants.Arm.INTAKE_ANGLE)),
                new ParallelCommandGroup(new ArmToAngleCmd(arm, Constants.Arm.DEFAULT_ARM_ANGLE),
                        new BackupPeter(peter)));
    }
}