package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmToAngle;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.peterCommands.runIntakeUntilDetected;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.peterCommands.BackUpPeter;

// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

public class Intake extends SequentialCommandGroup {
    public Intake(PeterSubsystem peter, ArmSubsystem arm) {
        addCommands(new ResetArm(arm),
        new runIntakeUntilDetected(peter).deadlineFor(new ArmToAngle(arm, Constants.Arm.INTAKE_ANGLE)),
        new ParallelCommandGroup(new ArmToAngle(arm, Constants.Arm.DEFAULT_ARM_ANGLE), new BackUpPeter(peter)));
    }


}
