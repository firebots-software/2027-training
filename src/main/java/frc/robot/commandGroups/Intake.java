// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.ArmCommands.MoveToIntake;
import frc.robot.commands.PeterCommands.RunIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Intake extends SequentialCommandGroup {
    public Intake(PeterSubsystem peter, ArmSubsystem arm) {
        addCommands(new MoveToIntake(arm), new RunIntake(peter));
    }
}