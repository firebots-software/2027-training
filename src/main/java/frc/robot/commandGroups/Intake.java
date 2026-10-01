// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.commands.ArmCommands.*;

public class Intake extends SequentialCommandGroup {
  public Intake(PeterSubsystem peter, ArmSubsystem arm, JoystickSubsystem joystick) {
    addCommands(
        new ResetArm(arm),
        new RunIntakeUntilDetection(peter)
            .deadlineWith(ArmToAngleCmd.toIntake(arm).withReturnToRest(EndBehavior.RETURN_ALWAYS)),
        new ParallelCommandGroup(
            ArmToAngleCmd.toNeutral(arm).withTolerance(1),
            new BackupPeter(peter),
            Rumble.withNoBlock(joystick, 0.25, 0.5, 0)));
  }
}