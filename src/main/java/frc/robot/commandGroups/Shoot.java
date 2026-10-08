// TODO: Sequence commands to shoot a note.
// You may want to add parameters for arm angle and shooting speed so that you can tune for the distance target.

// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.MoveToPosition;
import frc.robot.commands.PeterCommands.ShootPeter;
import frc.robot.commands.PeterCommands.SpinShooterUp;
import frc.robot.commands.PeterCommands.Stop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(PeterSubsystem peter, ArmSubsystem arm) {
        addCommands(new MoveToPosition(arm, 30), new SpinShooterUp(peter), new ShootPeter(peter), new Stop(peter));
    }
}
