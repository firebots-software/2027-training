// TODO: Sequence commands to shoot a note.
// You may want to add parameters for arm angle and shooting speed so that you can tune for the distance target.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.PeterSubsystem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(PeterSubsystem peter) {
        addCommands(new SpinUpShooter(peter));
    }
}