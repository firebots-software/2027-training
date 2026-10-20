// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

package frc.robot.commandGroups;
 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Pooer.ShooterType;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class Intake extends SequentialCommandGroup {
    public Intake(PeterSubsystem peter, ArmSubsystem arm) {
       addCommands(arm.IntakePosition(),peter.runIntakeUntilDetection());
    }

}