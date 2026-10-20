// TODO: Sequence commands to shoot a note.
// You may want to add parameters for arm angle and shooting speed so that you can tune for the distance target.

package frc.robot.commandGroups;
 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Pooer.ShooterType;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(PeterSubsystem peter, ArmSubsystem arm){
        addCommands(arm.ArmPosition(), peter.runShooter());
    }
}