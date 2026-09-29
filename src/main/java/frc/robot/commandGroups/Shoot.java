package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// TODO: Sequence commands to shoot a note.
// You may want to add parameters for arm angle and shooting speed so that you can tune for the distance target.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(PeterSubsystem peter, ArmSubsystem arm) {
        addCommands(new ResetArm(arm),
                new ParallelCommandGroup(new SpinUpShooter(peter), new ArmToAngleCmd(arm, Constants.Arm.BUNDT_ANGLE)),
                new ArmToAngleCmd(arm, Constants.Arm.BUNDT_ANGLE));
    }
}