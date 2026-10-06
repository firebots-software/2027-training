package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.MoveArmToAngle;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.PeterCommands.ShootNoWarm;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;


public class Shoot extends SequentialCommandGroup {
    public Shoot(PeterSubsystem peter, ArmSubsystem arm) {
        addCommands(
            new ResetArm(arm),
            new ParallelCommandGroup(
                new SpinUpShooter(peter, false),
                MoveArmToAngle.toBundt(arm).withTolerance(1)
            )
        );
    }
}