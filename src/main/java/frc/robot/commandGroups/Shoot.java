package frc.robot.commandGroups;


import java.security.interfaces.RSAKey;

import javax.crypto.EncryptedPrivateKeyInfo;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmToAngle;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.peterCommands.spinUpShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.Constants;


// TODO: Sequence commands to shoot a note. 
// You may want to add parameters for arm angle and shooting speed so that you can tune for the distance target.

public class Shoot extends SequentialCommandGroup {
    public Shoot(
        PeterSubsystem peterSubystem,
        ArmSubsystem armSubsystem,
        JoystickSubsystem joystickSubsystem) {
            addCommands(new ResetArm(armSubsystem),
            new ParallelCommandGroup(
                new spinUpShooter(peterSubystem, false),
                new ArmToAngle(armSubsystem, Constants.Arm.INTAKE_ANGLE).withTolerance(1)
            ));
           
        }
}