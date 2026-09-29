package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends Command {

    private ArmSubsystem arm;

    public ResetArm(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.resetPosition();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
