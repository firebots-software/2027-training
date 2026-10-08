package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToAngleCmd extends Command {

    private ArmSubsystem arm;
    private double angle;
    private double tol = -1;

    public ArmToAngleCmd(ArmSubsystem arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setTargetDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget(tol);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setTargetDegrees(Constants.Arm.DEFAULT_ARM_ANGLE);
    }

    public ArmToAngleCmd withTol(double tol) {
        this.tol = tol;
        return this;
    }
}
