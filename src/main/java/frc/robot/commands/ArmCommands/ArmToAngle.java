package frc.robot.commands.ArmCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


import edu.wpi.first.wpilibj2.command.Command;


public class ArmToAngle extends Command{
    
    private ArmSubsystem arm;
    private double angle;
    public double endToleranceDegrees = -1;

    public ArmToAngle(ArmSubsystem arm, double angle) {
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
        return arm.atTarget(endToleranceDegrees);
    }

    @Override
    public void end(boolean interrupted) {
    }

    public ArmToAngle withTolerance(double degrees) {
        this.endToleranceDegrees = degrees;
        return this;
    }

}
