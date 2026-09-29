package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

public class SpinUpShooter extends Command {

    private PeterSubsystem peter;

    public SpinUpShooter(PeterSubsystem peter) {
        this.peter = peter;
        addRequirements(peter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        peter.runShooter(Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS, Constants.Pooer.SHOOTER.SHOOTER_2.SPEED_RPS);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return peter.isShooterAtTarget();
    }
}
