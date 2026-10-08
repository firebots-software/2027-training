package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

public class RunIntakeUntilDetection extends Command {

    private PeterSubsystem peter;

    public RunIntakeUntilDetection(PeterSubsystem peter) {
        this.peter = peter;
        addRequirements(peter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        peter.runIntake(Constants.Pooer.SHOOTER.INTAKE.SPEED_RPS);
        peter.runPreShooter(Constants.Pooer.SHOOTER.PRESHOOTER.SPEED_RPS);
    }

    @Override
    public void end(boolean interrupted) {
        peter.stopIntake();
        peter.stopPreshooter();
    }

    @Override
    public boolean isFinished() {
        return peter.isNotePresent();
    }
}
