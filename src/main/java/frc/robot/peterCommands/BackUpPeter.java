package frc.robot.peterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class BackUpPeter extends Command {

    private PeterSubsystem peter;

    public BackUpPeter(PeterSubsystem peter) {
        this.peter = peter;
    }

    @Override
    public void initialize() {
        peter.resetPreshooterPos();
    }

    @Override
    public void execute() {
        peter.reversePreshooter(1.25);
    }

    @Override
    public void end(boolean interrupted) {
        peter.stopPreshooter();
    }

    @Override
    public boolean isFinished() {
        return peter.isBackedUp(1.25);
    }

}
