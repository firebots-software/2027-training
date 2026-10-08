package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class BackupPeter extends Command {

    private PeterSubsystem peter;
    private boolean posReset;

    public BackupPeter(PeterSubsystem peter) {
        this.peter = peter;
        addRequirements(peter);
    }

    @Override
    public void initialize() {
        peter.stopPreshooter();
        posReset = peter.resetPreshooterPos();
    }

    @Override
    public void execute() {
        if (!posReset) {
            posReset = peter.resetPreshooterPos();
            return;
        }
        peter.reversePreshooter(1.25);
    }

    @Override
    public void end(boolean interrupted) {
        peter.stopPreshooter();
    }

    @Override
    public boolean isFinished() {
        return posReset && peter.isBackedUp(1.25);
    }

}
