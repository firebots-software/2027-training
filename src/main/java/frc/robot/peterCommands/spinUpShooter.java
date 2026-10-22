package frc.robot.peterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class spinUpShooter extends Command {
    PeterSubsystem peterSubsystem;
    private boolean isAmp;
    

    public spinUpShooter(PeterSubsystem peterSubsystem, boolean isAmp) {
        this.peterSubsystem = peterSubsystem;
        this.isAmp = isAmp;

        addRequirements(peterSubsystem);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {
    if (isAmp) {
      peterSubsystem.spinLeftShooterForAmp();
      peterSubsystem.spinRightShooterForAmp();
    } else {
      peterSubsystem.spinLeftShooter();
      peterSubsystem.spinRightShooter();
    }
    }
    
    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (isAmp) {
            return peterSubsystem.isShooterReadyAmp();
        } else {
            return peterSubsystem.isShooterReady();
        }

    }




}