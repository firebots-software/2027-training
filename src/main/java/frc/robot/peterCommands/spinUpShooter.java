package frc.robot.peterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        peterSubsystem.runShooter(Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS, Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS);
        peterSubsystem.runPreShooter(Constants.Pooer.SHOOTER.PRESHOOTER.SPEED_RPS);
    }
    
    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return peterSubsystem.isShooterAtTarget();

    }




}