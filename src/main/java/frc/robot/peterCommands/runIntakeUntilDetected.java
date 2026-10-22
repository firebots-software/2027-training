package frc.robot.peterCommands;

import java.security.AlgorithmParameterGenerator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

public class runIntakeUntilDetected extends Command {
    private final PeterSubsystem peterSubsystem;

    public runIntakeUntilDetected(PeterSubsystem subsystem) {
        this.peterSubsystem = subsystem;

        addRequirements(subsystem);
    }


    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        peterSubsystem.runIntake(Constants.Pooer.SHOOTER.INTAKE.SPEED_RPS);
        peterSubsystem.runPreShooter(Constants.Pooer.SHOOTER.PRESHOOTER.SPEED_RPS);


    }

    @Override
    public boolean isFinished() {
        return peterSubsystem.isNoteThere();
    }
    
    @Override
    public void end(boolean interrupted) {
        peterSubsystem.stopIntake();
        peterSubsystem.stopPreshooter();

    }
}
