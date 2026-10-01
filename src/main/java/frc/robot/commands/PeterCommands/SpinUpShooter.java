package frc.robot.commands.PeterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

public class SpinUpShooter extends Command {
    // 1. Declare your subsystem dependency here

    private final PeterSubsystem peterSubsystem;
    public boolean isAmp;

    public SpinUpShooter(PeterSubsystem subsystem, boolean isAmp) {
        this.peterSubsystem = subsystem;
        this.isAmp = isAmp;
    
        // 2. Add the subsystem requirements so no other command tries to use it at the same time
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled (approx every 20ms).
    @Override
    public void execute() {
        peterSubsystem.shootAtRPM(Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS, Constants.Pooer.SHOOTER.SHOOTER_2.SPEED_RPS);
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return peterSubsystem.isShooterReady();
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
}