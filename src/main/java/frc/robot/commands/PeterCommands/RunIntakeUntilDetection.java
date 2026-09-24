package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class RunIntakeUntilDetection extends Command {
  
  // 1. Declare your subsystem dependency here
  private final PeterSubsystem peterSubsystem;

  public RunIntakeUntilDetection(PeterSubsystem subsystem) {
    this.peterSubsystem = subsystem;
    
    // 2. Add the subsystem requirements so no other command tries to use it at the same time
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 3. For our implementation, we actually leave this blank
    // We will handle telling the motors to spin in the execute() method instead.
  }

  // Called every time the scheduler runs while the command is scheduled (approx every 20ms).
  @Override
  public void execute() {
    // 4. What should the motors be doing while we wait for the note? Look in petersubsystem.java to find the methods
    // Hint: Call the subsystem methods to spin up the intake 
    // preshooter to pull the note in
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // 5. How does the command know when to stop?
    // Hint: Call the boolean function you created in the subsystem that checks the sensor.
    return false; // Replace this!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // 6. What should the motors do once the note is detected (or if the driver lets go of the button)?
    // Hint: Call the subsystem methods to stop the intake and preshooter!
    
  }
}