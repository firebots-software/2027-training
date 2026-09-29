// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.PeterCommands.IntakeUntilDetection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Intake extends SequentialCommandGroup {
    public Intake(PeterSubsystem peter, ArmSubsystem arm, JoystickSubsystem joystick) {
        addCommands(
            // Step 1: Start by resetting the arm so we know it's in a good state.
            new ResetArm(arm)
        
            // Step 2: The Core Intake Phase
            // run the intake until the note is detected, the arm needs to be 
            // down in the intake position while this happens. 
            // hint: use .deadlineFor()  
            // on it, which sets the intake running as the event that ends the arm staying down
            new IntakeUntilDetection(peter)
        
            // Step 3: Do 3 things at the exact same time
            // Once we have the note, we need to stow the arm, backup the note, and rumble the controller.
            // Hint: Create a new ParallelCommandGroup(...) here and pass it three commands:
            // Return the arm to neutral (tolerance 1)
            // backup the intake (peter)
            // You can use this command to rumble the controller: Rumble.withNoBlock(joystick, 0.25, 0.5, 0)
        );
    }
}