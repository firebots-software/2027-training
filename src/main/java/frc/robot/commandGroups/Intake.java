package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PeterCommands.BackupPeter;
import frc.robot.commands.PeterCommands.RunIntakeUntilDetection;
import frc.robot.commands.ArmCommands.MoveArmToAngle;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.Constants;

// TODO: Sequence commands to intake a note.
// You can reference your work from last meeting to see what is necessary to correctly ready a note for this robot.package frc.robot.commandGroups;


public class Intake extends SequentialCommandGroup {
  
  public Intake(PeterSubsystem peter, ArmSubsystem arm) {
    // addCommands() takes a comma-separated list of commands and runs them in this case sequentially..
    addCommands(
        
        // Step 1: Start by resetting the arm so we know it's in a good state.
        new ResetArm(arm),
        
        // Step 2: The Core Intake Phase
        // run the intake until the note is detected, the arm needs to be 
        // down in the intake position while this happens. 
        // hint: use .deadlineFor()  
        // on it, which sets the intake running as the event that ends the arm staying down
        new RunIntakeUntilDetection(peter).deadlineFor(new MoveArmToAngle(() -> Constants.Arm.INTAKE_ANGLE, arm)),
        
        // Step 3: Do 3 things at the exact same time
        // Once we have the note, we need to stow the arm, backup the note, and rumble the controller.
        // Hint: Create a new ParallelCommandGroup(...) here and pass it three commands:
        // Return the arm to neutral (tolerance 1)
        // backup the intake (peter)
        // You can use this command to rumble the controller: Rumble.withNoBlock(joystick, 0.25, 0.5, 0)
        new ParallelCommandGroup(new MoveArmToAngle(() -> Constants.Arm.DEFAULT_ARM_ANGLE, arm), new BackupPeter(peter))
        
    );
  }
}