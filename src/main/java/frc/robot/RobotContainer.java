package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// * KEEP FOR WIN COMMAND TESTING
import frc.robot.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;



public class RobotContainer {
    public final VisionSubsystem rightCamera = new VisionSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

    }
    
}