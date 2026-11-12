package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.SwerveSubsystem;
// * KEEP FOR WIN COMMAND TESTING
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Targeting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;



public class RobotContainer {
    public final VisionSubsystem rightCamera = new VisionSubsystem();

    private BooleanSupplier redside = RobotContainer::isRedAlliance;

  //   private Field2d field = new Field2d();
  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);

  private final CommandXboxController joystick = new CommandXboxController(0);
  
  public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    // Swerve
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY();
    DoubleSupplier leftRightFunction = () -> -joystick.getLeftX();
    DoubleSupplier rotationFunction = () -> -joystick.getRightX();
    DoubleSupplier speedFunction = () -> 1d;

    SwerveJoystickCommand swerveJoystickDefaultCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction,
            () -> true,
            joystick.leftTrigger()::getAsBoolean,
            () -> false,
            redside,
            drivetrain,
            () -> false,
            () -> false);

    joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.setDefaultCommand(swerveJoystickDefaultCommand);
  }

    public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isEmpty()
        ? false
        : DriverStation.getAlliance().get() == Alliance.Red;
  }
    
}