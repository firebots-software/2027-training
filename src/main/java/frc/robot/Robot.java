package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import frc.robot.util.LoggedTalonFX;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.OpModeRobot; // The correct 2027 OpMode entry point
import org.wpilib.system.RobotController;
import org.wpilib.tunable.Selectable;

public class Robot extends OpModeRobot { //
  private final RobotContainer m_robotContainer;
  private Selectable<String> intakeChooser;
  private Selectable<String> shootChooser;
  private String theAuto;

  public Robot() {
    // Global setup inside the constructor
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withLogExtras(false).withNtTunables(true));
    RobotController.setBrownoutVoltages(6.0, 6.75);

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.visionPeriodic();
    m_robotContainer.elasticLogging();
    CommandScheduler.getInstance().run();
    LoggedTalonFX.periodic_static();
  }

  public RobotContainer getContainer() {
    return m_robotContainer;
  }
}
