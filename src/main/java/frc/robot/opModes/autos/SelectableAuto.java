package frc.robot.opModes.autos;

import frc.robot.Constants.Swerve.Auto.AutoList;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.opModes.BaseOpMode;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.tunable.Selectable;

import choreo.auto.AutoRoutine;

@Autonomous(name = "Selectable Auto", group = "Custom3")
public class SelectableAuto extends BaseOpMode {
  private Command m_autoCommand;
  private Selectable<String> intakeChooser;
  private Selectable<String> shootChooser;
  private Selectable<AutoRoutine> autoChooser2;

  public SelectableAuto(Robot robot) {
    super(robot);
    m_autoCommand = m_robotContainer.getAutonomousCommand(AutoList.SELECTABLE_AUTO);
  }

  @Override
  public void start() {
    super.start();
    if (m_autoCommand != null) {
      CommandScheduler.getInstance().schedule(m_autoCommand);
    }
  }

  @Override
  public void end() {
    if (m_autoCommand != null) {
      CommandScheduler.getInstance().cancel(m_autoCommand);
    }
  }
}
