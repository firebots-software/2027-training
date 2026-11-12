package frc.robot.opModes.autos;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.opModes.BaseOpMode;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;

@Autonomous(name = "Dynamic", group = "Custom")
public class SelectableAuto extends BaseOpMode {

  private Command m_autoCommand;

  public SelectableAuto(Robot robot) {
    super(robot);
    m_autoCommand = m_robotContainer.getAutonomousCommand(Constants.Swerve.Auto.AutoList.DYNAMIC_AUTO);
  }

  @Override
  public void start() {
    super.start();
     m_autoCommand = m_robotContainer.getAutonomousCommand(Constants.Swerve.Auto.AutoList.DYNAMIC_AUTO);
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
