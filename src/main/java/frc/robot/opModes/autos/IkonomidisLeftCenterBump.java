package frc.robot.opModes.autos;

import frc.robot.Constants.Swerve.Auto.AutoList;
import frc.robot.Robot;
import frc.robot.opModes.BaseOpMode;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;

@Autonomous(name = "Aligned Center Left Bump", group = "Custom")
public class IkonomidisLeftCenterBump extends BaseOpMode {
  private Command m_autoCommand;

  public IkonomidisLeftCenterBump(Robot robot) {
    super(robot);
    m_autoCommand = m_robotContainer.getAutonomousCommand(AutoList.MOVE_LEFT_BUMP);
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
