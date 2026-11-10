package frc.robot.opModes.autos;

import java.util.Map;

import frc.robot.Constants.Swerve.Auto.AutoList;
import frc.robot.Robot;
import frc.robot.opModes.BaseOpMode;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.tunable.Selectable;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

@Autonomous(name = "Right Intake Shoot", group = "Custom")
public class SelectableAuto extends BaseOpMode {

  private final AutoFactory autoFactory;

  private Command m_autoCommand;

  private Map<String[], String> ShootMap = Map.of(
    new String[]{"CenterIntake", "CenterShoot"}, "CenterCenter",
    new String[]{"CenterIntake", "RightShoot"}, "CenterRightBump",
    new String[]{"CenterIntake", "LeftShoot"}, "CenterLeftBump",
    new String[]{"RightEndIntake", "CenterShoot"}, "RightEndCenter",
    new String[]{"RightEndIntake", "RightShoot"}, "RightRightBump",
    new String[]{"RightEndIntake", "LeftShoot"}, "RightLeftBump",
    new String[]{"LeftEndIntake", "CenterShoot"}, "LeftEndCenter",
    new String[]{"LeftEndIntake", "RightShoot"}, "LeftRightBump",
    new String[]{"LeftEndIntake", "LeftShoot"}, "LeftLeftBump"
  );


  public SelectableAuto(Robot robot) {
    super(robot);
    
    //autoFactory = swerveSubsystem.createAutoFactory();
  }

  @Override
  public void start() {
    super.start();
    if (m_autoCommand !== null ) {
      CommandScheduler.getInstance().schedule(autoChooser);
    }
  }



  @Override
  public void end() {
    if (m_autoCommand != null) {
      CommandScheduler.getInstance().cancel(m_autoCommand);
    }
  }

  public void loadAuto() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");
    
  }

}
