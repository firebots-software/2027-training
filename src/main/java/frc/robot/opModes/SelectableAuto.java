package frc.robot.opModes;

import org.wpilib.command2.Command;
import org.wpilib.tunable.Selectable;

import choreo.trajectory.Trajectory;
import frc.robot.opModes.autos.LeftReturnShoot;

public class SelectableAuto {
    private final Selectable<Trajectory> m_autoChooser =
    new Selectable<>();
    public RobotContainer() {
        m_autoChooser.add("", new LeftReturnShoot(null));
        m_autoChooser.add("",);
    }

    
}
