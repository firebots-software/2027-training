package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX armMotor, rollerMotor; 


public void setArmPosition(double targetArmPosition) {
    armMotor.setPosition(targetArmPosition);
}

public void stopRoller() {
    rollerMotor.set(0.0);
}

public double getRollerMotorVelocity() {
    return rollerMotor.getVelocity().getValueAsDouble();
}

public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble() * Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT;
}


}








