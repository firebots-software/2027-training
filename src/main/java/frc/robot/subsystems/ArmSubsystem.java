// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX armMotor, rollerMotor;
    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0);
    private VoltageOut m_voltageRequest = new VoltageOut(0.0);
    private TorqueCurrentFOC m_torqueRequest = new TorqueCurrentFOC(0.0);

    public ArmSubsystem() {
        armMotor = new TalonFX(Constants.Arm.ARM_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Arm.ROLLER_MOTOR_ID);
    }

    public void setRollerDutyCycle(double output) {
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(output));
    }

    public void setRollerVoltage(double output) {
        rollerMotor.setControl(m_voltageRequest.withOutput(output));
    }

    public void setArmTorqueCurrent(double output) {
        armMotor.setControl(m_torqueRequest.withOutput(output));
    }
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}