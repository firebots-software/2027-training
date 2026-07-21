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
    private DutyCycleOut m_dutyCycleRequest;
    private VoltageOut m_voltageRequest;
    private TorqueCurrentFOC m_torqueRequest;

    public ArmSubsystem() {
        armMotor = new TalonFX(Constants.Arm.ARM_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Arm.ROLLER_MOTOR_ID);
        m_dutyCycleRequest = new DutyCycleOut(0.0);
        m_voltageRequest = new VoltageOut(0.0);
        m_torqueRequest = new TorqueCurrentFOC(0.0);
    }
    
    public void setRollerDutyCycle(double power) {
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(power));
    }

    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(m_voltageRequest.withOutput(voltage));
    }

    public void setArmTorqueCurrent(double torque) {
        armMotor.setControl(m_torqueRequest.withOutput(torque));
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