// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;



public class ArmSubsystem extends SubsystemBase {
    private TalonFX armMotor, rollerMotor;
    
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0);
    private final VoltageOut m_voltageRequest = new VoltageOut(0.0);
    private final TorqueCurrentFOC m_torqueRequest = new TorqueCurrentFOC(0.0);
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0);
    private final VoltageOut m_voltageRequest = new VoltageOut(0.0);
    private final TorqueCurrentFOC m_torqueRequest = new TorqueCurrentFOC(0.0);

    public ArmSubsystem() {
        armMotor = new TalonFX(Constants.ArmConstants.kArmMotorId);
        rollerMotor = new TalonFX(Constants.ArmConstants.kRollerMotorId);

    }
    
    public void setRollerDutyCycle(double output) {
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(output));
    }

    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(m_voltageRequest.withOutput(volts));
    }

    public void setArmTorqueCurrent(double currentAmps) {
        armMotor.setControl(m_torqueRequest.withOutput(currentAmps));
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