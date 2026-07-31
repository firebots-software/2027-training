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
    private DutyCycleOut m_dutyCycleRequest;
    private VoltageOut m_voltageRequest;
    private TorqueCurrentFOC m_torqueRequest;

    public ArmSubsystem() {
        m_dutyCycleRequest = new DutyCycleOut(0.0);
        m_voltageRequest = new VoltageOut(0.0);
        m_torqueRequest = new TorqueCurrentFOC(0.0);
        armMotor = new TalonFX(Constants.Arm.ARM_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Arm.ROLLER_MOTOR_ID);
        CurrentLimitsConfigs rollerCurrentConfig = new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(30);
        MotorOutputConfigs rollerMotorConfig = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast);

        CurrentLimitsConfigs armCurrentConfig = new CurrentLimitsConfigs().withStatorCurrentLimit(100).withSupplyCurrentLimit(50);
        MotorOutputConfigs armMotorConfig = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        
    }

    public void setRollerDutyCycle(double val){
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(val));
    }

    public void setRollerVoltage(double val){
        rollerMotor.setControl(m_voltageRequest.withOutput(val));
    }

    public void setArmTorqueCurrent(double val){
        armMotor.setControl(m_torqueRequest.withOutput(val));
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