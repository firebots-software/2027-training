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

    public ArmSubsystem() {
        armMotor = new TalonFX(Constants.Arm.ARM_MOTOR_ID);
        rollerMotor = new TalonFX(Constants.Arm.ROLLER_MOTOR_ID);

        CurrentLimitsConfigs rollerClc = 
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50)
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(false);
        
        CurrentLimitsConfigs armClc =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs rollerMotorOutputConfigs =
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        MotorOutputConfigs armMotorOutputConfigs =
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration rollerConfig =
            new TalonFXConfiguration()
                .withCurrentLimits(rollerClc)
                .withMotorOutput(rollerMotorOutputConfigs);

        TalonFXConfiguration armConfig =
            new TalonFXConfiguration()
                .withCurrentLimits(rollerClc)
                .withMotorOutput(rollerMotorOutputConfigs);

        TalonFXConfigurator rollerConfigurator = rollerMotor.getConfigurator();
        TalonFXConfigurator armConfigurator = armMotor.getConfigurator();

        rollerConfigurator.apply(rollerConfig);
        armConfigurator.apply(armConfig);

    }

    public void setRollerDutyCycle(double voltage) {
        rollerMotor.setControl(m_voltageRequest.withOutput(voltage));
    }

    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(voltage));
    }

    public void setArmTorqueCurrent(double current) {
        armMotor.setControl(m_torqueRequest.withOutput(current));
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setRollerDutyCycle(double DutyCycleValue) {
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(DutyCycleValue));
    }

    public void setRollerVoltage(double VoltageOutValue) {
        rollerMotor.setControl(m_voltageRequest.withOutput(VoltageOutValue));
    }

    public void setArmTorqueCurrent(double TorqueValue) {
        armMotor.setControl(m_torqueRequest.withOutput(TorqueValue));
    }
}