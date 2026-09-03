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
    private final DutyCycleOut m_DutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut m_VoltageRequest = new VoltageOut(0);
    private final TorqueCurrentFOC m_TorqueRequest = new TorqueCurrentFOC(0);

    // initializing class
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
                .withNeutralMode(NeutralModeValue.Coast);

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
                .withCurrentLimits(armClc)
                .withMotorOutput(armMotorOutputConfigs);

        TalonFXConfigurator rollerConfigurator = rollerMotor.getConfigurator();
        TalonFXConfigurator armConfigurator = armMotor.getConfigurator();

        rollerConfigurator.apply(rollerConfig);
        armConfigurator.apply(armConfig);
        
    }

    public void setRollerDutyCycle(Double prcnt) {
        prcnt = Math.min(1, prcnt);
        rollerMotor.setControl(m_DutyCycleRequest.withOutput(prcnt));
    }

    public void setRollerVoltage(Double vltge) {
        rollerMotor.setControl(m_VoltageRequest.withOutput(vltge));
     }

   
    

    public void setArmTorqueCurrent (Double current) {
        armMotor.setControl(m_TorqueRequest.withOutput(current));
    }

}