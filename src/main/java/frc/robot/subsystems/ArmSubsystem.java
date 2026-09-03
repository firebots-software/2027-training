// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
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