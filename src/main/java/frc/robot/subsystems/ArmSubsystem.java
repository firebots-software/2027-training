// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        armMotor = new TalonFX(16);
        rollerMotor = new TalonFX(15);
    }
    public void setRollerDutyCycle(double outputVal){
        rollerMotor.setControl(m_dutyCycleRequest.withOutput(outputVal));
    }
    public void setRollerVoltage(double voltageVal){
        rollerMotor.setControl(m_voltageRequest.withOutput(voltageVal));
    }
    public void setArmTorqueCurrent(double torqueVal){
        armMotor.setControl(m_torqueRequest.withOutput(torqueVal));
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