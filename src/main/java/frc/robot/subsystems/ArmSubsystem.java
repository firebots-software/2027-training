// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX armMotor, rollersMotor;
    private CANcoder cancoder;

    private final PositionVoltage m_positionRequest = new PositionVoltage(0.0);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0.0);

    public ArmSubsystem() {
        armMotor = new TalonFX(14);
        rollersMotor = new TalonFX(16);

        Slot0Configs rollersSlot0Configs = new Slot0Configs()
                .withKV(Constants.Intake.Rollers.kV)
                .withKP(Constants.Intake.Rollers.kP)
                .withKI(Constants.Intake.Rollers.kI)
                .withKD(Constants.Intake.Rollers.kD);

        Slot0Configs armSlot0Configs = new Slot0Configs()
                .withKV(Constants.Intake.Arm.kV)
                .withKP(Constants.Intake.Arm.kP)
                .withKI(Constants.Intake.Arm.kI)
                .withKD(Constants.Intake.Arm.kD)
                .withKG(Constants.Intake.Arm.kG)
                .withGravityArmPositionOffset(Constants.Intake.Arm.GRAVITY_POS_OFFSET)
                .withKS(Constants.Intake.Arm.kS)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        CurrentLimitsConfigs rollersCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT);

        CurrentLimitsConfigs armCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.Arm.STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(Constants.Intake.Arm.SUPPLY_CURRENT_LIMIT);

        MotionMagicConfigs mmc = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Intake.Arm.mmcV)
                .withMotionMagicAcceleration(Constants.Intake.Arm.mmcA);

        ClosedLoopRampsConfigs clrc = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);

        // Creates a FusedCANcoder, which combines data from the CANcoder and the arm
        // motor's encoder
        cancoder = new CANcoder(Constants.Intake.Arm.ENCODER_PORT, Constants.Swerve.CAN_BUS);
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(Rotations.of(Constants.Intake.Arm.ENCODER_OFFSET));

        cancoder.getConfigurator().apply(ccConfig);
        cancoder.getConfigurator().apply(magnetSensorConfigs);

        // Add the CANcoder as a feedback source for the motor's built-in encoder
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(cancoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(
                        Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT
                                / Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT)
                .withSensorToMechanismRatio(Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT);

        TalonFXConfiguration rollersConfig = new TalonFXConfiguration()
                .withSlot0(rollersSlot0Configs)
                .withCurrentLimits(rollersCurrentLimitsConfigs)
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
                .withClosedLoopRamps(clrc);

        TalonFXConfiguration armConfig = new TalonFXConfiguration()
                .withSlot0(armSlot0Configs)
                .withMotionMagic(mmc)
                .withCurrentLimits(armCurrentLimitsConfigs)
                .withFeedback(feedbackConfigs)
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake));

        TalonFXConfigurator armMotorConfig = armMotor.getConfigurator();
        TalonFXConfigurator rollersMotorConfig = rollersMotor.getConfigurator();

        armMotorConfig.apply(armConfig);
        rollersMotorConfig.apply(rollersConfig);

    }

    public double getRollerSpeed() {
        return rollersMotor.getVelocity().getValueAsDouble();
    }

    public double getArmPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public void setRollerSpeed(double speed) {
        rollersMotor.setControl(
                m_velocityRequest.withVelocity(speed * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT));
    }

    public void setArmPosition(double armPos) {
        double degs = MathUtil.clamp(armPos, Constants.Intake.Arm.ARM_POS_MIN, Constants.Intake.Arm.ARM_POS_MAX) / 360.0;
        armMotor.setControl(m_positionRequest.withPosition(degs));
    }
}