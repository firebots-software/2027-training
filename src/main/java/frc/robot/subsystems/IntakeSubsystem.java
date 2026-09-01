package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX armMotor, rollerMotor;

    private double target_roller_speed, target_arm_angle;

    private final VelocityVoltage m_velocityVoltageRequest = new VelocityVoltage(0.0);
    private final PositionVoltage m_positionVoltageRequest = new PositionVoltage(0.0);

    public IntakeSubsystem() {
        armMotor = new TalonFX(Constants.Intake.Arm.CAN_ID);
        rollerMotor = new TalonFX(Constants.Intake.Rollers.CAN_ID);

        CurrentLimitsConfigs rollerClc = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs armClc = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Intake.Arm.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Intake.Arm.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs rollerMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        MotorOutputConfigs armMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        Slot0Configs rollersSlot0Configs = new Slot0Configs()
                .withKP(Constants.Intake.Rollers.kP)
                .withKV(Constants.Intake.Rollers.kV)
                .withKI(Constants.Intake.Rollers.kI)
                .withKD(Constants.Intake.Rollers.kD);

        Slot0Configs armSlot0Configs = new Slot0Configs()
                .withKP(Constants.Intake.Arm.kP)
                .withKI(Constants.Intake.Arm.kI)
                .withKD(Constants.Intake.Arm.kD)
                .withKV(Constants.Intake.Arm.kV)
                .withKS(Constants.Intake.Arm.kS)
                .withKG(Constants.Intake.Arm.kG);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
                .withSlot0(rollersSlot0Configs)
                .withCurrentLimits(rollerClc)
                .withMotorOutput(rollerMotorOutputConfigs);

        TalonFXConfiguration armConfig = new TalonFXConfiguration()
                .withSlot0(armSlot0Configs)
                .withCurrentLimits(armClc)
                .withMotorOutput(armMotorOutputConfigs);

        TalonFXConfigurator rollerConfigurator = rollerMotor.getConfigurator();
        TalonFXConfigurator armConfigurator = armMotor.getConfigurator();

        rollerConfigurator.apply(rollerConfig);
        armConfigurator.apply(armConfig);
    }

    public void setArmAngle(double degrees) {
        target_arm_angle = degrees;
        armMotor.setControl(m_positionVoltageRequest.withPosition(MathUtil.clamp(target_arm_angle / 360.0, Constants.Intake.Arm.ARM_POS_MIN, Constants.Intake.Arm.ARM_POS_MAX)))
    }

    public void setRollerSpeed(double vel_rps) {
        target_roller_speed = vel_rps;
        rollerMotor.setControl(m_velocityVoltageRequest.withVelocity(vel_rps * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT))
    }

    public void stopRollers() {
        target_roller_speed = 0.0;
        rollerMotor.stopMotor();
    }

    public boolean areRollersAtSpeed() {
        double current_v = rollerMotor.getVelocity().getValueAsDouble()
                / Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT;
        return Math.abs(current_v - target_roller_speed) <= 1.0;
    }

    public Command intake(double target_roller_speed, double target_arm_pos) {
        return runEnd(
                () -> {
                    setArmAngle(target_arm_pos);
                    setRollerSpeed(target_arm_pos);
                },
                this::stopRollers);
    }

}
