package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX armMotor, rollerMotor;

    private final VelocityDutyCycle velocityVoltage = new VelocityDutyCycle(0.0);
    private final PositionDutyCycle positionVoltage = new PositionDutyCycle(0.0);

    private double targetArmAngle, targetRollerSpeed;

    public IntakeSubsystem() {
        armMotor = new TalonFX(14);
        rollerMotor = new TalonFX(16);

        Slot0Configs armS0C =
            new Slot0Configs()
            .withKV(Constants.Intake.Arm.kV)
            .withKP(Constants.Intake.Arm.kP)
            .withKI(Constants.Intake.Arm.kI)
            .withKD(Constants.Intake.Arm.kD)
            .withKG(Constants.Intake.Arm.kG)
            .withKS(Constants.Intake.Arm.kV);

        CurrentLimitsConfigs armClc = 
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Arm.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Intake.Arm.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        
        MotorOutputConfigs armMoc =
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration armConfiguration =
            new TalonFXConfiguration()
            .withSlot0(armS0C)
            .withCurrentLimits(armClc)
            .withMotorOutput(armMoc);

        Slot0Configs rollerS0C =
            new Slot0Configs()
            .withKV(Constants.Intake.Rollers.kV)
            .withKP(Constants.Intake.Rollers.kP)
            .withKI(Constants.Intake.Rollers.kI)
            .withKD(Constants.Intake.Rollers.kD);

        CurrentLimitsConfigs rollerClc =
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs rollerMoc =
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration rollerConfiguration = 
            new TalonFXConfiguration()
            .withSlot0(rollerS0C)
            .withCurrentLimits(rollerClc)
            .withMotorOutput(rollerMoc);

        TalonFXConfigurator rollerConfigurator = rollerMotor.getConfigurator();
        TalonFXConfigurator armConfigurator = armMotor.getConfigurator();

        rollerConfigurator.apply(rollerConfiguration);
        armConfigurator.apply(armConfiguration);
    }

    public void setArmAngle(double deg) {
        targetArmAngle = deg;
        deg = MathUtil.clamp(deg, Constants.Intake.Arm.ARM_POS_MIN, Constants.Intake.Arm.ARM_POS_MAX);
        armMotor.setControl(positionVoltage.withPosition(deg / 360));
    }

    public void setRollerSpeed(double velocityRps) {
        targetRollerSpeed = velocityRps;
        rollerMotor.setControl(
            velocityVoltage.withVelocity(
                velocityRps * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT));
    }

    public void stopRollers() {
        targetRollerSpeed = 0.0;
        rollerMotor.stopMotor();
    }

    public Command runIntake(double targetRollerSpeed, double targetArmAngle) {
        return runEnd(
            () -> {
                setRollerSpeed(targetRollerSpeed);
                setArmAngle(targetArmAngle);
            },
            this::stopRollers);
    }
}
