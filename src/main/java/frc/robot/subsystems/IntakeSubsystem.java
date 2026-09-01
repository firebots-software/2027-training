package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX armMotor, rollerMotor;

    private final VelocityDutyCycle velocityVoltage = new VelocityDutyCycle(0.0);
    private final PositionDutyCycle positionVoltage = new PositionDutyCycle(0.0);

    public IntakeSubsystem() {
        armMotor = new TalonFX(14);
        rollerMotor = new TalonFX(16);

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
            .withCurrentLimits(armClc)
            .withMotorOutput(armMoc);

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
            .withCurrentLimits(rollerClc)
            .withMotorOutput(rollerMoc);

        TalonFXConfigurator rollerConfigurator = rollerMotor.getConfigurator();
        TalonFXConfigurator armConfigurator = armMotor.getConfigurator();

        rollerConfigurator.apply(rollerConfiguration);
        armConfigurator.apply(armConfiguration);
    }

    public void setArmAngle() {
        
    }
}
